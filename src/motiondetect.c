/*
 * motiondetect.c
 *
 *  Copyright (C) Georg Martius - February 1007-2011
 *   georg dot martius at web dot de
 *  Copyright (C) Alexey Osipov - Jule 2011
 *   simba at lerlan dot ru
 *   speed optimizations (threshold, spiral, SSE, asm)
 *
 *  This file is part of vid.stab video stabilization library
 *
 *  vid.stab is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License,
 *  as published by the Free Software Foundation; either version 2, or
 *  (at your option) any later version.
 *
 *  vid.stab is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with GNU Make; see the file COPYING.  If not, write to
 *  the Free Software Foundation, 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */
#include "motiondetect.h"
#include "motiondetect_internal.h"
#include "motiondetect_opt.h"
#include <math.h>
#include <limits.h>
#include <stdint.h>
#include <string.h>
#include <assert.h>

#ifdef USE_OMP
#include <omp.h>
#endif

#include "boxblur.h"
#include "vidstabdefines.h"
#include "localmotion2transform.h"
#include "transformtype_operations.h"

#define USE_SPIRAL_FIELD_CALC


/* internal data structures */

// structure that contains the contrast and the index of a field
typedef struct _contrast_idx {
  double contrast;
  int index;
} contrast_idx;

int vsMotionDetectInit(VSMotionDetect* md, const VSFrameInfo* fi,
         const char* modName) {
  assert(md && fi);
  md->fi = *fi;
  md->modName = modName;

  if(fi->pFormat<=PF_NONE ||  fi->pFormat==PF_PACKED || fi->pFormat>=PF_NUMBER) {
    vs_log_warn(md->modName, "unsupported Pixel Format (%i)\n",
                md->fi.pFormat);
    return VS_ERROR;
  }

  vsFrameAllocate(&md->prev, &md->fi);
  if (vsFrameIsNull(&md->prev)) {
    vs_log_error(md->modName, "malloc failed");
    return VS_ERROR;
  }

  vsFrameNull(&md->curr);
  vsFrameNull(&md->currorig);
  vsFrameNull(&md->currtmp);
  md->hasSeenOneFrame = 0;
  md->frameNum = 0;

  // Options
  md->stepSize  = 6;
  md->allowMax  = 0;
  md->algo      = 1;
  md->accuracy  = 9;
  md->shakiness = 5;
  md->fieldSize = VS_MIN(md->fi.width, md->fi.height) / 12;
  md->virtualTripod = 0;
  md->show = 0;
  md->contrastThreshold = 0.25;
  md->initialized = 1;
  return VS_OK;
}

int vsMotionDetectConfigure(VSMotionDetect* md) {
  if (md->initialized != 1)
    return VS_ERROR;

  md->shakiness = VS_MIN(10,VS_MAX(1,md->shakiness));
  md->accuracy = VS_MIN(15,VS_MAX(1,md->accuracy));
  if (md->accuracy < md->shakiness / 2) {
    vs_log_info(md->modName, "Accuracy should not be lower than shakiness/2 -- fixed");
    md->accuracy = md->shakiness / 2;
  }
  if (md->accuracy > 9 && md->stepSize > 6) {
    vs_log_info(md->modName, "For high accuracy use lower stepsize  -- set to 6 now");
    md->stepSize = 6; // maybe 4
  }

  // shift: shakiness 1: height/40; 10: height/4
  int minDimension = VS_MIN(md->fi.width, md->fi.height);
  md->maxShift
    = VS_MAX(4,(minDimension*md->shakiness)/40);
  // size: shakiness 1: height/40; 10: height/6 (clipped)
  md->fieldSize
    = VS_MAX(4,VS_MIN(minDimension/6, (minDimension*md->shakiness)/40));

#if defined(USE_SSE2) || defined(USE_SSE2_ASM)
  md->fieldSize = (md->fieldSize / 16 + 1) * 16;
#endif

  vs_log_info(md->modName, "Fieldsize: %i, Maximal translation: %i pixel\n",
        md->fieldSize, md->maxShift);
  if (md->algo == 1) {
    // initialize measurement fields. field_num is set here.
    if (!initFields(md)) {
      return VS_ERROR;
    }
    md->maxFields = (md->accuracy) * md->fieldNum / 15;
    vs_log_info(md->modName, "Number of used measurement fields: %i out of %i\n",
    md->maxFields, md->fieldNum);
  }
  //  if (md->show)
  vsFrameAllocate(&md->curr,&md->fi);
  vsFrameAllocate(&md->currtmp, &md->fi);

  md->initialized = 2;
  return VS_OK;
}

void vsMotionDetectionCleanup(VSMotionDetect* md) {
  if(md->fields) {
    vs_free(md->fields);
    md->fields=0;
  }
  vsFrameFree(&md->prev);
  vsFrameFree(&md->curr);
  vsFrameFree(&md->currtmp);

  md->initialized = 0;
}


int vsMotionDetection(VSMotionDetect* md, LocalMotions* motions, VSFrame *frame) {
  assert(md->initialized==2);

  md->currorig = *frame;
  // smoothen image to do better motion detection
  //  (larger stepsize or eventually gradient descent (need higher resolution))
  if (md->fi.pFormat > PF_PACKED) {
    // we could calculate a grayscale version and use the PLANAR stuff afterwards
    // so far smoothing is only implemented for PLANAR
    vsFrameCopy(&md->curr, frame, &md->fi);
  } else {
    // box-kernel smoothing (plain average of pixels), which is fine for us
    boxblurPlanar(&md->curr, frame, &md->currtmp, &md->fi, md->stepSize*1/*1.4*/,
               BoxBlurNoColor);
    // two times yields tent-kernel smoothing, which may be better, but I don't
    //  think we need it
    //boxblurPlanar(md->curr, md->curr, md->currtmp, &md->fi, md->stepSize*1,
    // BoxBlurNoColor);
  }

  if (md->hasSeenOneFrame) {
    //    md->curr = frame;
    if (md->fi.pFormat > PF_PACKED) {
      if (md->algo == 0)
        *motions = calcShiftPackedSimple(md);
      else if (md->algo == 1)
        *motions = calcTransFields(md, calcFieldTransPacked, contrastSubImgPacked);
    } else { // PLANAR
      if (md->algo == 0)
        *motions = calcShiftPlanarSimple(md);
      else if (md->algo == 1)
        *motions = calcTransFields(md, calcFieldTransPlanar, contrastSubImgPlanar);
    }
  } else {
    vs_vector_init(motions,md->maxFields);
    md->hasSeenOneFrame = 1;
  }

  if(md->virtualTripod < 1 || md->frameNum < md->virtualTripod)
  // copy current frame (smoothed) to prev for next frame comparison
  vsFrameCopy(&md->prev, &md->curr, &md->fi);
  md->frameNum++;
  return VS_OK;
}


/** initialise measurement fields on the frame.
    The size of the fields and the maxshift is used to
    calculate an optimal distribution in the frame.
*/
int initFields(VSMotionDetect* md) {
  int size = md->fieldSize;
  int rows = VS_MAX(3,(md->fi.height - md->maxShift*2)/size-1);
  int cols = VS_MAX(3,(md->fi.width - md->maxShift*2)/size-1);
  // make sure that the remaining rows have the same length
  md->fieldNum = rows * cols;
  md->fieldRows = rows;
  // vs_log_msg(md->modName, "field setup: rows: %i cols: %i Total: %i fields",
  //            rows, cols, md->field_num);

  if (!(md->fields = (Field*) vs_malloc(sizeof(Field) * md->fieldNum))) {
    vs_log_error(md->modName, "malloc failed!\n");
    return 0;
  } else {
    int i, j;
    // the border is the amount by which the field centers
    // have to be away from the image boundary
    // (stepsize is added in case shift is increased through stepsize)
    int border = size / 2 + md->maxShift + md->stepSize;
    int step_x = (md->fi.width - 2 * border) / VS_MAX(cols-1,1);
    int step_y = (md->fi.height - 2 * border) / VS_MAX(rows-1,1);
    for (j = 0; j < rows; j++) {
      for (i = 0; i < cols; i++) {
        int idx = j * cols + i;
        md->fields[idx].x = border + i * step_x;
        md->fields[idx].y = border + j * step_y;
        md->fields[idx].size = size;
      }
    }
  }
  return 1;
}

/**
   This routine is used in the simpleAlgorithms and may be removed at some point
   compares the two given images and returns the average absolute difference
   \param strive1 linesize of image 1 including bytesPerPixel
   \param strive2 linesize of image 2 including bytesPerPixel
   \param d_x shift in x direction
   \param d_y shift in y direction
*/
unsigned int compareImg(unsigned char* I1, unsigned char* I2, int width, int height,
                        int bytesPerPixel, int strive1, int strive2, int d_x, int d_y) {
  int i, j;
  unsigned char* p1 = NULL;
  unsigned char* p2 = NULL;
  unsigned int sum = 0;
  int effectWidth = width - abs(d_x);
  int effectHeight = height - abs(d_y);
  assert(width*bytesPerPixel <= strive1);
  assert(width*bytesPerPixel <= strive2);

  /*//   DEBUGGING code to export single frames */
  /*   char buffer[100]; */
  /*   sprintf(buffer, "pic_%02ix%02i_1.ppm", d_x, d_y); */
  /*   FILE *pic1 = fopen(buffer, "w"); */
  /*   sprintf(buffer, "pic_%02ix%02i_2.ppm", d_x, d_y); */
  /*   FILE *pic2 = fopen(buffer, "w"); */
  /*   fprintf(pic1, "P6\n%i %i\n255\n", effectWidth, effectHeight); */
  /*   fprintf(pic2, "P6\n%i %i\n255\n", effectWidth, effectHeight); */

  for (i = 0; i < effectHeight; i++) {
    p1 = I1;
    p2 = I2;
    if (d_y > 0) {
      p1 += (i + d_y) * strive1;
      p2 += i * strive2;
    } else {
      p1 += i * strive1;
      p2 += (i - d_y) * strive2;
    }
    if (d_x > 0) {
      p1 += d_x * bytesPerPixel;
    } else {
      p2 -= d_x * bytesPerPixel;
    }
    for (j = 0; j < effectWidth * bytesPerPixel; j++) {
      /*// debugging code continued */
      /* fwrite(p1,1,1,pic1);fwrite(p1,1,1,pic1);fwrite(p1,1,1,pic1);
         fwrite(p2,1,1,pic2);fwrite(p2,1,1,pic2);fwrite(p2,1,1,pic2);
      */
      sum += abs((int) *p1 - (int) *p2);
      p1++;
      p2++;
    }
  }
  /*  fclose(pic1);
      fclose(pic2);
  */
  return sum;
}


/** \see contrastSubImg*/
double contrastSubImgPlanar(VSMotionDetect* md, const Field* field) {
#ifdef USE_SSE2
  return contrastSubImg1_SSE(md->curr.data[0], field, md->curr.linesize[0],md->fi.height);
#else
  return contrastSubImg(md->curr.data[0],field,md->curr.linesize[0],md->fi.height,1);
#endif

}

/**
   \see contrastSubImg_Michelson three times called with bytesPerPixel=3
   for all channels
*/
double contrastSubImgPacked(VSMotionDetect* md, const Field* field) {
  unsigned char* const I = md->curr.data[0];
  int linesize2 = md->curr.linesize[0]/3; // linesize in pixels
  return (contrastSubImg(I, field, linesize2, md->fi.height, 3)
          + contrastSubImg(I + 1, field, linesize2, md->fi.height, 3)
          + contrastSubImg(I + 2, field, linesize2, md->fi.height, 3)) / 3;
}

/**
   calculates Michelson-contrast in the given small part of the given image
   to be more compatible with the absolute difference formula this is scaled by 0.1

   \param I pointer to framebuffer
   \param field Field specifies position(center) and size of subimage
   \param width width of frame (linesize in pixels)
   \param height height of frame
   \param bytesPerPixel calc contrast for only for first channel
*/
double contrastSubImg(unsigned char* const I, const Field* field, int width,
                      int height, int bytesPerPixel) {
  int k, j;
  unsigned char* p = NULL;
  int s2 = field->size / 2;
  unsigned char mini = 255;
  unsigned char maxi = 0;

  p = I + ((field->x - s2) + (field->y - s2) * width) * bytesPerPixel;
  for (j = 0; j < field->size; j++) {
    for (k = 0; k < field->size; k++) {
      mini = (mini < *p) ? mini : *p;
      maxi = (maxi > *p) ? maxi : *p;
      p += bytesPerPixel;
    }
    p += (width - field->size) * bytesPerPixel;
  }
  return (maxi - mini) / (maxi + mini + 0.1); // +0.1 to avoid division by 0
}


/** tries to register current frame onto previous frame.
    This is the most simple algorithm:
    shift images to all possible positions and calc summed error
    Shift with minimal error is selected.
*/
LocalMotions calcShiftPackedSimple(VSMotionDetect* md) {
  LocalMotions localmotions;
  vs_vector_init(&localmotions,1);
  LocalMotion lm;
  int i, j;
  int minerror =  INT_MAX;
  lm.v.x = 0, lm.v.y = 0;
  for (i = -md->maxShift; i <= md->maxShift; i++) {
    for (j = -md->maxShift; j <= md->maxShift; j++) {
      int error = compareImg(md->curr.data[0], md->prev.data[0], md->fi.width,
                             md->fi.height, 3, md->curr.linesize[0], md->prev.linesize[0],
                             i, j);
      // FIXME: linesize is not honored
      if (error < minerror) {
        minerror = error;
        lm.v.x = i;
        lm.v.y = j;
      }
    }
  }
  lm.f.x=0;
  lm.f.y=0;
  lm.match=minerror;
  lm.contrast=1;
  vs_vector_append_dup(&localmotions,&lm,sizeof(LocalMotion));
  return localmotions;
}

/** tries to register current frame onto previous frame.
    (only the luminance is used)
    This is the most simple algorithm:
    shift images to all possible positions and calc summed error
    Shift with minimal error is selected.
*/
LocalMotions calcShiftPlanarSimple(VSMotionDetect* md) {
  LocalMotions localmotions;
  vs_vector_init(&localmotions,1);
  LocalMotion lm;
  int i, j;
#ifdef STABVERBOSE
  FILE *f = NULL;
  char buffer[32];
  vs_snprintf(buffer, sizeof(buffer), "f%04i.dat", md->frameNum);
  f = fopen(buffer, "w");
  fprintf(f, "# splot \"%s\"\n", buffer);
#endif
  lm.v.x = 0, lm.v.y = 0;

  // we only use the luminance part of the image
  int minerror = INT_MAX;
  for (i = -md->maxShift; i <= md->maxShift; i++) {
    for (j = -md->maxShift; j <= md->maxShift; j++) {
      int error = compareImg(md->curr.data[0], md->prev.data[0],
                             md->fi.width, md->fi.height, 1,
                             md->curr.linesize[0], md->prev.linesize[0],
                             i, j);
#ifdef STABVERBOSE
      fprintf(f, "%i %i %f\n", i, j, error);
#endif
      if (error < minerror) {
        minerror = error;
        lm.v.x = i;
        lm.v.y = j;
      }
    }
  }
#ifdef STABVERBOSE
  fclose(f);
  vs_log_msg(md->modName, "Minerror: %f\n", minerror);
#endif
  lm.f.x=0;
  lm.f.y=0;
  lm.match=minerror;
  lm.contrast=1;
  vs_vector_append_dup(&localmotions,&lm,sizeof(LocalMotion));
  return localmotions;
}

/* calculates the optimal transformation for one field in Planar frames
 * (only luminance)
 */
LocalMotion calcFieldTransPlanar(VSMotionDetect* md, const Field* field, int fieldnum) {
  int tx = 0;
  int ty = 0;
  uint8_t *Y_c = md->curr.data[0], *Y_p = md->prev.data[0];
  int linesize_c = md->curr.linesize[0], linesize_p = md->prev.linesize[0];
  // we only use the luminance part of the image
  int i, j;
  int stepSize = md->stepSize;

#ifdef STABVERBOSE
  // printf("%i %i %f\n", md->frameNum, fieldnum, contr);
  FILE *f = NULL;
  char buffer[32];
  vs_snprintf(buffer, sizeof(buffer), "f%04i_%02i.dat", md->frameNum, fieldnum);
  f = fopen(buffer, "w");
  fprintf(f, "# splot \"%s\"\n", buffer);
#endif

#ifdef USE_SPIRAL_FIELD_CALC
  unsigned int minerror = UINT_MAX;

  // check all positions by outgoing spiral
  i = 0; j = 0;
  int limit = 1;
  int step = 0;
  int dir = 0;
  while (j >= -md->maxShift && j <= md->maxShift && i >= -md->maxShift && i <= md->maxShift) {
    unsigned int error = compareSubImg(Y_c, Y_p, field, linesize_c, linesize_p,
                                       md->fi.height, 1, i, j, minerror);

    if (error < minerror) {
      minerror = error;
      tx = i;
      ty = j;
    }

    //spiral indexing...
    step++;
    switch (dir) {
     case 0:
      i += stepSize;
      if (step == limit) {
        dir = 1;
        step = 0;
      }
      break;
     case 1:
      j += stepSize;
      if (step == limit) {
        dir = 2;
        step = 0;
        limit++;
      }
      break;
     case 2:
      i -= stepSize;
      if (step == limit) {
        dir = 3;
        step = 0;
      }
      break;
     case 3:
      j -= stepSize;
      if (step == limit) {
        dir = 0;
        step = 0;
        limit++;
      }
      break;
    }
  }
#else
  /* Here we improve speed by checking first the most probable position
     then the search paths are most effectively cut. (0,0) is a simple start
  */
  unsigned int minerror = compareSubImg(Y_c, Y_p, field, linesize_c, linesize_p,
                                        md->fi.height, 1, 0, 0, UINT_MAX);
  // check all positions...
  for (i = -md->maxShift; i <= md->maxShift; i += stepSize) {
    for (j = -md->maxShift; j <= md->maxShift; j += stepSize) {
      if( i==0 && j==0 )
        continue; //no need to check this since already done
      unsigned int error = compareSubImg(Y_c, Y_p, field, linesize_c, linesize_p,
                                         md->fi.height, 1, i, j, minerror);
      if (error < minerror) {
        minerror = error;
        tx = i;
        ty = j;
      }
#ifdef STABVERBOSE
      fprintf(f, "%i %i %f\n", i, j, error);
#endif
    }
  }

#endif

  while(stepSize > 1) {// make fine grain check around the best match
    int txc = tx; // save the shifts
    int tyc = ty;
    int newStepSize = stepSize/2;
    int r = stepSize - newStepSize;
    for (i = txc - r; i <= txc + r; i += newStepSize) {
      for (j = tyc - r; j <= tyc + r; j += newStepSize) {
        if (i == txc && j == tyc)
          continue; //no need to check this since already done
        unsigned int error = compareSubImg(Y_c, Y_p, field, linesize_c, linesize_p,
                                           md->fi.height, 1, i, j, minerror);
#ifdef STABVERBOSE
        fprintf(f, "%i %i %f\n", i, j, error);
#endif
        if (error < minerror) {
          minerror = error;
          tx = i;
          ty = j;
        }
      }
    }
    stepSize /= 2;
  }
#ifdef STABVERBOSE
  fclose(f);
  vs_log_msg(md->modName, "Minerror: %f\n", minerror);
#endif

  if (!md->allowMax && fabs(tx) >= md->maxShift + md->stepSize) {
#ifdef STABVERBOSE
    vs_log_msg(md->modName, "maximal x shift ");
#endif
    tx = 0;
  }
  if (!md->allowMax && fabs(ty) == md->maxShift + md->stepSize) {
#ifdef STABVERBOSE
    vs_log_msg(md->modName, "maximal y shift ");
#endif
    ty = 0;
  }
  LocalMotion lm = null_localmotion();
  lm.f = *field;
  lm.v.x = tx;
  lm.v.y = ty;
  lm.match = minerror;
  return lm;
}

/* calculates the optimal transformation for one field in Packed
 *   slower than the Planar version because it uses all three color channels
 */
LocalMotion calcFieldTransPacked(VSMotionDetect* md, const Field* field,
                              int fieldnum) {
  int tx = 0;
  int ty = 0;
  uint8_t *I_c = md->curr.data[0], *I_p = md->prev.data[0];
  int width1 = md->curr.linesize[0]/3; // linesize in pixels
  int width2 = md->prev.linesize[0]/3; // linesize in pixels
  int i, j;

  /* Here we improve speed by checking first the most probable position
     then the search paths are most effectively cut. (0,0) is a simple start
  */
  unsigned int minerror = compareSubImg(I_c, I_p, field, width1, width2, md->fi.height,
                                        3, 0, 0, UINT_MAX);
  // check all positions...
  for (i = -md->maxShift; i <= md->maxShift; i += md->stepSize) {
    for (j = -md->maxShift; j <= md->maxShift; j += md->stepSize) {
      if( i==0 && j==0 )
        continue; //no need to check this since already done
      unsigned int error = compareSubImg(I_c, I_p, field, width1, width2,
                                         md->fi.height, 3, i, j, minerror);
      if (error < minerror) {
        minerror = error;
        tx = i;
        ty = j;
      }
    }
  }
  if (md->stepSize > 1) { // make fine grain check around the best match
    int txc = tx; // save the shifts
    int tyc = ty;
    int r = md->stepSize - 1;
    for (i = txc - r; i <= txc + r; i += 1) {
      for (j = tyc - r; j <= tyc + r; j += 1) {
        if (i == txc && j == tyc)
          continue; //no need to check this since already done
        unsigned int error = compareSubImg(I_c, I_p, field, width1, width2,
                                           md->fi.height, 3, i, j, minerror);
        if (error < minerror) {
          minerror = error;
          tx = i;
          ty = j;
        }
      }
    }
  }

  if (!md->allowMax && fabs(tx) >= md->maxShift + md->stepSize) {
#ifdef STABVERBOSE
    vs_log_msg(md->modName, "maximal x shift ");
#endif
    tx = 0;
  }
  if (!md->allowMax && fabs(ty) == md->maxShift + md->stepSize) {
#ifdef STABVERBOSE
    vs_log_msg(md->modName, "maximal y shift ");
#endif
    ty = 0;
  }
  LocalMotion lm = null_localmotion();
  lm.f = *field;
  lm.v.x = tx;
  lm.v.y = ty;
  lm.match = minerror;
  return lm;
}

/* compares contrast_idx structures respect to the contrast
   (for sort function)
*/
int cmp_contrast_idx(const void *ci1, const void* ci2) {
  double a = ((contrast_idx*) ci1)->contrast;
  double b = ((contrast_idx*) ci2)->contrast;
  return a < b ? 1 : (a > b ? -1 : 0);
}

/* select only the best 'maxfields' fields
   first calc contrasts then select from each part of the
   frame a some fields
*/
VSVector selectfields(VSMotionDetect* md, contrastSubImgFunc contrastfunc) {
  int i, j;
  VSVector goodflds;
  contrast_idx *ci =
    (contrast_idx*) vs_malloc(sizeof(contrast_idx) * md->fieldNum);
  vs_vector_init(&goodflds, md->fieldNum);

  // we split all fields into row+1 segments and take from each segment
  // the best fields
  int numsegms = (md->fieldRows + 1);
  int segmlen = md->fieldNum / (md->fieldRows + 1) + 1;
  // split the frame list into rows+1 segments
  contrast_idx *ci_segms =
    (contrast_idx*) vs_malloc(sizeof(contrast_idx) * md->fieldNum);
  int remaining = 0;
  // calculate contrast for each field
  // #pragma omp parallel for shared(ci,md) no speedup because to short
  for (i = 0; i < md->fieldNum; i++) {
    ci[i].contrast = contrastfunc(md, &md->fields[i]);
    ci[i].index = i;
    if (ci[i].contrast < md->contrastThreshold)
      ci[i].contrast = 0;
    // else printf("%i %lf\n", ci[i].index, ci[i].contrast);
  }

  memcpy(ci_segms, ci, sizeof(contrast_idx) * md->fieldNum);
  // get best fields from each segment
  for (i = 0; i < numsegms; i++) {
    int startindex = segmlen * i;
    int endindex = segmlen * (i + 1);
    endindex = endindex > md->fieldNum ? md->fieldNum : endindex;
    //printf("Segment: %i: %i-%i\n", i, startindex, endindex);

    // sort within segment
    qsort(ci_segms + startindex, endindex - startindex,
          sizeof(contrast_idx), cmp_contrast_idx);
    // take maxfields/numsegms
    for (j = 0; j < md->maxFields / numsegms; j++) {
      if (startindex + j >= endindex)
        continue;
      // printf("%i %lf\n", ci_segms[startindex+j].index,
      //                    ci_segms[startindex+j].contrast);
      if (ci_segms[startindex + j].contrast > 0) {
        vs_vector_append_dup(&goodflds, &ci[ci_segms[startindex+j].index],
                             sizeof(contrast_idx));
        // don't consider them in the later selection process
        ci_segms[startindex + j].contrast = 0;
      }
    }
  }
  // check whether enough fields are selected
  // printf("Phase2: %i\n", vs_list_size(goodflds));
  remaining = md->maxFields - vs_vector_size(&goodflds);
  if (remaining > 0) {
    // take the remaining from the leftovers
    qsort(ci_segms, md->fieldNum, sizeof(contrast_idx), cmp_contrast_idx);
    for (j = 0; j < remaining; j++) {
      if (ci_segms[j].contrast > 0) {
        vs_vector_append_dup(&goodflds, &ci_segms[j], sizeof(contrast_idx));
      }
    }
  }
  // printf("Ende: %i\n", vs_list_size(goodflds));
  vs_free(ci);
  vs_free(ci_segms);
  return goodflds;
}

/* tries to register current frame onto previous frame.
 *   Algorithm:
 *   discards fields with low contrast
 *   select maxfields fields according to their contrast
 *   check theses fields for vertical and horizontal transformation
 *   use minimal difference of all possible positions
 *   calculate shift as cleaned mean of all remaining fields
 *   calculate rotation angle of each field in respect to center of fields
 *   after shift removal
 *   calculate rotation angle as cleaned mean of all angles
 *   compensate for possibly off-center rotation
 */
LocalMotions calcTransFields(VSMotionDetect* md,
                             calcFieldTransFunc fieldfunc,
                             contrastSubImgFunc contrastfunc) {
  LocalMotions localmotions;
  vs_vector_init(&localmotions,md->maxFields);

  int i, index = 0, num_motions;
#ifdef STABVERBOSE
  FILE *file = NULL;
  char buffer[32];
  vs_snprintf(buffer, sizeof(buffer), "k%04i.dat", md->frameNum);
  file = fopen(buffer, "w");
  fprintf(file, "# plot \"%s\" w l, \"\" every 2:1:0\n", buffer);
#endif

  VSVector goodflds = selectfields(md, contrastfunc);
  // use all "good" fields and calculate optimal match to previous frame
#ifdef USE_OMP
#pragma omp parallel for shared(goodflds, md, localmotions, fs) // does not bring speedup
#endif
  for(index=0; index < vs_vector_size(&goodflds); index++){
    int i = ((contrast_idx*)vs_vector_get(&goodflds,index))->index;
    LocalMotion m;
    m = fieldfunc(md, &md->fields[i], i); // e.g. calcFieldTransPlanar
    m.contrast = ((contrast_idx*)vs_vector_get(&goodflds,index))->contrast;
#ifdef STABVERBOSE
    fprintf(file, "%i %i\n%f %f %f %f\n \n\n", m.f.x, m.f.y,
            m.f.x + m.v.x, m.f.y + m.v.y, m.match, m.contrast);
#endif
    //if (t.match > somethreshold) { // ignore if too bad
    vs_vector_append_dup(&localmotions, &m, sizeof(LocalMotion));
      //}
  }

  num_motions = vs_vector_size(&localmotions); // amount of transforms we actually have
  vs_vector_del(&goodflds);
  if (num_motions < 1) {
    vs_log_warn(md->modName, "too low contrast! No field remains.\n \
                    (no translations are detected in frame %i)", md->frameNum);
  }

  if (md->show) { // draw fields and transforms into frame.
    // this has to be done one after another to handle possible overlap
    if (md->show > 1) {
      for (i = 0; i < num_motions; i++)
        drawFieldScanArea(md, LMGet(&localmotions,i));
    }
    for (i = 0; i < num_motions; i++)
      drawField(md, LMGet(&localmotions,i));
    for (i = 0; i < num_motions; i++)
      drawFieldTrans(md, LMGet(&localmotions,i));
  }
#ifdef STABVERBOSE
  fclose(file);
#endif
  return localmotions;
}





/** draws the field scanning area */
void drawFieldScanArea(VSMotionDetect* md, const LocalMotion* lm) {
  if (md->fi.pFormat > PF_PACKED)
    return;
  drawBox(md->currorig.data[0], md->currorig.linesize[0], md->fi.height, 1, lm->f.x, lm->f.y,
          lm->f.size + 2 * md->maxShift, lm->f.size + 2 * md->maxShift, 80);
}

/** draws the field */
void drawField(VSMotionDetect* md, const LocalMotion* lm) {
  if (md->fi.pFormat > PF_PACKED)
    return;
  drawBox(md->currorig.data[0], md->currorig.linesize[0], md->fi.height, 1, lm->f.x, lm->f.y,
          lm->f.size, lm->f.size, /*lm->match >100 ? 100 :*/ 40);
}

/** draws the transform data of this field */
void drawFieldTrans(VSMotionDetect* md, const LocalMotion* lm) {
  if (md->fi.pFormat > PF_PACKED)
    return;
  drawBox(md->currorig.data[0], md->currorig.linesize[0], md->fi.height, 1,
          lm->f.x, lm->f.y, 5, 5, 128); // draw center
  drawBox(md->currorig.data[0], md->currorig.linesize[0], md->fi.height, 1,
          lm->f.x + lm->v.x, lm->f.y + lm->v.y, 8, 8, 250); // draw translation
}

/**
 * draws a box at the given position x,y (center) in the given color
 (the same for all channels)
*/
void drawBox(unsigned char* I, int width, int height, int bytesPerPixel, int x,
       int y, int sizex, int sizey, unsigned char color) {

  unsigned char* p = NULL;
  int j, k;
  p = I + ((x - sizex / 2) + (y - sizey / 2) * width) * bytesPerPixel;
  for (j = 0; j < sizey; j++) {
    for (k = 0; k < sizex * bytesPerPixel; k++) {
      *p = color;
      p++;
    }
    p += (width - sizex) * bytesPerPixel;
  }
}

// void addTrans(VSMotionDetect* md, VSTransform sl) {
//   if (!md->transs) {
//     md->transs = vs_list_new(0);
//   }
//   vs_list_append_dup(md->transs, &sl, sizeof(sl));
// }

// VSTransform getLastVSTransform(VSMotionDetect* md){
//   if (!md->transs || !md->transs->head) {
//     return null_transform();
//   }
//   return *((VSTransform*)md->transs->tail);
// }


//#ifdef TESTING
/// plain C implementation of compareSubImg (without ORC)
unsigned int compareSubImg_thr(unsigned char* const I1, unsigned char* const I2,
                               const Field* field, int width1, int width2, int height,
           int bytesPerPixel, int d_x, int d_y,
           unsigned int threshold) {
  int k, j;
  unsigned char* p1 = NULL;
  unsigned char* p2 = NULL;
  int s2 = field->size / 2;
  unsigned int sum = 0;

  p1 = I1 + ((field->x - s2) + (field->y - s2) * width1) * bytesPerPixel;
  p2 = I2 + ((field->x - s2 + d_x) + (field->y - s2 + d_y) * width2)
    * bytesPerPixel;
  for (j = 0; j < field->size; j++) {
    for (k = 0; k < field->size * bytesPerPixel; k++) {
      sum += abs((int) *p1 - (int) *p2);
      p1++;
      p2++;
    }
    if( sum > threshold) // no need to calculate any longer: worse than the best match
      break;
    p1 += (width1 - field->size) * bytesPerPixel;
    p2 += (width2 - field->size) * bytesPerPixel;
  }
  return sum;
}


/*
 * Local variables:
 *   c-file-style: "stroustrup"
 *   c-file-offsets: ((case-label . *) (statement-case-intro . *))
 *   indent-tabs-mode: nil
 *   tab-width:  2
 *   c-basic-offset: 2 t
 * End:
 *
 */
