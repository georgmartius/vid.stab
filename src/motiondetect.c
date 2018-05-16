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
#include "transformtype_operations.h"

#define USE_SPIRAL_FIELD_CALC


/* internal data structures */

// structure that contains the contrast and the index of a field
typedef struct _contrast_idx {
  double contrast;
  int index;
} contrast_idx;


VSMotionDetectConfig vsMotionDetectGetDefaultConfig(const char* modName){
  VSMotionDetectConfig conf;
  conf.stepSize          = 6;
  conf.accuracy          = 15;
  conf.shakiness         = 5;
  conf.virtualTripod     = 0;
  conf.contrastThreshold = 0.25;
  conf.show              = 0;
  conf.modName           = modName;
  conf.numThreads        = 0;
  return conf;
}

void vsMotionDetectGetConfig(VSMotionDetectConfig* conf, const VSMotionDetect* md){
  if(md && conf)
    *conf = md->conf;
}

const VSFrameInfo* vsMotionDetectGetFrameInfo(const VSMotionDetect* md){
  return &md->fi;
}


int vsMotionDetectInit(VSMotionDetect* md, const VSMotionDetectConfig* conf, const VSFrameInfo* fi){
  assert(md && fi);
  md->conf = *conf;
  md->fi = *fi;

  if(fi->pFormat<=PF_NONE ||  fi->pFormat==PF_PACKED || fi->pFormat>=PF_NUMBER) {
    vs_log_warn(md->conf.modName, "unsupported Pixel Format (%i)\n",
                md->fi.pFormat);
    return VS_ERROR;
  }

#ifdef USE_OMP
  if(md->conf.numThreads==0)
    md->conf.numThreads=VS_MAX(omp_get_max_threads()*0.8,1);
  vs_log_info(md->conf.modName, "Multithreading: use %i threads\n",md->conf.numThreads);
#endif

  vsFrameAllocate(&md->prev, &md->fi);
  if (vsFrameIsNull(&md->prev)) {
    vs_log_error(md->conf.modName, "malloc failed");
    return VS_ERROR;
  }

  vsFrameNull(&md->curr);
  vsFrameNull(&md->currorig);
  vsFrameNull(&md->currtmp);
  md->hasSeenOneFrame = 0;
  md->frameNum = 0;

  // TODO: get rid of shakiness parameter in the long run
  md->conf.shakiness = VS_MIN(10,VS_MAX(1,md->conf.shakiness));
  md->conf.accuracy = VS_MIN(15,VS_MAX(1,md->conf.accuracy));
  if (md->conf.accuracy < md->conf.shakiness / 2) {
    vs_log_info(md->conf.modName, "Accuracy should not be lower than shakiness/2 -- fixed");
    md->conf.accuracy = md->conf.shakiness / 2;
  }
  if (md->conf.accuracy > 9 && md->conf.stepSize > 6) {
    vs_log_info(md->conf.modName, "For high accuracy use lower stepsize  -- set to 6 now");
    md->conf.stepSize = 6; // maybe 4
  }

  int minDimension = VS_MIN(md->fi.width, md->fi.height);
//  shift: shakiness 1: height/40; 10: height/4
//  md->maxShift = VS_MAX(4,(minDimension*md->conf.shakiness)/40);
//  size: shakiness 1: height/40; 10: height/6 (clipped)
//  md->fieldSize = VS_MAX(4,VS_MIN(minDimension/6, (minDimension*md->conf.shakiness)/40));

  // fixed size and shift now
  int maxShift      = VS_MAX(16, minDimension/7);
  int fieldSize     = VS_MAX(16, minDimension/10);
  int fieldSizeFine = VS_MAX(6, minDimension/60);
#if defined(USE_SSE2) || defined(USE_SSE2_ASM)
  fieldSize     = (fieldSize / 16 + 1) * 16;
  fieldSizeFine = (fieldSizeFine / 16 + 1) * 16;
#endif
  if (!initFields(md, &md->fieldscoarse, fieldSize, maxShift, md->conf.stepSize,
                  1, 0, md->conf.contrastThreshold)) {
    return VS_ERROR;
  }
  // for the fine check we use a smaller size and smaller maximal shift (=size)
  if (!initFields(md, &md->fieldsfine, fieldSizeFine, fieldSizeFine,
                  2, 1, fieldSizeFine, md->conf.contrastThreshold/2)) {
    return VS_ERROR;
  }

  vsFrameAllocate(&md->curr,&md->fi);
  vsFrameAllocate(&md->currtmp, &md->fi);

  md->initialized = 2;
  return VS_OK;
}

void vsMotionDetectionCleanup(VSMotionDetect* md) {
  if(md->fieldscoarse.fields) {
    vs_free(md->fieldscoarse.fields);
    md->fieldscoarse.fields=0;
  }
  if(md->fieldsfine.fields) {
    vs_free(md->fieldsfine.fields);
    md->fieldsfine.fields=0;
  }
  vsFrameFree(&md->prev);
  vsFrameFree(&md->curr);
  vsFrameFree(&md->currtmp);

  md->initialized = 0;
}

// returns true if match of local motion is better than threshold
short lm_match_better(void* thresh, void* lm){
  if(((LocalMotion*)lm)->match <= *((double*)thresh))
    return 1;
  else
    return 0;
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
    boxblurPlanar(&md->curr, frame, &md->currtmp, &md->fi, md->conf.stepSize*1/*1.4*/,
               BoxBlurNoColor);
    // two times yields tent-kernel smoothing, which may be better, but I don't
    //  think we need it
    //boxblurPlanar(md->curr, md->curr, md->currtmp, &md->fi, md->stepSize*1,
    // BoxBlurNoColor);
  }

  if (md->hasSeenOneFrame) {
    LocalMotions motionscoarse;
    LocalMotions motionsfine;
    vs_vector_init(&motionsfine,0);
    //    md->curr = frame;
    if (md->fi.pFormat > PF_PACKED) {
      motionscoarse = calcTransFields(md, &md->fieldscoarse,
                                      calcFieldTransPacked, contrastSubImgPacked);
    } else { // PLANAR
      motionscoarse = calcTransFields(md, &md->fieldscoarse,
                                      calcFieldTransPlanar, contrastSubImgPlanar);
    }
    int num_motions = vs_vector_size(&motionscoarse);
    if (num_motions < 1) {
      vs_log_warn(md->conf.modName, "too low contrast. \
(no translations are detected in frame %i)\n", md->frameNum);
    }else{
      // calc transformation and perform another scan with small fields
      VSTransform t = vsSimpleMotionsToTransform(md->fi, md->conf.modName, &motionscoarse);
      md->fieldsfine.offset    = t;
      md->fieldsfine.useOffset = 1;
      LocalMotions motions2;
      if (md->fi.pFormat > PF_PACKED) {
        motions2 = calcTransFields(md, &md->fieldsfine,
                                   calcFieldTransPacked, contrastSubImgPacked);
      } else { // PLANAR
        motions2 = calcTransFields(md, &md->fieldsfine,
                                   calcFieldTransPlanar, contrastSubImgPlanar);
      }
      // through out those with bad match (worse than mean of coarse scan)
      VSArray matchQualities1 = localmotionsGetMatch(&motionscoarse);
      double meanMatch = cleanmean(matchQualities1.dat, matchQualities1.len, NULL, NULL);
      motionsfine      = vs_vector_filter(&motions2, lm_match_better, &meanMatch);
      if(0){
        printf("\nMatches: mean:  %f | ", meanMatch);
        vs_array_print(matchQualities1, stdout);
        printf("\n         fine: ");
        VSArray matchQualities2 = localmotionsGetMatch(&motions2);
        vs_array_print(matchQualities2, stdout);
        printf("\n");
      }
    }
    if (md->conf.show) { // draw fields and transforms into frame.
      int num_motions_fine = vs_vector_size(&motionsfine);
      // this has to be done one after another to handle possible overlap
      if (md->conf.show > 1) {
        for (int i = 0; i < num_motions; i++)
          drawFieldScanArea(md, LMGet(&motionscoarse,i), md->fieldscoarse.maxShift);
      }
      for (int i = 0; i < num_motions; i++)
        drawField(md, LMGet(&motionscoarse,i), 1);
      for (int i = 0; i < num_motions_fine; i++)
        drawField(md, LMGet(&motionsfine,i), 0);
      for (int i = 0; i < num_motions; i++)
        drawFieldTrans(md, LMGet(&motionscoarse,i),180);
      for (int i = 0; i < num_motions_fine; i++)
        drawFieldTrans(md, LMGet(&motionsfine,i), 64);
    }
    *motions = vs_vector_concat(&motionscoarse,&motionsfine);
    //*motions = motionscoarse;
    //*motions = motionsfine;
  } else {
    vs_vector_init(motions,1); // dummy vector
    md->hasSeenOneFrame = 1;
  }

  // for tripod we keep a certain reference frame
  if(md->conf.virtualTripod < 1 || md->frameNum < md->conf.virtualTripod)
    // copy current frame (smoothed) to prev for next frame comparison
    vsFrameCopy(&md->prev, &md->curr, &md->fi);
  md->frameNum++;
  return VS_OK;
}


/** initialise measurement fields on the frame.
    The size of the fields and the maxshift is used to
    calculate an optimal distribution in the frame.
    if border is set then they are placed savely away from the border for maxShift
*/

int initFields(VSMotionDetect* md, VSMotionDetectFields* fs,
               int size, int maxShift, int stepSize,
               short keepBorder, int spacing, double contrastThreshold) {
  fs->fieldSize = size;
  fs->maxShift  = maxShift;
  fs->stepSize  = stepSize;
  fs->useOffset = 0;
  fs->contrastThreshold = contrastThreshold;

  int rows = VS_MAX(3,(md->fi.height - fs->maxShift*2)/(size+spacing)-1);
  int cols = VS_MAX(3,(md->fi.width - fs->maxShift*2)/(size+spacing)-1);
  // make sure that the remaining rows have the same length
  fs->fieldNum = rows * cols;
  fs->fieldRows = rows;

  if (!(fs->fields = (Field*) vs_malloc(sizeof(Field) * fs->fieldNum))) {
    vs_log_error(md->conf.modName, "malloc failed!\n");
    return 0;
  } else {
    int i, j;
    int border=fs->stepSize;
    // the border is the amount by which the field centers
    // have to be away from the image boundary
    // (stepsize is added in case shift is increased through stepsize)
    if(keepBorder)
      border = size / 2 + fs->maxShift + fs->stepSize;
    int step_x = (md->fi.width  - 2 * border) / VS_MAX(cols-1,1);
    int step_y = (md->fi.height - 2 * border) / VS_MAX(rows-1,1);
    for (j = 0; j < rows; j++) {
      for (i = 0; i < cols; i++) {
        int idx = j * cols + i;
        fs->fields[idx].x = border + i * step_x;
        fs->fields[idx].y = border + j * step_y;
        fs->fields[idx].size = size;
      }
    }
  }
  fs->maxFields = (md->conf.accuracy) * fs->fieldNum / 15;
  vs_log_info(md->conf.modName, "Fieldsize: %i, Maximal translation: %i pixel\n",
              fs->fieldSize, fs->maxShift);
  vs_log_info(md->conf.modName, "Number of used measurement fields: %i out of %i\n",
              fs->maxFields, fs->fieldNum);

  return 1;
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

/* calculates the optimal transformation for one field in Planar frames
 * (only luminance)
 */
LocalMotion calcFieldTransPlanar(VSMotionDetect* md, VSMotionDetectFields* fs,
                                 const Field* field, int fieldnum) {
  int tx = 0;
  int ty = 0;
  uint8_t *Y_c = md->curr.data[0], *Y_p = md->prev.data[0];
  int linesize_c = md->curr.linesize[0], linesize_p = md->prev.linesize[0];
  // we only use the luminance part of the image
  int i, j;
  int stepSize = fs->stepSize;
  int maxShift = fs->maxShift;
  Vec offset = { 0, 0};
  LocalMotion lm = null_localmotion();
  if(fs->useOffset){
    // Todo: we could put the preparedtransform into fs
    PreparedTransform pt = prepare_transform(&fs->offset, &md->fi);
    Vec fieldpos = {field->x, field->y};
    offset = sub_vec(transform_vec(&pt, &fieldpos), fieldpos);
    // is the field still in the frame
    int s2 = field->size/2;
    if(unlikely(fieldpos.x+offset.x-s2-maxShift-stepSize < 0 ||
                fieldpos.x+offset.x+s2+maxShift+stepSize >= md->fi.width ||
                fieldpos.y+offset.y-s2-maxShift-stepSize < 0 ||
                fieldpos.y+offset.y+s2+maxShift+stepSize >= md->fi.height)){
      lm.match=-1;
      return lm;
    }
  }

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
  while (j >= -maxShift && j <= maxShift && i >= -maxShift && i <= maxShift) {
    unsigned int error = compareSubImg(Y_c, Y_p, field, linesize_c, linesize_p,
                                       md->fi.height, 1, i + offset.x, j + offset.y,
                                       minerror);

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
  for (i = -maxShift; i <= maxShift; i += stepSize) {
    for (j = -maxShift; j <= maxShift; j += stepSize) {
      if( i==0 && j==0 )
        continue; //no need to check this since already done
      unsigned int error = compareSubImg(Y_c, Y_p, field, linesize_c, linesize_p,
                                         md->fi.height, 1, i+offset.x, j+offset.y, minerror);
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
                                           md->fi.height, 1, i+offset.x, j+offset.y, minerror);
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

  if (unlikely(fabs(tx) >= maxShift + stepSize - 1  ||
               fabs(ty) >= maxShift + stepSize)) {
#ifdef STABVERBOSE
    vs_log_msg(md->modName, "maximal shift ");
#endif
    lm.match =-1.0; // to be kicked out
    return lm;
  }
  lm.f = *field;
  lm.v.x = tx + offset.x;
  lm.v.y = ty + offset.y;
  lm.match = ((double) minerror)/(field->size*field->size);
  return lm;
}

/* calculates the optimal transformation for one field in Packed
 *   slower than the Planar version because it uses all three color channels
 */
LocalMotion calcFieldTransPacked(VSMotionDetect* md, VSMotionDetectFields* fs,
                                 const Field* field, int fieldnum) {
  int tx = 0;
  int ty = 0;
  uint8_t *I_c = md->curr.data[0], *I_p = md->prev.data[0];
  int width1 = md->curr.linesize[0]/3; // linesize in pixels
  int width2 = md->prev.linesize[0]/3; // linesize in pixels
  int i, j;
  int stepSize = fs->stepSize;
  int maxShift = fs->maxShift;

  Vec offset = { 0, 0};
  LocalMotion lm = null_localmotion();
  if(fs->useOffset){
    PreparedTransform pt = prepare_transform(&fs->offset, &md->fi);
    offset = transform_vec(&pt, (Vec*)field);
    // is the field still in the frame
    if(unlikely(offset.x-maxShift-stepSize < 0 || offset.x+maxShift+stepSize >= md->fi.width ||
                offset.y-maxShift-stepSize < 0 || offset.y+maxShift+stepSize >= md->fi.height)){
      lm.match=-1;
      return lm;
    }
  }

  /* Here we improve speed by checking first the most probable position
     then the search paths are most effectively cut. (0,0) is a simple start
  */
  unsigned int minerror = compareSubImg(I_c, I_p, field, width1, width2, md->fi.height,
                                        3, offset.x, offset.y, UINT_MAX);
  // check all positions...
  for (i = -maxShift; i <= maxShift; i += stepSize) {
    for (j = -maxShift; j <= maxShift; j += stepSize) {
      if( i==0 && j==0 )
        continue; //no need to check this since already done
      unsigned int error = compareSubImg(I_c, I_p, field, width1, width2,
                                         md->fi.height, 3, i + offset.x, j + offset.y, minerror);
      if (error < minerror) {
        minerror = error;
        tx = i;
        ty = j;
      }
    }
  }
  if (stepSize > 1) { // make fine grain check around the best match
    int txc = tx; // save the shifts
    int tyc = ty;
    int r = stepSize - 1;
    for (i = txc - r; i <= txc + r; i += 1) {
      for (j = tyc - r; j <= tyc + r; j += 1) {
        if (i == txc && j == tyc)
          continue; //no need to check this since already done
        unsigned int error = compareSubImg(I_c, I_p, field, width1, width2,
                                           md->fi.height, 3, i + offset.x, j + offset.y, minerror);
        if (error < minerror) {
          minerror = error;
          tx = i;
          ty = j;
        }
      }
    }
  }

  if (fabs(tx) >= maxShift + stepSize - 1 || fabs(ty) >= maxShift + stepSize - 1) {
#ifdef STABVERBOSE
    vs_log_msg(md->modName, "maximal shift ");
#endif
    lm.match = -1;
    return lm;
  }
  lm.f = *field;
  lm.v.x = tx + offset.x;
  lm.v.y = ty + offset.y;
  lm.match = ((double)minerror)/(field->size*field->size);
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
   frame some fields
   We may simplify here by using random. People want high quality, so typically we use all.
*/
VSVector selectfields(VSMotionDetect* md, VSMotionDetectFields* fs,
                      contrastSubImgFunc contrastfunc) {
  int i, j;
  VSVector goodflds;
  contrast_idx *ci =
    (contrast_idx*) vs_malloc(sizeof(contrast_idx) * fs->fieldNum);
  vs_vector_init(&goodflds, fs->fieldNum);

  // we split all fields into row+1 segments and take from each segment
  // the best fields
  int numsegms = (fs->fieldRows + 1);
  int segmlen = fs->fieldNum / (fs->fieldRows + 1) + 1;
  // split the frame list into rows+1 segments
  contrast_idx *ci_segms =
    (contrast_idx*) vs_malloc(sizeof(contrast_idx) * fs->fieldNum);
  int remaining = 0;
  // calculate contrast for each field
  // #pragma omp parallel for shared(ci,md) no speedup because to short
  for (i = 0; i < fs->fieldNum; i++) {
    ci[i].contrast = contrastfunc(md, &fs->fields[i]);
    ci[i].index = i;
    if (ci[i].contrast < fs->contrastThreshold)
      ci[i].contrast = 0;
    // else printf("%i %lf\n", ci[i].index, ci[i].contrast);
  }

  memcpy(ci_segms, ci, sizeof(contrast_idx) * fs->fieldNum);
  // get best fields from each segment
  for (i = 0; i < numsegms; i++) {
    int startindex = segmlen * i;
    int endindex = segmlen * (i + 1);
    endindex = endindex > fs->fieldNum ? fs->fieldNum : endindex;
    //printf("Segment: %i: %i-%i\n", i, startindex, endindex);

    // sort within segment
    qsort(ci_segms + startindex, endindex - startindex,
          sizeof(contrast_idx), cmp_contrast_idx);
    // take maxfields/numsegms
    for (j = 0; j < fs->maxFields / numsegms; j++) {
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
  remaining = fs->maxFields - vs_vector_size(&goodflds);
  if (remaining > 0) {
    // take the remaining from the leftovers
    qsort(ci_segms, fs->fieldNum, sizeof(contrast_idx), cmp_contrast_idx);
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
 */
LocalMotions calcTransFields(VSMotionDetect* md,
                             VSMotionDetectFields* fields,
                             calcFieldTransFunc fieldfunc,
                             contrastSubImgFunc contrastfunc) {
  LocalMotions localmotions;
  vs_vector_init(&localmotions,fields->maxFields);

#ifdef STABVERBOSE
  FILE *file = NULL;
  char buffer[32];
  vs_snprintf(buffer, sizeof(buffer), "k%04i.dat", md->frameNum);
  file = fopen(buffer, "w");
  fprintf(file, "# plot \"%s\" w l, \"\" every 2:1:0\n", buffer);
#endif

  VSVector goodflds = selectfields(md, fields, contrastfunc);
  // use all "good" fields and calculate optimal match to previous frame
#ifdef USE_OMP
  omp_set_num_threads(md->conf.numThreads);
#pragma omp parallel for shared(goodflds, md, localmotions)
#endif
  for(int index=0; index < vs_vector_size(&goodflds); index++){
    int i = ((contrast_idx*)vs_vector_get(&goodflds,index))->index;
    LocalMotion m;
    m = fieldfunc(md, fields, &fields->fields[i], i); // e.g. calcFieldTransPlanar
    if(m.match >= 0){
      m.contrast = ((contrast_idx*)vs_vector_get(&goodflds,index))->contrast;
#ifdef STABVERBOSE
      fprintf(file, "%i %i\n%f %f %f %f\n \n\n", m.f.x, m.f.y,
              m.f.x + m.v.x, m.f.y + m.v.y, m.match, m.contrast);
#endif
#pragma omp critical(localmotions_append)
      vs_vector_append_dup(&localmotions, &m, sizeof(LocalMotion));
    }
  }
  vs_vector_del(&goodflds);

#ifdef STABVERBOSE
  fclose(file);
#endif
  return localmotions;
}





/** draws the field scanning area */
void drawFieldScanArea(VSMotionDetect* md, const LocalMotion* lm, int maxShift) {
  if (md->fi.pFormat > PF_PACKED)
    return;
  drawRectangle(md->currorig.data[0], md->currorig.linesize[0], md->fi.height, 1, lm->f.x, lm->f.y,
                lm->f.size + 2 * maxShift, lm->f.size + 2 * maxShift, 80);
}

/** draws the field */
void drawField(VSMotionDetect* md, const LocalMotion* lm, short box) {
  if (md->fi.pFormat > PF_PACKED)
    return;
  if(box)
    drawBox(md->currorig.data[0], md->currorig.linesize[0], md->fi.height, 1,
            lm->f.x, lm->f.y, lm->f.size, lm->f.size, /*lm->match >100 ? 100 :*/ 40);
  else
    drawRectangle(md->currorig.data[0], md->currorig.linesize[0], md->fi.height, 1,
                  lm->f.x, lm->f.y, lm->f.size, lm->f.size, /*lm->match >100 ? 100 :*/ 40);
}

/** draws the transform data of this field */
void drawFieldTrans(VSMotionDetect* md, const LocalMotion* lm, int color) {
  if (md->fi.pFormat > PF_PACKED)
    return;
  Vec end = add_vec(field_to_vec(lm->f),lm->v);
  drawBox(md->currorig.data[0], md->currorig.linesize[0], md->fi.height, 1,
          lm->f.x, lm->f.y, 5, 5, 0); // draw center
  drawBox(md->currorig.data[0], md->currorig.linesize[0], md->fi.height, 1,
          lm->f.x + lm->v.x, lm->f.y + lm->v.y, 5, 5, 250); // draw translation
  drawLine(md->currorig.data[0], md->currorig.linesize[0],  md->fi.height, 1,
           (Vec*)&lm->f, &end, 3, color);

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

/**
 * draws a rectangle (not filled) at the given position x,y (center) in the given color
 at the first channel
*/
void drawRectangle(unsigned char* I, int width, int height, int bytesPerPixel, int x,
                   int y, int sizex, int sizey, unsigned char color) {

  unsigned char* p;
  int k;
  p = I + ((x - sizex / 2) + (y - sizey / 2) * width) * bytesPerPixel;
  for (k = 0; k < sizex; k++) { *p = color; p+= bytesPerPixel; } // upper line
  p = I + ((x - sizex / 2) + (y + sizey / 2) * width) * bytesPerPixel;
  for (k = 0; k < sizex; k++) { *p = color; p+= bytesPerPixel; } // lower line
  p = I + ((x - sizex / 2) + (y - sizey / 2) * width) * bytesPerPixel;
  for (k = 0; k < sizey; k++) { *p = color; p+= width * bytesPerPixel; } // left line
  p = I + ((x + sizex / 2) + (y - sizey / 2) * width) * bytesPerPixel;
  for (k = 0; k < sizey; k++) { *p = color; p+= width * bytesPerPixel; } // right line
}

/**
 * draws a line from a to b with given thickness(not filled) at the given position x,y (center) in the given color
 at the first channel
*/
void drawLine(unsigned char* I, int width, int height, int bytesPerPixel,
              Vec* a, Vec* b, int thickness, unsigned char color) {
  unsigned char* p;
  Vec div = sub_vec(*b,*a);
  if(div.y==0){ // horizontal line
    if(div.x<0) {*a=*b; div.x*=-1;}
    for(int r=-thickness/2; r<=thickness/2; r++){
      p = I + ((a->x) + (a->y+r) * width) * bytesPerPixel;
      for (int k = 0; k <= div.x; k++) { *p = color; p+= bytesPerPixel; }
    }
  }else{
    if(div.x==0){ // vertical line
      if(div.y<0) {*a=*b; div.y*=-1;}
      for(int r=-thickness/2; r<=thickness/2; r++){
        p = I + ((a->x+r) + (a->y) * width) * bytesPerPixel;
        for (int k = 0; k <= div.y; k++) { *p = color; p+= width * bytesPerPixel; }
      }
    }else{
      double m = (double)div.x/(double)div.y;
      int horlen = thickness + fabs(m);
      for( int c=0; c<= abs(div.y); c++){
        int dy = div.y<0 ? -c : c;
        int x = a->x + m*dy - horlen/2;
        p = I + (x + (a->y+dy) * width) * bytesPerPixel;
        for( int k=0; k<= horlen; k++){ *p = color; p+= bytesPerPixel; }
      }
    }
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
