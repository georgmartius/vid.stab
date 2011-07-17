/*
 * motiondetect.c
 *
 *  Created on: Feb 21, 2011
 *  Copyright (C) Georg Martius - February 2011
 *
 *  This file is part of transcode, a video stream processing tool
 *
 *  transcode is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2, or (at your option)
 *  any later version.
 *
 *  transcode is distributed in the hope that it will be useful,
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
#include <math.h>
#include <stdint.h>
#include <string.h>
#include <assert.h>

#include "orc/motiondetectorc.h"

#include "deshakedefines.h"

/* internal data structures */

// structure that contains the contrast and the index of a field
typedef struct _contrast_idx {
  double contrast;
  int index;
} contrast_idx;

int initMotionDetect(MotionDetect* md, const DSFrameInfo* fi,
		     const char* modName) {
  assert(md && fi);
  md->fi = *fi;
  md->modName = modName;

  md->prev = ds_zalloc(md->fi.framesize);
  if (!md->prev) {
    ds_log_error(md->modName, "malloc failed");
    return DS_ERROR;
  }
  md->currcopy = 0;
  md->hasSeenOneFrame = 0;
  md->transs = 0;

  // Options
  md->stepSize = 6;
  md->allowMax = 0;
  md->algo = 1;
  //    md->field_num   = 64;
  md->accuracy = 4;
  md->shakiness = 4;
  md->fieldSize = DS_MIN(md->fi.width, md->fi.height) / 12;
  md->show = 0;
  md->contrastThreshold = 0.03;
  md->maxAngleVariation = 1;
  md->initialized = 1;
  return DS_OK;
}

int configureMotionDetect(MotionDetect* md) {
  if (md->initialized != 1)
    return DS_ERROR;

  md->shakiness = DS_MIN(10,DS_MAX(1,md->shakiness));
  md->accuracy = DS_MIN(15,DS_MAX(1,md->accuracy));
  if (md->accuracy < md->shakiness / 2) {
    ds_log_info(md->modName, "accuracy should not be lower than shakiness/2");
    md->accuracy = md->shakiness / 2;
  }

  // shift and size: shakiness 1: height/40; 10: height/4
  md->maxShift
    = DS_MAX(4,(DS_MIN(md->fi.width, md->fi.height)*md->shakiness)/40);
  md->fieldSize
    = DS_MAX(4,(DS_MIN(md->fi.width, md->fi.height)*md->shakiness)/40);

  ds_log_info(md->modName, "Fieldsize: %i, Maximal translation: %i pixel",
	      md->fieldSize, md->maxShift);
  if (md->algo == 1) {
    // initialize measurement fields. field_num is set here.
    if (!initFields(md)) {
      return DS_ERROR;
    }
    md->maxFields = (md->accuracy) * md->fieldNum / 15;
    ds_log_info(md->modName, "Number of used measurement fields: %i out of %i",
		md->maxFields, md->fieldNum);
  }
  if (md->show)
    md->currcopy = ds_zalloc(md->fi.framesize);
  else
    md->currcopy=NULL;

  md->initialized = 2;
  return DS_OK;
}

void cleanupMotionDetection(MotionDetect* md) {
  ds_list_del(md->transs, 1);
  if (md->prev) {
    ds_free(md->prev);
    md->prev = NULL;
  }
  if (md->currcopy) {
    ds_free(md->currcopy);
    md->currcopy = NULL;
  }
  md->initialized = 0;
}

int motionDetection(MotionDetect* md, unsigned char *frame) {
  assert(md->initialized==2);
  if (md->show) // save the buffer to restore at the end for prev
    memcpy(md->currcopy, frame, md->fi.framesize);

  if (md->hasSeenOneFrame) {
    md->curr = frame;
    if (md->fi.pFormat == PF_RGB) {
      if (md->algo == 0)
	addTrans(md, calcShiftRGBSimple(md));
      else if (md->algo == 1)
	addTrans(md, calcTransFields(md, calcFieldTransRGB,
				     contrastSubImgRGB));
    } else if (md->fi.pFormat == PF_YUV) {
      if (md->algo == 0)
	addTrans(md, calcShiftYUVSimple(md));
      else if (md->algo == 1)
	addTrans(md, calcTransFields(md, calcFieldTransYUV,
				     contrastSubImgYUV));
    } else {
      ds_log_warn(md->modName, "unsupported Pixel Format (Codec: %i)\n",
		  md->fi.pFormat);
      return DS_ERROR;
    }
  } else {
    md->hasSeenOneFrame = 1;
    addTrans(md, null_transform());
  }

  if (!md->show) { // copy current frame to prev for next frame comparison
    memcpy(md->prev, frame, md->fi.framesize);
  } else { // use the copy because we changed the original frame
    memcpy(md->prev, md->currcopy, md->fi.framesize);
  }
  md->t++;
  return DS_OK;
}

/** initialise measurement fields on the frame.
    The size of the fields and the maxshift is used to
    calculate an optimal distribution in the frame.
*/
int initFields(MotionDetect* md) {
  int size = md->fieldSize;
  int rows = DS_MAX(3,(md->fi.height - md->maxShift*2)/size-1);
  int cols = DS_MAX(3,(md->fi.width - md->maxShift*2)/size-1);
  // make sure that the remaining rows have the same length
  md->fieldNum = rows * cols;
  md->fieldRows = rows;
  // ds_log_msg(md->modName, "field setup: rows: %i cols: %i Total: %i fields",
  //            rows, cols, md->field_num);

  if (!(md->fields = (Field*) malloc(sizeof(Field) * md->fieldNum))) {
    ds_log_error(md->modName, "malloc failed!\n");
    return 0;
  } else {
    int i, j;
    // the border is the amount by which the field centers
    // have to be away from the image boundary
    // (stepsize is added in case shift is increased through stepsize)
    int border = size / 2 + md->maxShift + md->stepSize;
    int step_x = (md->fi.width - 2 * border) / DS_MAX(cols-1,1);
    int step_y = (md->fi.height - 2 * border) / DS_MAX(rows-1,1);
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
   compares the two given images and returns the average absolute difference
   \param d_x shift in x direction
   \param d_y shift in y direction
*/
int compareImg(unsigned char* I1, unsigned char* I2, int width, int height,
		  int bytesPerPixel, int d_x, int d_y) {
  int i, j;
  unsigned char* p1 = NULL;
  unsigned char* p2 = NULL;
  int sum = 0;
  int effectWidth = width - abs(d_x);
  int effectHeight = height - abs(d_y);

  /*   DEBUGGING code to export single frames */
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
      p1 += (i + d_y) * width * bytesPerPixel;
      p2 += i * width * bytesPerPixel;
    } else {
      p1 += i * width * bytesPerPixel;
      p2 += (i - d_y) * width * bytesPerPixel;
    }
    if (d_x > 0) {
      p1 += d_x * bytesPerPixel;
    } else {
      p2 -= d_x * bytesPerPixel;
    }
    for (j = 0; j < effectWidth * bytesPerPixel; j++) {
      /* debugging code continued */
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
  return sum; // / ((double) effectWidth * effectHeight * bytesPerPixel);
}

/**
   compares a small part of two given images
   and returns the average absolute difference.
   Field center, size and shift have to be choosen,
   so that no clipping is required.
   Uses optimized inner loops by ORC.

   \param field Field specifies position(center) and size of subimage
   \param d_x shift in x direction
   \param d_y shift in y direction
*/
int compareSubImg(unsigned char* const I1, unsigned char* const I2,
                     const Field* field, int width, int height, int bytesPerPixel, 
                     int d_x, int d_y) {
  unsigned char* p1 = NULL;
  unsigned char* p2 = NULL;
  int s2 = field->size / 2;
  int sum=0;
  p1 = I1 + ((field->x - s2) + (field->y - s2) * width) * bytesPerPixel;
  p2 = I2 + ((field->x - s2 + d_x) + (field->y - s2 + d_y) * width)
    * bytesPerPixel;
  
  image_difference_optimized(&sum, p1, width * bytesPerPixel, 
                             p2, width*bytesPerPixel, 
                             field->size* bytesPerPixel , field->size);

  return sum; //  / ((double) field->size * field->size * bytesPerPixel);
}



/** \see contrastSubImg called with bytesPerPixel=1*/
double contrastSubImgYUV(MotionDetect* md, const Field* field) {
  return contrastSubImg(md->curr, field, md->fi.width, md->fi.height);
}

/**
   \see contrastSubImg three times called with bytesPerPixel=3
   for all channels
*/
double contrastSubImgRGB(MotionDetect* md, const Field* field) {
  unsigned char* const I = md->curr;
  return (contrastSubImg_Michelson(I, field, md->fi.width, md->fi.height, 3)
	  + contrastSubImg_Michelson(I + 1, field, md->fi.width, md->fi.height, 3)
	  + contrastSubImg_Michelson(I + 2, field, md->fi.width, md->fi.height, 3)) / 3;
}


/**
   calculates the contrast in the given small part of the given image
   using the absolute difference from mean luminance (like Root-Mean-Square,
   but with abs() (Manhattan-Norm))
   For multichannel images use contrastSubImg_Michelson()

   \param I pointer to framebuffer
   \param field Field specifies position(center) and size of subimage
   \param width width of frame
   \param height height of frame
*/
double contrastSubImg(unsigned char* const I, const Field* field, 
		      int width, int height) {
  unsigned char* p = NULL;
  int s2 = field->size / 2;
  int numpixel = field->size*field->size;

  p = I + ((field->x - s2) + (field->y - s2) * width);
  
  int sum=0;
  image_sum_optimized(&sum, p, width, field->size, field->size);
  unsigned char mean = sum / numpixel;
  int var=0;
  image_variance_optimized(&var, p, width, mean, field->size, field->size);
  return (double)var/numpixel/255.0;
}

/**
   calculates Michelson-contrast in the given small part of the given image
   to be more compatible with the absolute difference formula this is scaled by 0.1

   \param I pointer to framebuffer
   \param field Field specifies position(center) and size of subimage
   \param width width of frame
   \param height height of frame
   \param bytesPerPixel calc contrast for only for first channel
*/
double contrastSubImg_Michelson(unsigned char* const I, const Field* field, int width,
				int height, int bytesPerPixel) {
  int k, j;
  unsigned char* p = NULL;
  int s2 = field->size / 2;
  unsigned char mini = 255;
  unsigned char maxi = 0;

  p = I + ((field->x - s2) + (field->y - s2) * width) * bytesPerPixel;
  // TODO: use some mmx or sse stuff here
  for (j = 0; j < field->size; j++) {
    for (k = 0; k < field->size; k++) {
      mini = (mini < *p) ? mini : *p;
      maxi = (maxi > *p) ? maxi : *p;
      p += bytesPerPixel;
    }
    p += (width - field->size) * bytesPerPixel;
  }
  return 0.1*(maxi - mini) / (maxi + mini + 0.1); // +0.1 to avoid division by 0
}


/** tries to register current frame onto previous frame.
    This is the most simple algorithm:
    shift images to all possible positions and calc summed error
    Shift with minimal error is selected.
*/
Transform calcShiftRGBSimple(MotionDetect* md) {
  int x = 0, y = 0;
  int i, j;
  int minerror =  INT_MAX;
  for (i = -md->maxShift; i <= md->maxShift; i++) {
    for (j = -md->maxShift; j <= md->maxShift; j++) {
      int error = compareImg(md->curr, md->prev, md->fi.width,
				md->fi.height, 3, i, j);
      if (error < minerror) {
	minerror = error;
	x = i;
	y = j;
      }
    }
  }
  return new_transform(x, y, 0, 0, 0);
}

/** tries to register current frame onto previous frame.
    (only the luminance is used)
    This is the most simple algorithm:
    shift images to all possible positions and calc summed error
    Shift with minimal error is selected.
*/
Transform calcShiftYUVSimple(MotionDetect* md) {
  int x = 0, y = 0;
  int i, j;
  unsigned char *Y_c, *Y_p;// , *Cb, *Cr;
#ifdef STABVERBOSE
  FILE *f = NULL;
  char buffer[32];
  ds_snprintf(buffer, sizeof(buffer), "f%04i.dat", md->t);
  f = fopen(buffer, "w");
  fprintf(f, "# splot \"%s\"\n", buffer);
#endif

  // we only use the luminance part of the image
  Y_c = md->curr;
  //  Cb_c = md->curr + md->fi.width*md->fi.height;
  //Cr_c = md->curr + 5*md->fi.width*md->fi.height/4;
  Y_p = md->prev;
  //Cb_p = md->prev + md->fi.width*md->fi.height;
  //Cr_p = md->prev + 5*md->fi.width*md->fi.height/4;

  int minerror = INT_MAX;
  for (i = -md->maxShift; i <= md->maxShift; i++) {
    for (j = -md->maxShift; j <= md->maxShift; j++) {
      int error = compareImg(Y_c, Y_p, md->fi.width, md->fi.height, 1,
				i, j);
#ifdef STABVERBOSE
      fprintf(f, "%i %i %f\n", i, j, error);
#endif
      if (error < minerror) {
	minerror = error;
	x = i;
	y = j;
      }
    }
  }
#ifdef STABVERBOSE
  fclose(f);
  ds_log_msg(md->modName, "Minerror: %f\n", minerror);
#endif
  return new_transform(x, y, 0, 0, 0);
}

/* calculates rotation angle for the given transform and
 * field with respect to the given center-point
 */
double calcAngle(MotionDetect* md, Field* field, Transform* t, int center_x,
		 int center_y) {
  // we better ignore fields that are to close to the rotation center
  if (abs(field->x - center_x) + abs(field->y - center_y) < md->maxShift) {
    return 0;
  } else {
    // double r = sqrt(field->x*field->x + field->y*field->y);
    double a1 = atan2(field->y - center_y, field->x - center_x);
    double a2 = atan2(field->y - center_y + t->y, field->x - center_x
		      + t->x);
    double diff = a2 - a1;
    return (diff > M_PI) ? diff - 2 * M_PI : ((diff < -M_PI) ? diff + 2
					      * M_PI : diff);
  }
}

/* calculates the optimal transformation for one field in YUV frames
 * (only luminance)
 */
Transform calcFieldTransYUV(MotionDetect* md, const Field* field, int fieldnum) {
  int tx = 0;
  int ty = 0;
  uint8_t *Y_c = md->curr, *Y_p = md->prev;
  // we only use the luminance part of the image
  int i, j;

  /*     // check contrast in sub image */
  /*     double contr = contrastSubImg(Y_c, field, md->fi.width, md->fi.height, 1); */
  /*     if(contr < md->contrast_threshold) { */
  /*         t.extra=-1; */
  /*         return t; */
  /*     } */
#ifdef STABVERBOSE
  // printf("%i %i %f\n", md->t, fieldnum, contr);
  FILE *f = NULL;
  char buffer[32];
  ds_snprintf(buffer, sizeof(buffer), "f%04i_%02i.dat", md->t, fieldnum);
  f = fopen(buffer, "w");
  fprintf(f, "# splot \"%s\"\n", buffer);
#endif

  int minerror = INT_MAX;
  for (i = -md->maxShift; i <= md->maxShift; i += md->stepSize) {
    for (j = -md->maxShift; j <= md->maxShift; j += md->stepSize) {
      int error = compareSubImg(Y_c, Y_p, field, md->fi.width, md->fi.height,
				1, i, j);
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

  if (md->stepSize > 1) { // make fine grain check around the best match
    int txc = tx; // save the shifts
    int tyc = ty;
    int r = md->stepSize - 1;
    for (i = txc - r; i <= txc + r; i += 1) {
      for (j = tyc - r; j <= tyc + r; j += 1) {
	if (i == txc && j == tyc)
	  continue; //no need to check this since already done
	int error = compareSubImg(Y_c, Y_p, field, md->fi.width,
				  md->fi.height, 1, i, j);
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
  }
#ifdef STABVERBOSE
  fclose(f);
  ds_log_msg(md->modName, "Minerror: %f\n", minerror);
#endif

  if (!md->allowMax && fabs(tx) >= md->maxShift + md->stepSize) {
#ifdef STABVERBOSE
    ds_log_msg(md->modName, "maximal x shift ");
#endif
    tx = 0;
  }
  if (!md->allowMax && fabs(ty) == md->maxShift + md->stepSize) {
#ifdef STABVERBOSE
    ds_log_msg(md->modName, "maximal y shift ");
#endif
    ty = 0;
  }
  Transform t = null_transform();
  t.x = tx;
  t.y = ty;
  return t;
}

/* calculates the optimal transformation for one field in RGB
 *   slower than the YUV version because it uses all three color channels
 */
Transform calcFieldTransRGB(MotionDetect* md, const Field* field, int fieldnum) {
  Transform t = null_transform();
  uint8_t *I_c = md->curr, *I_p = md->prev;
  int i, j;

  int minerror = INT_MAX;
  for (i = -md->maxShift; i <= md->maxShift; i += 2) {
    for (j = -md->maxShift; j <= md->maxShift; j += 2) {
      int error = compareSubImg(I_c, I_p, field, md->fi.width,
                                md->fi.height, 3, i, j);
      if (error < minerror) {
	minerror = error;
	t.x = i;
	t.y = j;
      }
    }
  }
  for (i = t.x - 1; i <= t.x + 1; i += 2) {
    for (j = -t.y - 1; j <= t.y + 1; j += 2) {
      int error = compareSubImg(I_c, I_p, field, md->fi.width,
                                md->fi.height, 3, i, j);
      if (error < minerror) {
	minerror = error;
	t.x = i;
	t.y = j;
      }
    }
  }
  if (!md->allowMax && fabs(t.x) == md->maxShift) {
    t.x = 0;
  }
  if (!md->allowMax && fabs(t.y) == md->maxShift) {
    t.y = 0;
  }
  return t;
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
DSList* selectfields(MotionDetect* md, contrastSubImgFunc contrastfunc) {
  int i, j;
  DSList* goodflds = ds_list_new(0);
  contrast_idx *ci =
    (contrast_idx*) ds_malloc(sizeof(contrast_idx) * md->fieldNum);

  // we split all fields into row+1 segments and take from each segment
  // the best fields
  int numsegms = (md->fieldRows + 1);
  int segmlen = md->fieldNum / (md->fieldRows + 1) + 1;
  // split the frame list into rows+1 segments
  contrast_idx *ci_segms =
    (contrast_idx*) ds_malloc(sizeof(contrast_idx) * md->fieldNum);
  int remaining = 0;
  // calculate contrast for each field
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
	ds_list_append_dup(goodflds, &ci[ci_segms[startindex+j].index],
			   sizeof(contrast_idx));
	// don't consider them in the later selection process
	ci_segms[startindex + j].contrast = 0;
      }
    }
  }
  // check whether enough fields are selected
  // printf("Phase2: %i\n", ds_list_size(goodflds));
  remaining = md->maxFields - ds_list_size(goodflds);
  if (remaining > 0) {
    // take the remaining from the leftovers
    qsort(ci_segms, md->fieldNum, sizeof(contrast_idx), cmp_contrast_idx);
    for (j = 0; j < remaining; j++) {
      if (ci_segms[j].contrast > 0) {
	ds_list_append_dup(goodflds, &ci_segms[j], sizeof(contrast_idx));
      }
    }
  }
  // printf("Ende: %i\n", ds_list_size(goodflds));
  ds_free(ci);
  ds_free(ci_segms);
  return goodflds;
}

/* tries to register current frame onto previous frame.
 *   Algorithm:
 *   check all fields for vertical and horizontal transformation
 *   use minimal difference of all possible positions
 *   discards fields with low contrast
 *   select maxfields field according to their contrast
 *   calculate shift as cleaned mean of all remaining fields
 *   calculate rotation angle of each field in respect to center of fields
 *   after shift removal
 *   calculate rotation angle as cleaned mean of all angles
 *   compensate for possibly off-center rotation
 */
Transform calcTransFields(MotionDetect* md, calcFieldTransFunc fieldfunc,
			  contrastSubImgFunc contrastfunc) {
  Transform* ts = (Transform*) ds_malloc(sizeof(Transform) * md->fieldNum);
  Field** fs = (Field**) ds_malloc(sizeof(Field*) * md->fieldNum);
  double *angles = (double*) ds_malloc(sizeof(double) * md->fieldNum);
  int i, index = 0, num_trans;
  Transform t;
#ifdef STABVERBOSE
  FILE *file = NULL;
  char buffer[32];
  ds_snprintf(buffer, sizeof(buffer), "k%04i.dat", md->t);
  file = fopen(buffer, "w");
  fprintf(file, "# plot \"%s\" w l, \"\" every 2:1:0\n", buffer);
#endif

  DSList* goodflds = selectfields(md, contrastfunc);

  // use all "good" fields and calculate optimal match to previous frame
  contrast_idx* f;
  while ((f = (contrast_idx*) ds_list_pop(goodflds, 0)) != 0) {
    int i = f->index;
    t = fieldfunc(md, &md->fields[i], i); // e.g. calcFieldTransYUV
#ifdef STABVERBOSE
    fprintf(file, "%i %i\n%f %f %i\n \n\n", md->fields[i].x, md->fields[i].y,
	    md->fields[i].x + t.x, md->fields[i].y + t.y, t.extra);
#endif
    if (t.extra != -1) { // ignore if extra == -1 (unused at the moment)
      ts[index] = t;
      fs[index] = md->fields + i;
      index++;
    }
  }
  ds_list_fini(goodflds);

  t = null_transform();
  num_trans = index; // amount of transforms we actually have
  if (num_trans < 1) {
    ds_log_warn(md->modName, "too low contrast! No field remains.\n \
                    (no translations are detected in frame %i)", md->t);
    return t;
  }

  int center_x = 0;
  int center_y = 0;
  // calc center point of all remaining fields
  for (i = 0; i < num_trans; i++) {
    center_x += fs[i]->x;
    center_y += fs[i]->y;
  }
  center_x /= num_trans;
  center_y /= num_trans;

  if (md->show) { // draw fields and transforms into frame.
    // this has to be done one after another to handle possible overlap
    if (md->show > 1) {
      for (i = 0; i < num_trans; i++)
	drawFieldScanArea(md, fs[i], &ts[i]);
    }
    for (i = 0; i < num_trans; i++)
      drawField(md, fs[i], &ts[i]);
    for (i = 0; i < num_trans; i++)
      drawFieldTrans(md, fs[i], &ts[i]);
  }
  /* median over all transforms
     t= median_xy_transform(ts, md->field_num);*/
  // cleaned mean
  t = cleanmean_xy_transform(ts, num_trans);

  // substract avg
  for (i = 0; i < num_trans; i++) {
    ts[i] = sub_transforms(&ts[i], &t);
  }
  // figure out angle
  if (md->fieldNum < 6) {
    // the angle calculation is inaccurate for 5 and less fields
    t.alpha = 0;
  } else {
    for (i = 0; i < num_trans; i++) {
      angles[i] = calcAngle(md, fs[i], &ts[i], center_x, center_y);
    }
    double min, max;
    t.alpha = -cleanmean(angles, num_trans, &min, &max);
    if (max - min > md->maxAngleVariation) {
      t.alpha = 0;
      ds_log_info(md->modName, "too large variation in angle(%f)\n",
		  max-min);
    }
  }
  // compensate for off-center rotation
  double p_x = (center_x - md->fi.width / 2);
  double p_y = (center_y - md->fi.height / 2);
  t.x += (cos(t.alpha) - 1) * p_x - sin(t.alpha) * p_y;
  t.y += sin(t.alpha) * p_x + (cos(t.alpha) - 1) * p_y;

#ifdef STABVERBOSE
  fclose(file);
#endif
  return t;
}

/** draws the field scanning area */
void drawFieldScanArea(MotionDetect* md, const Field* field, const Transform* t) {
  if (!md->fi.pFormat == PF_YUV)
    return;
  drawBox(md->curr, md->fi.width, md->fi.height, 1, field->x, field->y,
	  field->size + 2 * md->maxShift, field->size + 2 * md->maxShift, 80);
}

/** draws the field */
void drawField(MotionDetect* md, const Field* field, const Transform* t) {
  if (!md->fi.pFormat == PF_YUV)
    return;
  drawBox(md->curr, md->fi.width, md->fi.height, 1, field->x, field->y,
	  field->size, field->size, t->extra == -1 ? 100 : 40);
}

/** draws the transform data of this field */
void drawFieldTrans(MotionDetect* md, const Field* field, const Transform* t) {
  if (!md->fi.pFormat == PF_YUV)
    return;
  drawBox(md->curr, md->fi.width, md->fi.height, 1, field->x, field->y, 5, 5,
	  128); // draw center
  drawBox(md->curr, md->fi.width, md->fi.height, 1, field->x + t->x, field->y
	  + t->y, 8, 8, 250); // draw translation
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

void addTrans(MotionDetect* md, Transform sl) {
  if (!md->transs) {
    md->transs = ds_list_new(0);
  }
  ds_list_append_dup(md->transs, &sl, sizeof(sl));
}


#ifdef TESTING
/// plain C implementation of compareSubImg (without ORC)
int compareSubImg_C(unsigned char* const I1, unsigned char* const I2,
                       const Field* field, int width, int height, int bytesPerPixel, int d_x,
		     int d_y) {
  int k, j;
  unsigned char* p1 = NULL;
  unsigned char* p2 = NULL;
  int s2 = field->size / 2;
  int sum = 0;

  p1 = I1 + ((field->x - s2) + (field->y - s2) * width) * bytesPerPixel;
  p2 = I2 + ((field->x - s2 + d_x) + (field->y - s2 + d_y) * width)
    * bytesPerPixel;
  for (j = 0; j < field->size; j++) {
    for (k = 0; k < field->size * bytesPerPixel; k++) {
      sum += abs((int) *p1 - (int) *p2);
      p1++;
      p2++;
    }
    p1 += (width - field->size) * bytesPerPixel;
    p2 += (width - field->size) * bytesPerPixel;
  }
  return sum;
}

/// plain C implementation of contrastSubImg (without ORC)
double contrastSubImg_C(unsigned char* const I, const Field* field, int width, int height) {
  int k, j;
  unsigned char* p = NULL;
  unsigned char* pstart = NULL;
  int s2 = field->size / 2;
  int sum=0;				       
  int mean;
  int var=0;
  int numpixel = field->size*field->size;  
  
  pstart = I + ((field->x - s2) + (field->y - s2) * width); 
  p = pstart;
  for (j = 0; j < field->size; j++) {      
    for (k = 0; k < field->size; k++, p++) {
      sum+=*p;
    }
    p += (width - field->size);
  }
  mean=sum/numpixel;
  p = pstart;  
  for (j = 0; j < field->size; j++) {      
    for (k = 0; k < field->size; k++, p++) {
      var+=abs(*p-mean);
    }
    p += (width - field->size);
  }
  return (double)var/numpixel/255.0;
}
#endif


/*
 * Local variables:
 *   c-file-style: "stroustrup"
 *   c-file-offsets: ((case-label . *) (statement-case-intro . *))
 *   indent-tabs-mode: nil
 * End:
 *
 */
