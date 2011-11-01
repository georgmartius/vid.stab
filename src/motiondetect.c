/*
 * motiondetect.c
 *
 *  Copyright (C) Georg Martius - February 1007-2011
 *   georg dot martius at web dot de  
 *  Copyright (C) Alexey Osipov - Jule 2011
 *   simba at lerlan dot ru
 *   speed optimizations (threshold, spiral, SSE, asm)

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

#include "boxblur.h"
#include "deshakedefines.h"

#ifdef USE_SSE2
#include <emmintrin.h>

#define USE_SSE2_CMP_HOR
#define SSE2_CMP_SUM_ROWS 8
#endif

//#include <omp.h>

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
  md->curr     = 0;
  md->currorig = 0;
  md->currtmp  = 0;
  md->hasSeenOneFrame = 0;
  md->transs = 0;

  // Options
  md->stepSize  = 6;
  md->allowMax  = 0;
  md->algo      = 1;
  md->accuracy  = 5;
  md->shakiness = 5;
  md->fieldSize = DS_MIN(md->fi.width, md->fi.height) / 12;
  md->show = 0;
  md->contrastThreshold = 0.025;
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
    ds_log_info(md->modName, "Accuracy should not be lower than shakiness/2 -- fixed");
    md->accuracy = md->shakiness / 2;
  }
  if (md->accuracy > 9 && md->stepSize > 6) {
    ds_log_info(md->modName, "For high accuracy use lower stepsize  -- set to 6 now");
    md->stepSize = 6; // maybe 4
  }

  // shift: shakiness 1: height/40; 10: height/4 
  int minDimension = DS_MIN(md->fi.width, md->fi.height);
  md->maxShift
    = DS_MAX(4,(minDimension*md->shakiness)/40);
  // size: shakiness 1: height/40; 10: height/6 (clipped) 
  md->fieldSize
    = DS_MAX(4,DS_MIN(minDimension/6, (minDimension*md->shakiness)/40));

#if defined(USE_SSE2) || defined(USE_SSE2_ASM)
  md->fieldSize = (md->fieldSize / 16 + 1) * 16;
#endif

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
  //  if (md->show)
  md->curr = ds_zalloc(md->fi.framesize);
  //  else
  //    md->currcopy=NULL;
  md->currtmp = ds_zalloc(md->fi.framesize);
  
  md->initialized = 2;
  return DS_OK;
}

void cleanupMotionDetection(MotionDetect* md) {
  if(md->fields) {
    ds_free(md->fields);
    md->fields=0;
  }
  if(md->transs) {
    ds_list_del(md->transs, 1);
    md->transs=0;
  }
  if (md->prev) {
    ds_free(md->prev);
    md->prev = NULL;
  }
  if (md->curr) {
    ds_free(md->curr);
    md->curr = NULL;
  }
  if (md->currtmp) {
    ds_free(md->currtmp);
    md->currtmp = NULL;
  }
  md->initialized = 0;
}

int motionDetection(MotionDetect* md, unsigned char *frame) {
  assert(md->initialized==2);

  md->currorig = frame;
  // smoothen image to do better motion detection 
  //  (larger stepsize or eventually gradient descent (need higher resolution)
  if (md->fi.pFormat == PF_RGB) { 
    // we could calculate a grayscale version and use the YUV stuff afterwards
    // so far only YUV implemented
    memcpy(md->curr, frame, md->fi.framesize);
  } else {
    // box-kernel smoothing (plain average of pixels), which is fine for us
    boxblurYUV(md->curr, frame, md->currtmp, &md->fi, md->stepSize*1/*1.4*/, 
    	       BoxBlurNoColor);
    // two times yields tent-kernel smoothing, which may be better, but I don't
    //  think we need it
    //boxblurYUV(md->curr, md->curr, md->currtmp, &md->fi, md->stepSize*1, 
    // BoxBlurNoColor);
  }

  if (md->hasSeenOneFrame) {
    //    md->curr = frame;
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

  // copy current frame (smoothed) to prev for next frame comparison
  memcpy(md->prev, md->curr, md->fi.framesize);
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
   This routine is used in the simpleAlgorithms and may be removed at some point
   compares the two given images and returns the average absolute difference
   \param d_x shift in x direction
   \param d_y shift in y direction
*/
unsigned int compareImg(unsigned char* I1, unsigned char* I2, int width, int height,
	       int bytesPerPixel, int d_x, int d_y) {
  int i, j;
  unsigned char* p1 = NULL;
  unsigned char* p2 = NULL;
  unsigned int sum = 0;
  int effectWidth = width - abs(d_x);
  int effectHeight = height - abs(d_y);

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
unsigned int compareSubImg_thr_orc(unsigned char* const I1, unsigned char* const I2,
			   const Field* field, int width, int height, 
			   int bytesPerPixel, int d_x, int d_y, 
			   unsigned int threshold) {
  unsigned char* p1 = NULL;
  unsigned char* p2 = NULL;
  int s2 = field->size / 2;
  int j;
  unsigned int sum = 0;
  p1 = I1 + ((field->x - s2) + (field->y - s2) * width) * bytesPerPixel;
  p2 = I2 + ((field->x - s2 + d_x) + (field->y - s2 + d_y) * width)
    * bytesPerPixel;
  
  for (j = 0; j < field->size; j++) {
    unsigned int s = 0;
    image_line_difference_optimized(&s, p1, p2, field->size* bytesPerPixel);
    sum += s;
    if( sum > threshold) // no need to calculate any longer: worse than the best match
      break;
    p1 += width * bytesPerPixel;
    p2 += width * bytesPerPixel;
  }


  return sum;
}



/** \see contrastSubImg*/
double contrastSubImgYUV(MotionDetect* md, const Field* field) {
  return contrastSubImg(md->curr, field, md->fi.width, md->fi.height);
}

/**
   \see contrastSubImg_Michelson three times called with bytesPerPixel=3
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
  
  unsigned int sum=0;
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
  int stepSize = md->stepSize;

#ifdef STABVERBOSE
  // printf("%i %i %f\n", md->t, fieldnum, contr);
  FILE *f = NULL;
  char buffer[32];
  ds_snprintf(buffer, sizeof(buffer), "f%04i_%02i.dat", md->t, fieldnum);
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
      unsigned int error = compareSubImg(Y_c, Y_p, field, md->fi.width, md->fi.height,
                 1, i, j, minerror);

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
  unsigned int minerror = compareSubImg(Y_c, Y_p, field, md->fi.width, md->fi.height,
					1, 0, 0, UINT_MAX);
  // check all positions...
  for (i = -md->maxShift; i <= md->maxShift; i += stepSize) {
    for (j = -md->maxShift; j <= md->maxShift; j += stepSize) {
      if( i==0 && j==0 ) 
	continue; //no need to check this since already done      
      unsigned int error = compareSubImg(Y_c, Y_p, field, md->fi.width, md->fi.height,
					 1, i, j, minerror);
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
	unsigned int error = compareSubImg(Y_c, Y_p, field, md->fi.width,
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
  int tx = 0;
  int ty = 0;
  uint8_t *I_c = md->curr, *I_p = md->prev;
  int i, j;

  /* Here we improve speed by checking first the most probable position
     then the search paths are most effectively cut. (0,0) is a simple start    
  */
  unsigned int minerror = compareSubImg(I_c, I_p, field, md->fi.width, md->fi.height,
					3, 0, 0, UINT_MAX);
  // check all positions...
  for (i = -md->maxShift; i <= md->maxShift; i += md->stepSize) {
    for (j = -md->maxShift; j <= md->maxShift; j += md->stepSize) {
      if( i==0 && j==0 ) 
	continue; //no need to check this since already done
      unsigned int error = compareSubImg(I_c, I_p, field, md->fi.width,
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
	unsigned int error = compareSubImg(I_c, I_p, field, md->fi.width,
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
DSVector selectfields(MotionDetect* md, contrastSubImgFunc contrastfunc) {
  int i, j;
  DSVector goodflds;
  contrast_idx *ci =
    (contrast_idx*) ds_malloc(sizeof(contrast_idx) * md->fieldNum);
  ds_vector_init(&goodflds, md->fieldNum);
  
  // we split all fields into row+1 segments and take from each segment
  // the best fields
  int numsegms = (md->fieldRows + 1);
  int segmlen = md->fieldNum / (md->fieldRows + 1) + 1;
  // split the frame list into rows+1 segments
  contrast_idx *ci_segms =
    (contrast_idx*) ds_malloc(sizeof(contrast_idx) * md->fieldNum);
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
	ds_vector_append_dup(&goodflds, &ci[ci_segms[startindex+j].index],
			   sizeof(contrast_idx));
	// don't consider them in the later selection process
	ci_segms[startindex + j].contrast = 0;
      }
    }
  }
  // check whether enough fields are selected
  // printf("Phase2: %i\n", ds_list_size(goodflds));
  remaining = md->maxFields - ds_vector_size(&goodflds);
  if (remaining > 0) {
    // take the remaining from the leftovers
    qsort(ci_segms, md->fieldNum, sizeof(contrast_idx), cmp_contrast_idx);
    for (j = 0; j < remaining; j++) {
      if (ci_segms[j].contrast > 0) {
	ds_vector_append_dup(&goodflds, &ci_segms[j], sizeof(contrast_idx));
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

  DSVector goodflds = selectfields(md, contrastfunc);  
  // use all "good" fields and calculate optimal match to previous frame
  // #pragma omp parallel for shared(goodflds, md, ts, fs) // does not bring speedup
  for(index=0; index < ds_vector_size(&goodflds); index++){
    int i = ((contrast_idx*)ds_vector_get(&goodflds,index))->index;
       
    t = fieldfunc(md, &md->fields[i], i); // e.g. calcFieldTransYUV
#ifdef STABVERBOSE
    fprintf(file, "%i %i\n%f %f %i\n \n\n", md->fields[i].x, md->fields[i].y,
	    md->fields[i].x + t.x, md->fields[i].y + t.y, t.extra);
#endif
    if (t.extra != -1) { // ignore if extra == -1 (unused at the moment)
      ts[index] = t;
      fs[index] = md->fields + i;
    }
  }

  t = null_transform();
  num_trans = ds_vector_size(&goodflds); // amount of transforms we actually have
  ds_vector_del(&goodflds);
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
  drawBox(md->currorig, md->fi.width, md->fi.height, 1, field->x, field->y,
	  field->size + 2 * md->maxShift, field->size + 2 * md->maxShift, 80);
}

/** draws the field */
void drawField(MotionDetect* md, const Field* field, const Transform* t) {
  if (!md->fi.pFormat == PF_YUV)
    return;
  drawBox(md->currorig, md->fi.width, md->fi.height, 1, field->x, field->y,
	  field->size, field->size, t->extra == -1 ? 100 : 40);
}

/** draws the transform data of this field */
void drawFieldTrans(MotionDetect* md, const Field* field, const Transform* t) {
  if (!md->fi.pFormat == PF_YUV)
    return;
  drawBox(md->currorig, md->fi.width, md->fi.height, 1, field->x, field->y, 5, 5,
	  128); // draw center
  drawBox(md->currorig, md->fi.width, md->fi.height, 1, field->x + t->x, field->y
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


//#ifdef TESTING
/// plain C implementation of compareSubImg (without ORC)
unsigned int compareSubImg_thr(unsigned char* const I1, unsigned char* const I2,
			     const Field* field, int width, int height, 
			     int bytesPerPixel, int d_x, int d_y,
			     unsigned int threshold) {
  int k, j;
  unsigned char* p1 = NULL;
  unsigned char* p2 = NULL;
  int s2 = field->size / 2;
  unsigned int sum = 0;

  p1 = I1 + ((field->x - s2) + (field->y - s2) * width) * bytesPerPixel;
  p2 = I2 + ((field->x - s2 + d_x) + (field->y - s2 + d_y) * width)
    * bytesPerPixel;
  for (j = 0; j < field->size; j++) {
    for (k = 0; k < field->size * bytesPerPixel; k++) {
      sum += abs((int) *p1 - (int) *p2);
      p1++;
      p2++;
    }
    if( sum > threshold) // no need to calculate any longer: worse than the best match
      break;
    p1 += (width - field->size) * bytesPerPixel;
    p2 += (width - field->size) * bytesPerPixel;
  }
  return sum;
}

// implementation with 1 orc function, but no threshold
unsigned int compareSubImg_orc(unsigned char* const I1, unsigned char* const I2,
			   const Field* field, int width, int height, 
			   int bytesPerPixel, int d_x, int d_y, 
			   unsigned int threshold) {
  unsigned char* p1 = NULL;
  unsigned char* p2 = NULL;
  int s2 = field->size / 2;
  unsigned int sum=0;
  p1 = I1 + ((field->x - s2) + (field->y - s2) * width) * bytesPerPixel;
  p2 = I2 + ((field->x - s2 + d_x) + (field->y - s2 + d_y) * width)
    * bytesPerPixel;
  
  image_difference_optimized(&sum, p1, width * bytesPerPixel, 
                             p2, width*bytesPerPixel, 
                             field->size* bytesPerPixel , field->size);

  return sum;
}


/// plain C implementation of contrastSubImg (without ORC)
double contrastSubImg_C(unsigned char* const I, const Field* field, int width, int height) {
  int k, j;
  unsigned char* p = NULL;
  unsigned char* pstart = NULL;
  int s2 = field->size / 2;
  unsigned int sum=0;				       
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

#ifdef USE_SSE2
unsigned int compareSubImg_thr_sse2(unsigned char* const I1, unsigned char* const I2,
                     const Field* field,
                     int width, int height, int bytesPerPixel, int d_x, int d_y, unsigned int treshold)
{
    int k, j;
    unsigned char* p1 = NULL;
    unsigned char* p2 = NULL;
    int s2 = field->size / 2;
    unsigned int sum = 0;

    static unsigned char mask[16] = {0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00};    
    unsigned char row = 0;
#ifndef USE_SSE2_CMP_HOR
    unsigned char summes[16];
    int i;
#endif
    __m128i xmmsum, xmmmask;
    xmmsum = _mm_setzero_si128();
    xmmmask = _mm_loadu_si128((__m128i const*)mask);

    p1=I1 + ((field->x - s2) + (field->y - s2)*width)*bytesPerPixel;
    p2=I2 + ((field->x - s2 + d_x) + (field->y - s2 + d_y)*width)*bytesPerPixel;
    for (j = 0; j < field->size; j++){
        for (k = 0; k < field->size * bytesPerPixel; k+=16){
            {
                __m128i xmm0, xmm1, xmm2;
                xmm0 = _mm_loadu_si128((__m128i const *)p1);
                xmm1 = _mm_loadu_si128((__m128i const *)p2);

                xmm2 = _mm_subs_epu8(xmm0, xmm1);
                xmm0 = _mm_subs_epu8(xmm1, xmm0);
                xmm0 = _mm_adds_epu8(xmm0, xmm2);

                xmm1 = _mm_and_si128(xmm0, xmmmask);
                xmm0 = _mm_srli_si128(xmm0, 1);
                xmm0 = _mm_and_si128(xmm0, xmmmask);

                xmmsum = _mm_adds_epu16(xmmsum, xmm0);
                xmmsum = _mm_adds_epu16(xmmsum, xmm1);
            }

            p1+=16;
            p2+=16;

            row++;
            if (row == SSE2_CMP_SUM_ROWS) {
                row = 0;
#ifdef USE_SSE2_CMP_HOR
                {
                    __m128i xmm1;

                    xmm1 = _mm_srli_si128(xmmsum, 8);
                    xmmsum = _mm_adds_epu16(xmmsum, xmm1);

                    xmm1 = _mm_srli_si128(xmmsum, 4);
                    xmmsum = _mm_adds_epu16(xmmsum, xmm1);

                    xmm1 = _mm_srli_si128(xmmsum, 2);
                    xmmsum = _mm_adds_epu16(xmmsum, xmm1);

                    sum += _mm_extract_epi16(xmmsum, 0);
                }
#else
                _mm_storeu_si128((__m128i*)summes, xmmsum);
                for(i = 0; i < 16; i+=2)
                    sum += summes[i] + summes[i+1]*256;
#endif
                xmmsum = _mm_setzero_si128();
            }
        }
        if (sum > treshold)
            break;
        p1 += (width - field->size) * bytesPerPixel;
        p2 += (width - field->size) * bytesPerPixel;
    }

#if (SSE2_CMP_SUM_ROWS != 1) && (SSE2_CMP_SUM_ROWS != 2) && (SSE2_CMP_SUM_ROWS != 4) \
  && (SSE2_CMP_SUM_ROWS != 8) && (SSE2_CMP_SUM_ROWS != 16)
    //process all data left unprocessed
    //this part can be safely ignored if
    //SSE_SUM_ROWS = {1, 2, 4, 8, 16}
#ifdef USE_SSE2_CMP_HOR
    {
        __m128i xmm1;

        xmm1 = _mm_srli_si128(xmmsum, 8);
        xmmsum = _mm_adds_epu16(xmmsum, xmm1);

        xmm1 = _mm_srli_si128(xmmsum, 4);
        xmmsum = _mm_adds_epu16(xmmsum, xmm1);

        xmm1 = _mm_srli_si128(xmmsum, 2);
        xmmsum = _mm_adds_epu16(xmmsum, xmm1);

        sum += _mm_extract_epi16(xmmsum, 0);
    }
#else
    _mm_storeu_si128((__m128i*)summes, xmmsum);
    for(i = 0; i < 16; i+=2)
       sum += summes[i] + summes[i+1]*256;
#endif
#endif

    return sum;
}
#endif // USE_SSE2

#ifdef USE_SSE2_ASM
unsigned int compareSubImg_thr_sse2_asm(unsigned char* const I1, unsigned char* const I2,
                     const Field* field,
                     int width, int height, int bytesPerPixel, int d_x, int d_y, unsigned int treshold)
{
    unsigned char* p1 = NULL;
    unsigned char* p2 = NULL;
    int s2 = field->size / 2;
    unsigned int sum = 0;

    static unsigned char mask[16] = {0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00};    
    p1=I1 + ((field->x - s2) + (field->y - s2)*width)*bytesPerPixel;
    p2=I2 + ((field->x - s2 + d_x) + (field->y - s2 + d_y)*width)*bytesPerPixel;
    asm (
        "xor %0,%0\n"
        "pxor %%xmm4,%%xmm4\n"         //8 x 16bit partial sums
        "movdqu (%3),%%xmm3\n"         //mask

        //main loop
        "movl %4,%%edx\n"              //edx = field->size * bytesPerPixel / 16
        "mov $8,%%ecx\n"               //cx = 8
        "1:\n"

          //calc intermediate sum of abs differences for 16 bytes
          "movdqu (%1),%%xmm0\n"       //p1
          "movdqu (%2),%%xmm1\n"       //p2
          "movdqu %%xmm0,%%xmm2\n"     //xmm2 = xmm0
          "psubusb %%xmm1,%%xmm0\n"    //xmm0 = xmm0 - xmm1 (by bytes)
          "psubusb %%xmm2,%%xmm1\n"    //xmm1 = xmm1 - xmm2 (by bytes)
          "paddusb %%xmm1,%%xmm0\n"    //xmm0 = xmm0 + xmm1 (absolute difference)
          "movdqu %%xmm0,%%xmm2\n"     //xmm2 = xmm0
          "pand %%xmm3,%%xmm2\n"       //xmm2 = xmm2 & xmm3 (apply mask)
          "psrldq $1,%%xmm0\n"         //xmm0 = xmm0 >> 8 (shift by 1 byte)
          "pand %%xmm3,%%xmm0\n"       //xmm0 = xmm0 & xmm3 (apply mask)
          "paddusw %%xmm0,%%xmm4\n"    //xmm4 = xmm4 + xmm0 (by words)
          "paddusw %%xmm2,%%xmm4\n"    //xmm4 = xmm4 + xmm2 (by words)

          "add $16,%1\n"               //move to next 16 bytes (p1)
          "add $16,%2\n"               //move to next 16 bytes (p2)

          //check if we need flush sum (i.e. xmm4 is about to saturate)
          "dec %%ecx\n"
          "jnz 2f\n"                   //skip flushing if not
          //flushing...
          "movdqu %%xmm4,%%xmm0\n"
          "psrldq $8,%%xmm0\n"
          "paddusw %%xmm0,%%xmm4\n"
          "movdqu %%xmm4,%%xmm0\n"
          "psrldq $4,%%xmm0\n"
          "paddusw %%xmm0,%%xmm4\n"
          "movdqu %%xmm4,%%xmm0\n"
          "psrldq $2,%%xmm0\n"
          "paddusw %%xmm0,%%xmm4\n"
          "movd %%xmm4,%%ecx\n"
          "and $0xFFFF,%%ecx\n"
          "addl %%ecx,%0\n"
          "pxor %%xmm4,%%xmm4\n"       //clearing xmm4
          "mov $8,%%ecx\n"             //cx = 8

          //check if we need to go to another line
          "2:\n"
          "dec %%edx\n"
          "jnz 1b\n"                   //skip if not

          //move p1 and p2 to the next line
          "add %5,%1\n"
          "add %5,%2\n"
          "cmp %7,%0\n"                //if (sum > treshold)
          "ja 3f\n"                    //    break;
          "movl %4,%%edx\n"

          //check if all lines done
          "decl %6\n"
          "jnz 1b\n"                   //if not, continue looping
        "3:\n"
        :"=r"(sum)
        :"r"(p1),"r"(p2),"r"(mask),"g"(field->size * bytesPerPixel / 16),"g"((unsigned char*)((width - field->size) * bytesPerPixel)),"g"(field->size), "g"(treshold), "0"(sum)
        :"%xmm0","%xmm1","%xmm2","%xmm3","%xmm4","%ecx","%edx"
    );

    return sum;
}
#endif // USE_SSE2_ASM
//#endif // TESTING


/*
 * Local variables:
 *   c-file-style: "stroustrup"
 *   c-file-offsets: ((case-label . *) (statement-case-intro . *))
 *   indent-tabs-mode: nil
 * End:
 *
 */
