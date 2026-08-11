/*
 *  transformtype.c
 *
 *  Copyright (C) Georg Martius - June 2007
 *
 *  SPDX-License-Identifier: LGPL-2.1-or-later
 *
 *  This file is part of vid.stab video stabilization library
 *
 *  vid.stab is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as published
 *  by the Free Software Foundation; either version 2.1 of the License, or
 *  (at your option) any later version.
 *
 *  vid.stab is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public License
 *  along with vid.stab; see the file COPYING.LESSER.  If not, see
 *  <https://www.gnu.org/licenses/>.
 *
 */
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "transformtype.h"
#include "transformtype_operations.h"
#include "vidstabdefines.h"

/***********************************************************************
 * helper functions to create and operate with transforms.
 * all functions are non-destructive
 */

/* create an initialized transform*/
VSTransform new_transform(double x, double y, double alpha,
                          double zoom, double barrel, double rshutter, int extra)
{
  VSTransform t;
  t.x        = x;
  t.y        = y;
  while (alpha < -M_PI){
    alpha += 2.0*M_PI;
  }
  while (alpha > M_PI){
    alpha -= 2.0*M_PI;
  }
  t.alpha    = alpha;
  t.zoom     = zoom;
  t.barrel   = barrel;
  t.rshutter = rshutter;
  t.extra    = extra;
  return t;
}

/* create a zero initialized transform*/
VSTransform null_transform(void)
{
  return new_transform(0, 0, 0, 0, 0, 0, 0);
}

/* adds two transforms */
VSTransform add_transforms(const VSTransform* t1, const VSTransform* t2)
{
  VSTransform t;
  t.x        = t1->x + t2->x;
  t.y        = t1->y + t2->y;
  t.alpha    = t1->alpha + t2->alpha;
  t.zoom     = t1->zoom + t2->zoom;
  t.barrel   = t1->barrel + t2->barrel;
  t.rshutter = t1->rshutter + t2->rshutter;
  t.extra    = t1->extra || t2->extra;
  return t;
}

/* like add_transform but with non-pointer signature */
VSTransform add_transforms_(const VSTransform t1, const VSTransform t2)
{
  return add_transforms(&t1, &t2);
}

/* subtracts two transforms */
VSTransform sub_transforms(const VSTransform* t1, const VSTransform* t2)
{
  VSTransform t;
  t.x        = t1->x - t2->x;
  t.y        = t1->y - t2->y;
  t.alpha    = t1->alpha - t2->alpha;
  t.zoom     = t1->zoom - t2->zoom;
  t.barrel   = t1->barrel - t2->barrel;
  t.rshutter = t1->rshutter - t2->rshutter;
  t.extra    = t1->extra || t2->extra;
  return t;
}

/* multiplies a transforms with a scalar */
VSTransform mult_transform(const VSTransform* t1, double f)
{
  VSTransform t;
  t.x        = t1->x        * f;
  t.y        = t1->y        * f;
  t.alpha    = t1->alpha    * f;
  t.zoom     = t1->zoom     * f;
  t.barrel   = t1->barrel   * f;
  t.rshutter = t1->rshutter * f;
  t.extra    = t1->extra;
  return t;
}

/* like mult_transform but with non-pointer signature */
VSTransform mult_transform_(const VSTransform t1, double f)
{
  return mult_transform(&t1,f);
}

void storeVSTransform(FILE* f, const VSTransform* t){
  fprintf(f,"Trans %lf %lf %lf %lf %i\n", t->x, t->y, t->alpha, t->zoom, t->extra);
}


double focal_from_fov(double fovDeg, int width){
  if(fovDeg <= 0.0 || fovDeg >= 180.0 || width <= 0) return 0.0;
  /* the half angle spans half the width; /360 halves and converts at once */
  return (width/2.0) / tan(fovDeg * M_PI/360.0);
}

/* out = a . b, aliasing-safe. */
static void mat3mul(const double a[9], const double b[9], double out[9]){
  double t[9];
  int i, j, k;
  for(i=0; i<3; i++)
    for(j=0; j<3; j++){
      double s = 0;
      for(k=0; k<3; k++) s += a[3*i+k] * b[3*k+j];
      t[3*i+j] = s;
    }
  for(i=0; i<9; i++) out[i] = t[i];
}

void rotation_matrix_backward(double yaw, double pitch, double roll, double r[9]){
  double cy = cos(-yaw),  sy = sin(-yaw);
  double cp = cos(pitch), sp = sin(pitch);
  double cr = cos(roll),  sr = sin(roll);
  double Ry[9] = {  cy, 0, sy,    0,  1,   0,   -sy,  0, cy };
  double Rx[9] = {   1, 0,  0,    0, cp, -sp,     0, sp, cp };
  double Rz[9] = {  cr,-sr, 0,   sr, cr,   0,     0,  0,  1 };
  /* Multiplied out rather than hand-expanded on purpose.  This runs once per
     frame per plane, so the eighteen extra multiplies do not register, and
     an algebraic slip here produces a matrix that still looks like a small
     rotation -- the failure would show up as a quietly worse fit, not as an
     obviously broken picture. */
  mat3mul(Ry, Rx, r);
  mat3mul(r,  Rz, r);
}

PreparedTransform prepare_transform(const VSTransform* t, const VSFrameInfo* fi){
  return prepare_transform_fov(t, fi, 0.0);
}

PreparedTransform prepare_transform_fov(const VSTransform* t,
                                        const VSFrameInfo* fi, double f){
  PreparedTransform pt;
  pt.t = t;
  double z = 1.0+t->zoom/100.0;
  pt.zcos_a = z*cos(t->alpha); // scaled cos
  pt.zsin_a = z*sin(t->alpha); // scaled sin
  pt.c_x    = fi->width / 2;
  pt.c_y    = fi->height / 2;
  pt.z      = z;
  pt.f      = f;
  if(f > 0.0){
    double b[9];
    /* x and y are yaw and pitch already scaled into centre-pixels by f, so
       dividing by f is what turns them back into angles. */
    rotation_matrix_backward(t->x/f, t->y/f, t->alpha, b);
    /* forward = backward^-1 = backward^T */
    pt.r[0]=b[0]; pt.r[1]=b[3]; pt.r[2]=b[6];
    pt.r[3]=b[1]; pt.r[4]=b[4]; pt.r[5]=b[7];
    pt.r[6]=b[2]; pt.r[7]=b[5]; pt.r[8]=b[8];
  }
  return pt;
}

Vec transform_vec(const PreparedTransform* pt, const Vec* v){
  double x,y;
  transform_vec_double(&x, &y, pt, v);
  Vec res = {x,y};
  return res;
}

void transform_vec_double(double* x, double* y, const PreparedTransform* pt, const Vec* v){
  double rx = v->x - pt->c_x;
  double ry = v->y - pt->c_y;
  if(pt->f > 0.0){
    /* Z is f cos(angle) to first order and the angles a stabiliser sees are
       small, so it cannot reach zero here; a frame would have to rotate most
       of the way to edge-on first. */
    double X = pt->r[0]*rx + pt->r[1]*ry + pt->r[2]*pt->f;
    double Y = pt->r[3]*rx + pt->r[4]*ry + pt->r[5]*pt->f;
    double Z = pt->r[6]*rx + pt->r[7]*ry + pt->r[8]*pt->f;
    *x = pt->z * pt->f * X/Z + pt->c_x;
    *y = pt->z * pt->f * Y/Z + pt->c_y;
    return;
  }
  *x =  pt->zcos_a * rx + pt->zsin_a * ry + pt->t->x + pt->c_x;
  *y = -pt->zsin_a * rx + pt->zcos_a * ry + pt->t->y + pt->c_y;
}

Vec sub_vec(Vec v1, Vec v2){
  Vec r = {v1.x - v2.x, v1.y - v2.y};
  return r;
}
Vec add_vec(Vec v1, Vec v2){
  Vec r = {v1.x + v2.x, v1.y + v2.y};
  return r;
}
Vec field_to_vec(Field f){
  Vec r = {f.x , f.y};
  return r;
}

/* compares a transform with respect to x (for sort function) */
int cmp_trans_x(const void *t1, const void* t2)
{
  double a = ((VSTransform*)t1)->x;
  double b = ((VSTransform*)t2)->x;
  return a < b ? -1 : ( a > b ? 1 : 0 );
}

/* compares a transform with respect to y (for sort function) */
int cmp_trans_y(const void *t1, const void* t2)
{
  double a = ((VSTransform*)t1)->y;
  double b = ((VSTransform*)t2)->y;
  return a < b ? -1 : ( a > b ? 1: 0 );
}

/* static int cmp_trans_alpha(const void *t1, const void* t2){ */
/*   double a = ((VSTransform*)t1)->alpha; */
/*   double b = ((VSTransform*)t2)->alpha; */
/*   return a < b ? -1 : ( a > b ? 1 : 0 ); */
/* } */


/* compares two double values (for sort function)*/
int cmp_double(const void *t1, const void* t2)
{
  double a = *((double*)t1);
  double b = *((double*)t2);
  return a < b ? -1 : ( a > b ? 1 : 0 );
}

/* compares two int values (for sort function)*/
int cmp_int(const void *t1, const void* t2)
{
  int a = *((int*)t1);
  int b = *((int*)t2);
  return a < b ? -1 : ( a > b ? 1 : 0 );
}

/**
 * median_xy_transform: calulcates the median of an array
 * of transforms, considering only x and y
 *
 * Parameters:
 *    transforms: array of transforms.
 *           len: length  of array
 * Return value:
 *     A new transform with x and y beeing the median of
 *     all transforms. alpha and other fields are 0.
 * Preconditions:
 *     len>0
 * Side effects:
 *     None
 */
VSTransform median_xy_transform(const VSTransform* transforms, int len)
{
  VSTransform* ts = vs_malloc(sizeof(VSTransform) * len);
  VSTransform t   = null_transform();
  memcpy(ts,transforms, sizeof(VSTransform)*len );
  int half = len/2;
  qsort(ts, len, sizeof(VSTransform), cmp_trans_x);
  t.x = len % 2 == 0 ? ts[half].x : (ts[half].x + ts[half+1].x)/2;
  qsort(ts, len, sizeof(VSTransform), cmp_trans_y);
  t.y = len % 2 == 0 ? ts[half].y : (ts[half].y + ts[half+1].y)/2;
  vs_free(ts);
  return t;
}

/**
 * cleanmean_xy_transform: calulcates the cleaned mean of an array
 * of transforms, considering only x and y
 *
 * Parameters:
 *    transforms: array of transforms.
 *           len: length  of array
 * Return value:
 *     A new transform with x and y beeing the cleaned mean
 *     (meaning upper and lower pentile are removed) of
 *     all transforms. alpha and other fields are 0.
 * Preconditions:
 *     len>0
 * Side effects:
 *     None
 */
VSTransform cleanmean_xy_transform(const VSTransform* transforms, int len)
{
  VSTransform* ts = vs_malloc(sizeof(VSTransform) * len);
  VSTransform t = null_transform();
  int i, cut = len / 5;
  memcpy(ts, transforms, sizeof(VSTransform) * len);
  qsort(ts,len, sizeof(VSTransform), cmp_trans_x);
  for (i = cut; i < len - cut; i++){ // all but cutted
    t.x += ts[i].x;
  }
  qsort(ts, len, sizeof(VSTransform), cmp_trans_y);
  for (i = cut; i < len - cut; i++){ // all but cutted
    t.y += ts[i].y;
  }
  vs_free(ts);
  return mult_transform(&t, 1.0 / (len - (2.0 * cut)));
}


/**
 * calulcates the cleaned maximum and minimum of an array of transforms,
 * considerung only x and y
 * It cuts off the upper and lower x-th percentil
 *
 * Parameters:
 *    transforms: array of transforms.
 *           len: length  of array
 *     percentil: the x-th percentil to cut off
 *           min: pointer to min (return value)
 *           max: pointer to max (return value)
 * Return value:
 *     call by reference in min and max
 * Preconditions:
 *     len>0, 0<=percentil<50
 * Side effects:
 *     only on min and max
 */
void cleanmaxmin_xy_transform(const VSTransform* transforms, int len,
                              int percentil,
                              VSTransform* min, VSTransform* max){
  VSTransform* ts = vs_malloc(sizeof(VSTransform) * len);
  int cut = len * percentil / 100;
  memcpy(ts, transforms, sizeof(VSTransform) * len);
  qsort(ts,len, sizeof(VSTransform), cmp_trans_x);
  min->x = ts[cut].x;
  max->x = ts[len-cut-1].x;
  qsort(ts, len, sizeof(VSTransform), cmp_trans_y);
  min->y = ts[cut].y;
  max->y = ts[len-cut-1].y;
  vs_free(ts);
}

/* calculates the required zoom value to have no borders visible
 */
double transform_get_required_zoom(const VSTransform* transform, int width, int height){
  return 100.0*(2.0*VS_MAX(fabs(transform->x)/width,fabs(transform->y)/height)  // translation part
                + fabs(sin(transform->alpha)));          // rotation part

}


/**
 * media: median of a double array
 *
 * Parameters:
 *            ds: array of values
 *           len: length  of array
 * Return value:
 *     the median value of the array
 * Preconditions: len>0
 * Side effects:  ds will be sorted!
 */
double median(double* ds, int len)
{
  int half=len/2;
  qsort(ds,len, sizeof(double), cmp_double);
  return len % 2 == 0 ? ds[half] : (ds[half] + ds[half+1])/2;
}


/** square of a number */
double sqr(double x){ return x*x; }

/**
 * mean: mean of a double array
 *
 * Parameters:
 *            ds: array of values
 *           len: length  of array
 * Return value: the mean value of the array
 * Preconditions: len>0
 * Side effects:  None
 */
double mean(const double* ds, int len)
{
  double sum=0;
  int i = 0;
  for (i = 0; i < len; i++)
    sum += ds[i];
  return sum / len;
}

/**
 * stddev: standard deviation of a double array
 *
 * Parameters:
 *            ds: array of values
 *           len: length  of array
 *          mean: mean of the array (@see mean())
 * Return value: the standard deviation value of the array
 * Preconditions: len>0
 * Side effects:  None
 */
double stddev(const double* ds, int len, double mean)
{
  double sum=0;
  int i = 0;
  for (i = 0; i < len; i++)
    sum += sqr(ds[i]-mean);
  return sqrt(sum / len);
}

/**
 * cleanmean: mean with cutted upper and lower pentile
 *
 * Parameters:
 *            ds: array of values
 *           len: length  of array
 *       minimum: minimal value (after cleaning) if not NULL
 *       maximum: maximal value (after cleaning) if not NULL
 * Return value:
 *     the mean value of the array without the upper
 *     and lower pentile (20% each)
 *     and minimum and maximum without the pentiles
 * Preconditions: len>0
 * Side effects:  ds will be sorted!
 */
double cleanmean(double* ds, int len, double* minimum, double* maximum)
{
  int cut    = len / 5;
  double sum = 0;
  int i      = 0;
  qsort(ds, len, sizeof(double), cmp_double);
  for (i = cut; i < len - cut; i++) { // all but first and last
    sum += ds[i];
  }
  if (minimum)
    *minimum = ds[cut];
  if (maximum)
    *maximum = ds[len-cut-1];
  return sum / (len - (2.0 * cut));
}

/************************************************/
/***************LOCALMOTION**********************/

LocalMotion null_localmotion(){
  LocalMotion lm;
  memset(&lm,0,sizeof(lm));
  return lm;
}

int* localmotions_getx(const LocalMotions* localmotions){
  int len = vs_vector_size(localmotions);
  int* xs = vs_malloc(sizeof(int) * len);
  int i;
  for (i=0; i<len; i++){
    xs[i]=LMGet(localmotions,i)->v.x;
  }
  return xs;
}

int* localmotions_gety(const LocalMotions* localmotions){
  int len = vs_vector_size(localmotions);
  int* ys = vs_malloc(sizeof(int) * len);
  int i;
  for (i=0; i<len; i++){
    ys[i]=LMGet(localmotions,i)->v.y;
  }
  return ys;
}

LocalMotion sub_localmotion(const LocalMotion* lm1, const LocalMotion* lm2){
  LocalMotion res = *lm1;
  res.v.x -= lm2->v.x;
  res.v.y -= lm2->v.y;
  return res;
}


/**
 * cleanmean_localmotions: calulcates the cleaned mean of a vector
 * of local motions considering
 *
 * Parameters:
 *    localmotions : vs_vector of local motions
 * Return value:
 *     A localmotion with vec with x and y being the cleaned mean
 *     (meaning upper and lower pentile are removed) of
 *     all local motions. all other fields are 0.
 * Preconditions:
 *     size of vector >0
 * Side effects:
 *     None
 */
LocalMotion cleanmean_localmotions(const LocalMotions* localmotions)
{
  int len = vs_vector_size(localmotions);
  int i, cut = len / 5;
  int* xs = localmotions_getx(localmotions);
  int* ys = localmotions_gety(localmotions);
  LocalMotion m = null_localmotion();
  m.v.x=0; m.v.y=0;
  qsort(xs,len, sizeof(int), cmp_int);
  for (i = cut; i < len - cut; i++){ // all but cutted
    m.v.x += xs[i];
  }
  qsort(ys, len, sizeof(int), cmp_int);
  for (i = cut; i < len - cut; i++){ // all but cutted
    m.v.y += ys[i];
  }
  vs_free(xs);
  vs_free(ys);
  m.v.x/=(len - (2.0 * cut));
  m.v.y/=(len - (2.0 * cut));
  return m;
}

VSArray localmotionsGetMatch(const LocalMotions* localmotions){
  VSArray m = vs_array_new(vs_vector_size(localmotions));
  for (int i=0; i<m.len; i++){
    m.dat[i]=LMGet(localmotions,i)->match;
  }
  return m;
}


/*
 * Local variables:
 *   c-file-style: "stroustrup"
 *   c-file-offsets: ((case-label . *) (statement-case-intro . *))
 *   indent-tabs-mode: nil
 *   c-basic-offset: 2 t
 * End:
 *
 * vim: expandtab shiftwidth=2:
 */
