/*
 *  l1campathoptimization.c
 *
 *  Copyright (C) Georg Martius - Jan - 2014
 *   georg dot martius at web dot de
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

#include "l1campathoptimization.h"
#include "vidstabdefines.h"
#include "transformtype_operations.h"
#include <assert.h>

#ifdef USE_GLPK
#include <glpk.h>

#define X 0
#define Y 1
#define A 2
#define B 3

#define P 0
#define E1 1
#define E2 2
#define E3 3
#define CORNER 4

/** return the row number in the LP constraint Matrix (starting with 1)
    @param e: E1,E2,E3,CORNER: constraint for first, second or third derivative, and corners
    @param t: 0..N: time // note: for e1 .. N-1, e2 ..N-2 and e3 ..N-3
    @param param: 0..3: X,Y,A,B (or for e4 it is just the corner coord number)
    @param upperorlower: 0,1: upper or lower constraint
    @param N: number of frame pairs
 */
int getRowNum(int e, int t, int param, int upperorlower, int N){
  if(e<CORNER)
    return 8*((e-1)*(N)-(e*(e-1)/2)) + t*8 + param*2 + upperorlower + 1;
  else
    return getRowNum(3, N-3, 0, 0 ,N) + t*4 + param;
}

/** return the col number in the LP constraint Matrix (starting with 1)
    @param p_e: P, E1, E2, E3: 0:parameter, 1-3 constraint for first, second or third derivative
    @param t: 0..N-1: time // note: for e1 ..N-1, e2 ..N-2 and e3 ..N-3
    @param param: 0..3: X,Y,A,B
    @param N: number of frame pairs
 */
int getColNum(int p_e, int t, int param, int N){
  return 4*((p_e)*N-(p_e*(p_e-1)/2)) + t*4 + param + 1;
}

// helper macros to make the code below more compact
#define ROW(e,time,param,upperorlower) row=getRowNum(e, time, param, upperorlower, N);
#define SET(ep,offset,param,val) ia[idx] = row, ja[idx] = getColNum(ep, t+offset, param, N), ar[idx] =  val; idx++;
#define F(offset, param) (inversetransforms[t+(offset)].param)

void dumpConstraintMatrix(glp_prob *lp, int N){
  int rows = glp_get_num_rows(lp);
  int cols = glp_get_num_cols(lp);
  double** mat = vs_zalloc(sizeof(double*)*(rows+1));
  for(int i=1; i<=rows; i++){
    mat[i] = vs_zalloc(sizeof(double)*(cols+1));
  }
  int* ind    = vs_zalloc(sizeof(int)*(cols+1));
  double* val = vs_zalloc(sizeof(double)*(cols+1));
  for(int i=1; i<=rows; i++){
    int len= glp_get_mat_row(lp, i, ind, val);
    for(int k=1; k<=len;k++){
      mat[i][ind[k]]=val[k];
    }
  };
  // matrix complete -> Print to file
  FILE* f=fopen("matrix.log","w");
  fprintf(f,"Name  \tbound    \t");
  for(int j=0; j<N; j++)   for(int k=0;k<4;k++) fprintf(f,"P%i-%i       \t", j,k);
  for(int j=0; j<N-1; j++) for(int k=0;k<4;k++) fprintf(f,"E1_%i-%i     \t", j,k);
  for(int j=0; j<N-2; j++) for(int k=0;k<4;k++) fprintf(f,"E2_%i-%i     \t", j,k);
  for(int j=0; j<N-3; j++) for(int k=0;k<4;k++) fprintf(f,"E3_%i-%i     \t", j,k);
  fprintf(f,"\n");

  const int e1=(N-1)*8,e2=((N-1)+(N-2))*8,e3=((N-1)+(N-2)+(N-3))*8;
  for(int i=1; i<=rows; i++){
    int type = glp_get_row_type(lp, i);
    if(i-1<e1) fprintf(f,"E1_%i-%i \t",      (i-1)/8,(i-1)%8);
    else if(i-1<e2) fprintf(f,"E2_%i-%i \t", (i-e1-1)/8,(i-e1-1)%8);
    else if(i-1<e3) fprintf(f,"E3_%i-%i \t", (i-e2-1)/8,(i-e2-1)%8);
    else fprintf(f,"CO_%i-%i \t",            (i-e3-1)/4,(i-e3-1)%4);
    if(type==GLP_LO){
      fprintf(f,"%f\t <= ", glp_get_row_lb(lp,i));
    }else if(type==GLP_UP){
      fprintf(f,"%f\t >= ", glp_get_row_ub(lp,i));
    }else if(type==GLP_DB){
      fprintf(f,"%f\t <= ", glp_get_row_lb(lp,i));
    }else {
      fprintf(stderr,"unknown row type: %i (%i)\n",type,i);
      fprintf(f,"????????\t ?? ");
    }
    for(int j=1; j<=cols; j++){
      fprintf(f,"%f\t", mat[i][j]);
    }
    if(type==GLP_DB){
      fprintf(f," <= %f", glp_get_row_ub(lp,i));
    }
    fprintf(f,"\n");
  }
  fclose(f);
}


/** optimal camera path algorithm as described in
   @INPROCEEDINGS{GrundmannKwatra2011,
    author = {M. Grundmann and V. Kwatra and I. Essa },
    title = {Auto-Directed Video Stabilization with Robust L1 Optimal Camera Paths},
    booktitle = {IEEE Conference on Computer Vision and Pattern Recognition (CVPR)},
    year = {2011},}
  */
int cameraPathOptimalL1Internal(VSTransformData* td, VSTransformationsLS* trans){
  VSTransformLS* ts = trans->ts;
  if (trans->len < 1)
    return VS_ERROR;
  if (td->conf.verbose & VS_DEBUG) {
    vs_log_msg(td->conf.modName, "Optimization of camera path:");
  }
  fprintf(stderr,"L1Opt\n");
  //assume relative transforms
  if (!td->conf.relative) {
    vs_log_error(td->conf.modName, "require relative transforms\n");
    return VS_ERROR;
  }

  // inclusion constraint
  // TODO: need more documentation
  double x2=td->fiSrc.width/2;
  double y2=td->fiSrc.height/2;
  // TODO: configurable
  double maxZoom = 1+(10.0/100.0);
  // size of crop frame
  double xI2=td->fiSrc.width/2 / maxZoom;
  double yI2=td->fiSrc.height/2 / maxZoom;
  double xO2=td->fiSrc.width/2  * maxZoom; // to outside
  double yO2=td->fiSrc.height/2 * maxZoom; // to outside


  if (td->conf.smoothing>0) {
    glp_prob *lp;

    lp = glp_create_prob();
    glp_set_prob_name(lp, "vid.stab");
    glp_set_obj_dir(lp, GLP_MIN);

    int N = trans->len;
    // we need to convert the transform to inverse transforms
    VSTransformLS* inversetransforms = (VSTransformLS*)vs_malloc(sizeof(VSTransformLS)*N);
    for(int t=0; t<N; t++){
      inversetransforms[t] = invert_transformLS(&ts[t]);
    }

    // Rows: contstraint equations
    // for each param we have two contraints (positive and negative) for each requirement (they differ in length) +
    //   inclusion constraints (-1 because the indices are 1.. in glp so we have always +1)
    int numrows =  getRowNum(CORNER,N,0,0,N)-1;
    // Cols: variables and slack variables
    //             params + 3 constraints for 4 parameters per frame
    int numcols = getColNum(E3,N-3,0,N)-1;
    glp_add_rows(lp, numrows);
    glp_add_cols(lp, numcols);

    // weights for path
    double w1 = 10; // 10
    double w2 = 1; // 1.0;
    double w3 = 100; //100;

    for(int t=0; t< N; t++){
      if(t<N-1){
        glp_set_row_bnds(lp, getRowNum(E1,t,X,0,N), GLP_LO, F(1,x), 0.0); // |DW|
        glp_set_row_bnds(lp, getRowNum(E1,t,X,1,N), GLP_UP, 0.0, F(1,x)); // |DW|
        glp_set_row_bnds(lp, getRowNum(E1,t,Y,0,N), GLP_LO, F(1,y), 0.0); // |DW|
        glp_set_row_bnds(lp, getRowNum(E1,t,Y,1,N), GLP_UP, 0.0, F(1,y)); // |DW|
        for(int p=A; p<=B; p++){
          glp_set_row_bnds(lp, getRowNum(E1,t,p,0,N), GLP_LO, 0.0, 0.0); // |DW|
          glp_set_row_bnds(lp, getRowNum(E1,t,p,1,N), GLP_UP, 0.0, 0.0); // |DW|
        }
      }
      if(t<N-2){
        glp_set_row_bnds(lp, getRowNum(E2,t,X,0,N), GLP_LO, F(2,x) - F(1,x), 0.0); // |DW|
        glp_set_row_bnds(lp, getRowNum(E2,t,X,1,N), GLP_UP, 0.0, F(2,x) - F(1,x)); // |DW|
        glp_set_row_bnds(lp, getRowNum(E2,t,Y,0,N), GLP_LO, F(2,y) - F(1,y), 0.0); // |DW|
        glp_set_row_bnds(lp, getRowNum(E2,t,Y,1,N), GLP_UP, 0.0, F(2,y) - F(1,y)); // |DW|
        for(int p=A; p<=B; p++){
          glp_set_row_bnds(lp, getRowNum(E2,t,p,0,N), GLP_LO, 0.0, 0.0); // |D^2W|
          glp_set_row_bnds(lp, getRowNum(E2,t,p,1,N), GLP_UP, 0.0, 0.0); // |D^2W|
        }
      }
      if(t<N-3){
        glp_set_row_bnds(lp, getRowNum(E3,t,X,0,N), GLP_LO, F(3,x) - 2*F(2,x) + F(1,x), 0.0); // |D^3W|
        glp_set_row_bnds(lp, getRowNum(E3,t,X,1,N), GLP_UP, 0.0, F(3,x) - 2*F(2,x) + F(1,x)); // |D^3W|
        glp_set_row_bnds(lp, getRowNum(E3,t,Y,0,N), GLP_LO, F(3,y) - 2*F(2,y) + F(1,y), 0.0); // |D^3W|
        glp_set_row_bnds(lp, getRowNum(E3,t,Y,1,N), GLP_UP, 0.0, F(3,y) - 2*F(2,y) + F(1,y)); // |D^3W|
        for(int p=A; p<=B; p++){
          glp_set_row_bnds(lp, getRowNum(E3,t,p,0,N), GLP_LO, 0.0, 0.0); // |D^3W|
          glp_set_row_bnds(lp, getRowNum(E3,t,p,1,N), GLP_UP, 0.0, 0.0); // |D^3W|
        }
      }

      for(int p=X; p<=B; p++){
        glp_set_row_bnds(lp, getRowNum(CORNER,t,0,0,N), GLP_DB, -xO2, -xI2); // Corner
        glp_set_row_bnds(lp, getRowNum(CORNER,t,1,0,N), GLP_DB, -yO2, -yI2); // Corner
        glp_set_row_bnds(lp, getRowNum(CORNER,t,2,0,N), GLP_DB,  xI2,  xO2); // Corner
        glp_set_row_bnds(lp, getRowNum(CORNER,t,3,0,N), GLP_DB,  yI2,  yO2); // Corner
        /* glp_set_row_bnds(lp, getRowNum(CORNER,t,0,0,N), GLP_DB, -20, 20); // proximity */
        /* glp_set_row_bnds(lp, getRowNum(CORNER,t,1,0,N), GLP_DB, -20, 20); // proximity */
        /* glp_set_row_bnds(lp, getRowNum(CORNER,t,2,0,N), GLP_DB,  0.9, 1.1); // proximity */
        /* glp_set_row_bnds(lp, getRowNum(CORNER,t,3,0,N), GLP_DB, -0.1, 0.1); // proximity */
      }
    }

    // set columns (structural variables) and weights.
    for(int t=0; t< N; t++){
      for(int p=X; p<=B; p++){
        // params (no constraints)
        glp_set_col_bnds(lp, getColNum(P,t,p,N), GLP_FR, 0.0, 0.0);
        glp_set_obj_coef(lp, getColNum(P,t,p,N), 0.0);
        // slack variables (e1-e3) (bounded from below)
        if(t<N-1) {
          glp_set_col_bnds(lp, getColNum(E1,t,p,N), GLP_LO, 0.0, 0.0);
          glp_set_obj_coef(lp, getColNum(E1,t,p,N), w1);
        }
        if(t<N-2) {
          glp_set_col_bnds(lp, getColNum(E2,t,p,N), GLP_LO, 0.0, 0.0);
          glp_set_obj_coef(lp, getColNum(E2,t,p,N), w2);
        }
        if(t<N-3) {
          glp_set_col_bnds(lp, getColNum(E3,t,p,N), GLP_LO, 0.0, 0.0);
          glp_set_obj_coef(lp, getColNum(E3,t,p,N), w3);
        }
      }
    }
    int*    ia=vs_malloc(sizeof(int)*20*numrows); // we need not more than 20 entries per row
    int*    ja=vs_malloc(sizeof(int)*20*numrows);
    double* ar=vs_malloc(sizeof(double)*20*numrows);
    int row;

    int idx=1;
    for(int t=0; t< N; t++){
      // see formulas derived in the Mathematica file in the docs
      // replacement rules:
      //  \([+-]?\)e\([123]\)\(\w\) -> SET(E\2,0,\3,\1 1)  # replace eXX's
      //  + (\(.*?\)) w\([0123]\)\(\w\) -> SET(P,\2,\3,\1) # + (XX) wXX's
      //  \([+-]\) w\([0123]\)\(\w\) -> SET(P,\2,\3,\1 1)  # +- wXX's
      //  \([+-]\) \([123]\) w\([0123]\)\(\w\) -> SET(P,\3,\4,\1\2)  # +- X wXX's
      //  \([+-]\) \([1234]\) f\([0123]\)\(\w\) w\([0123]\)\(\w\) -> SET(P,\5,\6,\1\2*F(\3,\4))   # +- X fXX wXX's
      //  \([+-]\) f\([0123]\)\(\w\) w\([0123]\)\(\w\) -> SET(P,\4,\5,\1F(\2,\3)) # +- fXX wXX's
      //  \([+-]\) f\([0123]\)\(\w\) -> \1F(\2,\3)                  # +- fXX
      //  \([+-]\) \([1234]\) f\([0123]\)\(\w\) -> \1\2*F(\3,\4)     # +- X fXX
      // |DW|
      if(t<N-1) {       // |D^2W|
        //   f1x <=   e1X          + w0X            - f1a w1X           - f1b w1Y
        ROW(E1, t, X, 0) SET(E1,0,X,1)  SET(P,0,X,1)  SET(P,1,X,-F(1,a)) SET(P,1,Y,-F(1,b));
        //   f1x >=  -e1X          + w0X            - f1a w1X           - f1b w1Y
        ROW(E1, t, X, 1) SET(E1,0,X,-1) SET(P,0,X,1) SET(P,1,X,-F(1,a)) SET(P,1,Y,-F(1,b));
        ROW(E1, t, Y, 0) SET(E1,0,Y,1)  SET(P,0,Y,1) SET(P,1,X,+F(1,b)) SET(P,1,Y,-F(1,a));
        ROW(E1, t, Y, 1) SET(E1,0,Y,-1) SET(P,0,Y,1) SET(P,1,X,+F(1,b)) SET(P,1,Y,-F(1,a));
        ROW(E1, t, A, 0) SET(E1,0,A,1)  SET(P,0,A,1) SET(P,1,A,-F(1,a)) SET(P,1,B,+F(1,b));
        ROW(E1, t, A, 1) SET(E1,0,A,-1) SET(P,0,A,1) SET(P,1,A,-F(1,a)) SET(P,1,B,+F(1,b));
        ROW(E1, t, B, 0) SET(E1,0,B,1)  SET(P,0,B,1) SET(P,1,A,-F(1,b)) SET(P,1,B,-F(1,a));
        ROW(E1, t, B, 1) SET(E1,0,B,-1) SET(P,0,B,1) SET(P,1,A,-F(1,b)) SET(P,1,B,-F(1,a));
      }
      if(t<N-2) {       // |D^2W|
        /* //  f2x - f1x <=   e2X           - w0X         + (1 + f1a) w1X     + f1b w1Y          - f2a w2X          - f2b w2Y */
        ROW(E2, t, X, 0) SET(E2,0,X, 1) SET(P,0,X,-1)  SET(P,1,X,1+F(1,a)) SET(P,1,Y,F(1,b))  SET(P,2,X,-F(2,a)) SET(P,2,Y, -F(2,b));
        ROW(E2, t, X, 1) SET(E2,0,X,-1) SET(P,0,X,-1)  SET(P,1,X,1+F(1,a)) SET(P,1,Y,F(1,b))  SET(P,2,X,-F(2,a)) SET(P,2,Y, -F(2,b));
        // f2y - f1y <=         e2Y      - w0Y         - f1b w1X           + (1 + f1a) w1Y    + f2b w2X          - f2a w2Y
        ROW(E2, t, Y, 0) SET(E2,0,Y, 1) SET(P,0,Y,-1) SET(P,1,X,-F(1,b)) SET(P,1,Y,1+F(1,a))  SET(P,2,X,+F(2,b)) SET(P,2,Y,-F(2,a));
        ROW(E2, t, Y, 1) SET(E2,0,Y,-1) SET(P,0,Y,-1) SET(P,1,X,-F(1,b)) SET(P,1,Y,1+F(1,a))  SET(P,2,X,+F(2,b)) SET(P,2,Y,-F(2,a));
        // 0         <=         e2A     - w0A         + (1 + f1a) w1A     - f1b w1B           - f2a w2A          + f2b w2B
        ROW(E2, t, A, 0) SET(E2,0,A, 1) SET(P,0,A,-1) SET(P,1,A,1+F(1,a)) SET(P,1,B,-F(1,b))  SET(P,2,A,-F(2,a)) SET(P,2,B,+F(2,b));
        ROW(E2, t, A, 1) SET(E2,0,A,-1) SET(P,0,A,-1) SET(P,1,A,1+F(1,a)) SET(P,1,B,-F(1,b))  SET(P,2,A,-F(2,a)) SET(P,2,B,+F(2,b));
        // 0         <=         e2B     - w0B         + f1b w1A         + (1 + f1a) w1B       - f2b w2A          - f2a w2B
        ROW(E2, t, B, 0) SET(E2,0,B, 1) SET(P,0,B,-1) SET(P,1,A,F(1,b)) SET(P,1,B,1+F(1,a))   SET(P,2,A,-F(2,b)) SET(P,2,B,-F(2,a));
        ROW(E2, t, B, 1) SET(E2,0,B,-1) SET(P,0,B,-1) SET(P,1,A,F(1,b)) SET(P,1,B,1+F(1,a))   SET(P,2,A,-F(2,b)) SET(P,2,B,-F(2,a));
      }
      //      printf("3 %i %i\n", t, idx );
      if(t<N-3) {       // |D^3W|
        // f3x - 2 f2x + f1x <= e3X + w0X + (-2 - f1a) w1X - f1b w1Y + (1 + 2 f2a) w2X + 2 f2b w2Y - f3a w3X - f3b w3Y
        ROW(E3, t, X, 0) SET(E3,0,X, 1) SET(P,0,X,+ 1) SET(P,1,X,-2 -F(1,a)) SET(P,1,Y,-F(1,b)) SET(P,2,X,1 +2*F(2,a)) SET(P,2,Y,+2*F(2,b)) SET(P,3,X,-F(3,a)) SET(P,3,Y,-F(3,b));
        ROW(E3, t, X, 1) SET(E3,0,X,-1) SET(P,0,X,+ 1) SET(P,1,X,-2 -F(1,a)) SET(P,1,Y,-F(1,b)) SET(P,2,X,1 +2*F(2,a)) SET(P,2,Y,+2*F(2,b)) SET(P,3,X,-F(3,a)) SET(P,3,Y,-F(3,b));
        // f3y - 2 f2y + f1y <= e3Y + w0Y + f1b w1X + (-2 - f1a) w1Y - 2 f2b w2X + (1 + 2 f2a) w2Y + f3b w3X - f3a w3Y
        ROW(E3, t, Y, 0) SET(E3,0,Y, 1) SET(P,0,Y,+ 1) SET(P,1,X,+F(1,b)) SET(P,1,Y,-2 -F(1,a)) SET(P,2,X,-2*F(2,b)) SET(P,2,Y,1 +2*F(2,a)) SET(P,3,X,+F(3,b)) SET(P,3,Y,-F(3,a));
        ROW(E3, t, Y, 1) SET(E3,0,Y,-1) SET(P,0,Y,+ 1) SET(P,1,X,+F(1,b)) SET(P,1,Y,-2 -F(1,a)) SET(P,2,X,-2*F(2,b)) SET(P,2,Y,1 +2*F(2,a)) SET(P,3,X,+F(3,b)) SET(P,3,Y,-F(3,a));

        // 0 <= e3A + w0A + (-2 - f1a) w1A + f1b w1B + (1 + 2 f2a) w2A - 2 f2b w2B - f3a w3A + f3b w3B
        ROW(E3, t, A, 0) SET(E3,0,A, 1) SET(P,0,A,+ 1) SET(P,1,A,-2 -F(1,a)) SET(P,1,B,+F(1,b)) SET(P,2,A,1 +2*F(2,a)) SET(P,2,B,-2*F(2,b)) SET(P,3,A,-F(3,a)) SET(P,3,B,+F(3,b));
        ROW(E3, t, A, 1) SET(E3,0,A,-1) SET(P,0,A,+ 1) SET(P,1,A,-2 -F(1,a)) SET(P,1,B,+F(1,b)) SET(P,2,A,1 +2*F(2,a)) SET(P,2,B,-2*F(2,b)) SET(P,3,A,-F(3,a)) SET(P,3,B,+F(3,b));

        // 0 <= e3B + w0B - f1b w1A + (-2 - f1a) w1B + 2 f2b w2A + (1 + 2 f2a) w2B - f3b w3A - f3a w3B
        ROW(E3, t, B, 0) SET(E3,0,B, 1) SET(P,0,B,+ 1) SET(P,1,A,-F(1,b)) SET(P,1,B,-2 -F(1,a)) SET(P,2,A,+2*F(2,b)) SET(P,2,B,1 +2*F(2,a)) SET(P,3,A,-F(3,b)) SET(P,3,B,-F(3,a));
        ROW(E3, t, B, 1) SET(E3,0,B, 1) SET(P,0,B,+ 1) SET(P,1,A,-F(1,b)) SET(P,1,B,-2 -F(1,a)) SET(P,2,A,+2*F(2,b)) SET(P,2,B,1 +2*F(2,a)) SET(P,3,A,-F(3,b)) SET(P,3,B,-F(3,a));
      }

      // inclusion and proximity (corners)
      //    -xO2  <= w0X        - w0A x2          - w0B y2 <= -xI2
      ROW(CORNER,t,0,0) SET(P,0,X,1) SET(P,0,A,-x2) SET(P,0,B,-y2);
      //    -yO2  <= w0Y        + w0B x2         - w0A y2 <= -yI2
      ROW(CORNER,t,1,0) SET(P,0,Y,1) SET(P,0,B, x2) SET(P,0,A,-y2);
      //      xI2 <= w0X        + w0A x2         + w0B y2 <= xO2
      ROW(CORNER,t,2,0) SET(P,0,X,1) SET(P,0,A, x2) SET(P,0,B, y2);
      //      yI2 <= w0Y        - w0B x2         + w0A y2 <= yO2
      ROW(CORNER,t,3,0) SET(P,0,Y,1) SET(P,0,B,-x2) SET(P,0,A, y2);
      /* //  proximity TEST */
      /* ROW(CORNER,t,0,0) SET(P,0,X,1); */
      /* ROW(CORNER,t,1,0) SET(P,0,Y,1); */
      /* ROW(CORNER,t,2,0) SET(P,0,A,1); */
      /* ROW(CORNER,t,3,0) SET(P,0,B,1); */
    }
    int numentries = idx-1;
    glp_load_matrix(lp, numentries, ia, ja, ar);

    dumpConstraintMatrix(lp,N);

    glp_simplex(lp, NULL);
    //glp_interior(lp, NULL);
    double z = glp_get_obj_val(lp);
    printf("\nz = %g\n", z);
    // retrieve optimized transforms and convert them to forward transforms
    for(int t=0; t< N; t++){
      ts[t].x = glp_get_col_prim(lp, getColNum(P,t,X,N));
      ts[t].y = glp_get_col_prim(lp, getColNum(P,t,Y,N));
      ts[t].a = glp_get_col_prim(lp, getColNum(P,t,A,N));
      ts[t].b = glp_get_col_prim(lp, getColNum(P,t,B,N));
      storeVSTransformLS(stdout,&ts[t]);
      ts[t]=invert_transformLS(&ts[t]);
    }

    glp_delete_prob(lp);
    vs_free(ia); vs_free(ja); vs_free(ar);
    vs_free(inversetransforms);
  }
  return VS_OK;
}

int cameraPathOptimalL1(VSTransformData* td, VSTransformations* trans){
  // convert to linear similarity transforms (LS)
  VSTransformationsLS ts=transformationsAZtoLS(trans);
  int status = cameraPathOptimalL1Internal(td, &ts);
  if(status==VS_OK){
    // convert back
    for(int i=0; i<ts.len; i++){
      trans->ts[i]=transformLStoAZ(&ts.ts[i]);
    }
  }
  vs_free(ts.ts);
  return status;
}

#else
// dummy implementation with error
int cameraPathL1(VSTransformData* td, VSTransformations* trans){
  return VS_ERROR;
}
#endif
