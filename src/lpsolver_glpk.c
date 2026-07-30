/*
 *  lpsolver_glpk.c
 *
 *  GLPK backend for the minimal LP interface in lpsolver.h.
 *
 *  Copyright (C) Georg Martius - January 2014 - 2026
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

#include "lpsolver.h"

#ifdef USE_GLPK

#include "vidstabdefines.h"
#include <glpk.h>
#include <string.h>
#include <stdio.h>

struct VSLinProg {
  glp_prob* lp;
  int    numrows;
  int    numcols;
  int    maxentries;
  int    numentries;   // number of entries stored so far
  int*   ia;           // 1-based triplet arrays as required by glp_load_matrix
  int*   ja;
  double* ar;
  int    solved;
  const char* status;
};

/** translates a (lo,up) pair into GLPK's bound type */
static int boundtype(double lo, double up){
  int lofin = lo > -VS_LP_INF;
  int upfin = up <  VS_LP_INF;
  if (!lofin && !upfin) return GLP_FR;
  if (lofin && !upfin)  return GLP_LO;
  if (!lofin && upfin)  return GLP_UP;
  return (lo == up) ? GLP_FX : GLP_DB;
}

VSLinProg* vs_lp_new(const char* name, int numrows, int numcols, int maxentries){
  if (numrows <= 0 || numcols <= 0 || maxentries <= 0) return 0;
  VSLinProg* p = (VSLinProg*)vs_zalloc(sizeof(VSLinProg));
  if (!p) return 0;
  p->numrows    = numrows;
  p->numcols    = numcols;
  p->maxentries = maxentries;
  p->numentries = 0;
  p->status     = "";
  /* +1 because GLPK's triplet arrays are 1-based */
  p->ia = (int*)   vs_malloc(sizeof(int)    * (maxentries + 1));
  p->ja = (int*)   vs_malloc(sizeof(int)    * (maxentries + 1));
  p->ar = (double*)vs_malloc(sizeof(double) * (maxentries + 1));
  p->lp = glp_create_prob();
  if (!p->ia || !p->ja || !p->ar || !p->lp) {
    vs_lp_free(p);
    return 0;
  }
  glp_set_prob_name(p->lp, name ? name : "vid.stab");
  glp_set_obj_dir(p->lp, GLP_MIN);
  glp_add_rows(p->lp, numrows);
  glp_add_cols(p->lp, numcols);
  /* GLPK's default row type is GLP_FX with bound 0; we want free rows until
     the model says otherwise, so that a forgotten bound does not silently
     turn into an equality. */
  for (int i = 1; i <= numrows; i++) glp_set_row_bnds(p->lp, i, GLP_FR, 0.0, 0.0);
  return p;
}

void vs_lp_free(VSLinProg* p){
  if (!p) return;
  if (p->lp) glp_delete_prob(p->lp);
  if (p->ia) vs_free(p->ia);
  if (p->ja) vs_free(p->ja);
  if (p->ar) vs_free(p->ar);
  vs_free(p);
}

void vs_lp_set_row_bounds(VSLinProg* p, int row, double lo, double up){
  if (!p || row < 0 || row >= p->numrows) return;
  glp_set_row_bnds(p->lp, row + 1, boundtype(lo, up), lo, up);
}

void vs_lp_set_col_bounds(VSLinProg* p, int col, double lo, double up){
  if (!p || col < 0 || col >= p->numcols) return;
  glp_set_col_bnds(p->lp, col + 1, boundtype(lo, up), lo, up);
}

void vs_lp_set_obj(VSLinProg* p, int col, double coef){
  if (!p || col < 0 || col >= p->numcols) return;
  glp_set_obj_coef(p->lp, col + 1, coef);
}

void vs_lp_add_entry(VSLinProg* p, int row, int col, double val){
  if (!p) return;
  if (row < 0 || row >= p->numrows || col < 0 || col >= p->numcols) return;
  if (p->numentries >= p->maxentries) {  // must not happen; guarded for safety
    p->status = "matrix entry buffer overflow";
    return;
  }
  int k = ++p->numentries;
  p->ia[k] = row + 1;
  p->ja[k] = col + 1;
  p->ar[k] = val;
}

int vs_lp_solve(VSLinProg* p, int verbose){
  if (!p) return VS_ERROR;
  p->solved = 0;
  /* GLPK writes to the terminal from glp_scale_prob() too, which the message
     level in glp_smcp does not cover, so silence it globally instead */
  glp_term_out(verbose ? GLP_ON : GLP_OFF);
  glp_load_matrix(p->lp, p->numentries, p->ia, p->ja, p->ar);
  /* Scaling matters here: the translational parameters live on a pixel scale
     while a and b are around 1, so the columns differ by three orders of
     magnitude. */
  glp_scale_prob(p->lp, GLP_SF_AUTO);

  glp_smcp parm;
  glp_init_smcp(&parm);
  parm.msg_lev = verbose ? GLP_MSG_ALL : GLP_MSG_ERR;
  parm.presolve = GLP_ON;
  /* dual simplex with a primal fallback: the smoothness rows make the primal
     heavily degenerate, the dual copes far better */
  parm.meth = GLP_DUALP;

  int ret = glp_simplex(p->lp, &parm);
  if (ret != 0) {
    /* presolve can fail on numerically awkward instances; retry without it,
       which also lets us query a proper solution status afterwards */
    parm.presolve = GLP_OFF;
    ret = glp_simplex(p->lp, &parm);
  }
  glp_term_out(GLP_ON);
  if (ret != 0) {
    p->status = "simplex failed";
    return VS_ERROR;
  }
  switch (glp_get_status(p->lp)) {
   case GLP_OPT:    p->solved = 1; p->status = ""; return VS_OK;
   case GLP_FEAS:   p->status = "solution feasible but not proven optimal"; break;
   case GLP_INFEAS: p->status = "solution infeasible"; break;
   case GLP_NOFEAS: p->status = "problem has no feasible solution"; break;
   case GLP_UNBND:  p->status = "problem is unbounded"; break;
   default:         p->status = "solution status undefined"; break;
  }
  return VS_ERROR;
}

double vs_lp_get_obj_value(const VSLinProg* p){
  return (p && p->solved) ? glp_get_obj_val(p->lp) : 0.0;
}

double vs_lp_get_col_value(const VSLinProg* p, int col){
  if (!p || !p->solved || col < 0 || col >= p->numcols) return 0.0;
  return glp_get_col_prim(p->lp, col + 1);
}

const char* vs_lp_status_msg(const VSLinProg* p){
  return (p && p->status) ? p->status : "";
}

const char* vs_lp_backend_name(void){
  static char name[64];
  snprintf(name, sizeof(name), "GLPK %s", glp_version());
  return name;
}

#endif /* USE_GLPK */

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
