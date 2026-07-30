/*
 *  lpsolver.h
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
#ifndef __LPSOLVER_H
#define __LPSOLVER_H

/** Minimal interface to a linear program of the form
 *
 *      minimize    c^T x
 *      subject to  rowlo_i <= (A x)_i <= rowup_i
 *                  collo_j <=    x_j  <= colup_j
 *
 *  Rows and columns are numbered 0..num-1.  Use +-VS_LP_INF for an
 *  unbounded side; setting lo == up gives an equality.
 *
 *  The point of this thin layer is that the construction of the camera path
 *  LP in l1campathoptimization.c is independent of which solver is used,
 *  so a backend can be swapped in without touching the model.
 */

/// any bound with |value| >= VS_LP_INF counts as infinite
#define VS_LP_INF 1e30

typedef struct VSLinProg VSLinProg;

/** creates an empty problem.
    @param numrows    number of constraint rows
    @param numcols    number of structural variables
    @param maxentries upper bound on the number of nonzero matrix entries
    All objective coefficients start at 0, all rows and columns start free.
    Returns NULL on allocation failure. */
VSLinProg* vs_lp_new(const char* name, int numrows, int numcols, int maxentries);

void vs_lp_free(VSLinProg* lp);

void vs_lp_set_row_bounds(VSLinProg* lp, int row, double lo, double up);
void vs_lp_set_col_bounds(VSLinProg* lp, int col, double lo, double up);
void vs_lp_set_obj(VSLinProg* lp, int col, double coef);

/** adds one nonzero entry A[row][col] = val.  Each (row,col) pair must be
    added at most once. */
void vs_lp_add_entry(VSLinProg* lp, int row, int col, double val);

/** solves the problem.  Returns VS_OK when an optimal solution was found and
    VS_ERROR otherwise (infeasible, unbounded, numerical failure, ...).
    @param verbose if nonzero the backend may print progress information */
int vs_lp_solve(VSLinProg* lp, int verbose);

/** objective value of the solution; only valid after vs_lp_solve returned VS_OK */
double vs_lp_get_obj_value(const VSLinProg* lp);

/** value of variable col; only valid after vs_lp_solve returned VS_OK */
double vs_lp_get_col_value(const VSLinProg* lp, int col);

/** name of the compiled-in backend, for logging */
const char* vs_lp_backend_name(void);

/** human readable description of why the last solve failed, or "" */
const char* vs_lp_status_msg(const VSLinProg* lp);

#endif /* __LPSOLVER_H */

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
