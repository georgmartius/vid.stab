/*
 * dcvector.c -- a double linked vector
 * (C) 2011 - Georg Martius
 *   georg dot martius at web dot de
 *
 *  This file is part of vid.stab video stabilization library
 *
 *  vid.stab is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License,
 *   WITH THE RESTRICTION for NONCOMMERICIAL USAGE see below,
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
 *  This work is licensed under the Creative Commons
 *  Attribution-NonCommercial-ShareAlike 2.5 License. To view a copy of
 *  this license, visit http://creativecommons.org/licenses/by-nc-sa/2.5/
 *  or send a letter to Creative Commons, 543 Howard Street, 5th Floor,
 *  San Francisco, California, 94105, USA.
 *  This EXCLUDES COMMERCIAL USAGE
 *
 */

#include "vsvector.h"
#include "vidstabdefines.h"
#include <assert.h>


/*************************************************************************/
int vs_vector_resize(VSVector *V, int newsize);

/*************************************************************************/

int vs_vector_init(VSVector *V, int buffersize){
  assert(V);
  if(buffersize>0){
    V->data=(void**)vs_zalloc(sizeof(void*)*buffersize);
    if(!V->data) return VS_ERROR;
  }else{
    V->data = 0;
    buffersize = 0;
  }
  V->buffersize=buffersize;
  V->nelems=0;
  return VS_OK;
}

int vs_vector_fini(VSVector *V){
  assert(V);
  if(V->data) vs_free(V->data);
  V->data = 0;
  V->buffersize=0;
  V->nelems=0;
  return VS_OK;
}

int vs_vector_del(VSVector *V){
  vs_vector_zero(V);
  return vs_vector_fini(V);
}

int vs_vector_zero(VSVector *V){
  assert(V);
  assert(V->nelems < 1 || V->data);
  int i;
  for(i=0; i < V->nelems; i++){
    if(V->data[i])
      vs_free(V->data[i]);
  }
  V->nelems=0;
  return VS_OK;

}

int vs_vector_size(const VSVector *V){
  assert(V);
  return V->nelems;
}


int vs_vector_append(VSVector *V, void *data){
  assert(V && data);
  if(!V->data || V->buffersize < 1) vs_vector_init(V,4);
  if(V->nelems >= V->buffersize){
    if(vs_vector_resize(V, V->buffersize*2)!=VS_OK) return VS_ERROR;
  }
  V->data[V->nelems]=data;
  V->nelems++;
  return VS_OK;
}

int vs_vector_append_dup(VSVector *V, void *data, int data_size){
  assert(V && data);
  if(!V->data || V->buffersize < 1) vs_vector_init(V,4);
  void* d = vs_malloc(data_size);
  if(!d) return VS_ERROR;
  memcpy(d, data, data_size);
  return vs_vector_append(V, d);
}


void *vs_vector_get(const VSVector *V, int pos){
  assert(V && V->data);
  if(pos<0 || pos >= V->nelems)
    return 0;
  else
    return V->data[pos];
}

void* vs_vector_set(VSVector *V, int pos, void *data){
  assert(V && data && pos>=0);
  if(!V->data || V->buffersize < 1) vs_vector_init(V,4);
  if(V->buffersize <= pos) {
    int nsize = V->buffersize;
    while(nsize <= pos) nsize *=2;
    if(vs_vector_resize(V, nsize)!=VS_OK) return 0; // insuficient error handling here! VS_ERROR
  }
  if(pos >= V->nelems){ // insert after end of vector
    int i;
    for(i=V->nelems; i< pos+1; i++){
      V->data[i]=0;
    }
    V->nelems=pos+1;
  }
  void* old = V->data[pos];
  V->data[pos] = data;
  return old;
}

void* vs_vector_set_dup(VSVector *V, int pos, void *data, int data_size){
  void* d = vs_malloc(data_size);
  if(!d) return 0; // insuficient error handling here! VS_ERROR
  memcpy(d, data, data_size);
  return vs_vector_set(V, pos, d);
}


int vs_vector_resize(VSVector *V, int newsize){
  assert(V && V->data);
  if(newsize<1) newsize=1;
  V->data = (void**)vs_realloc(V->data, newsize * sizeof(void*));
  V->buffersize=newsize;
  if(V->nelems>V->buffersize){
    V->nelems=V->buffersize;
  }
  if (!V->data){
    vs_log_error("VS_Vector","out of memory!");
    return VS_ERROR;
  } else
    return VS_OK;
}


/*
 * Local variables:
 *   c-file-style: "stroustrup"
 *   c-file-offsets: ((case-label . *) (statement-case-intro . *))
 *   indent-tabs-mode: nil
 *   c-basic-offset: 2 t
 * End:
 *
 * vim: expandtab shiftwidth=4:
 */

