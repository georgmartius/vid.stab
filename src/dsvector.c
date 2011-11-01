/*
 * dcvector.c -- a double linked vector
 * (C) 2011 - Georg Martius <georg -dot- maritus -at- web -dot- de>
 *
 * This file is part of transcode, a video stream processing tool.
 *
 * transcode is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * transcode is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */


#include "dsvector.h"
#include "deshakedefines.h"
#include <assert.h>


/*************************************************************************/
int ds_vector_resize(DSVector *V, int newsize);

/*************************************************************************/

int ds_vector_init(DSVector *V, int buffersize){
  assert(V);
  if(buffersize<1) buffersize=1;
  V->data=(void**)ds_zalloc(sizeof(void*)*buffersize);
  V->buffersize=buffersize;
  V->nelems=0;
  if(!V->data) return DS_ERROR;
  return DS_OK;
}

int ds_vector_fini(DSVector *V){
  assert(V && V->data);
  ds_free(V->data);
  V->data = 0;
  V->buffersize=0;
  V->nelems=0;  
  return DS_OK;
}

int ds_vector_del(DSVector *V){
  assert(V && V->data);
  int i;
  for(i=0; i < V->nelems; i++){
    if(V->data[i])
      ds_free(V->data[i]);
  }
  return ds_vector_fini(V);
}


int ds_vector_size(DSVector *V){
  assert(V && V->data);
  return V->nelems;
}


int ds_vector_append(DSVector *V, void *data){
  assert(V && V->data && data);
  if(V->nelems >= V->buffersize){
    if(!ds_vector_resize(V, V->buffersize*2)) return DS_ERROR;
  }
  V->data[V->nelems]=data;
  V->nelems++;
  return DS_OK;
}

int ds_vector_append_dup(DSVector *V, void *data, int data_size){
  assert(V && V->data && data);
  void* d = ds_malloc(data_size);
  if(!d) return DS_ERROR;
  memcpy(d, data, data_size);
  return ds_vector_append(V, d);
}


//int ds_vector_insert(DSVector *V, int pos, void *data);

void *ds_vector_get(DSVector *V, int pos){
  assert(V && V->data);
  if(pos<0 || pos >= V->nelems) 
    return 0;
  else 
    return V->data[pos];  
}

//void *ds_vector_pop(DSVector *V, int pos);


int ds_vector_resize(DSVector *V, int newsize){
  assert(V && V->data);
  if(newsize<1) newsize=1;
  V->data = (void**)ds_realloc(V->data, newsize);
  V->buffersize=newsize;
  if(V->nelems>V->buffersize) 
    V->nelems=V->buffersize;
  if (V->data) 
    return DS_OK;
  else
    return DS_ERROR;
}


/*
 * Local variables:
 *   c-file-style: "stroustrup"
 *   c-file-offsets: ((case-label . *) (statement-case-intro . *))
 *   indent-tabs-mode: nil
 * End:
 *
 * vim: expandtab shiftwidth=4:
 */

