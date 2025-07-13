/*
 * vsvector.h -- a dynamic array
 * (C) 2011 - Georg Martius
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
 */
#ifndef VSVECTOR_H
#define VSVECTOR_H

#include <stddef.h>
#include <stdio.h>
#include "vidstab_api.h"

/**
   A vector for arbitrary elements that resizes
*/
typedef struct vsvector_ VSVector;
struct VS_API vsvector_ {
  void**  data;
  int    buffersize;
  int    nelems;
};

/**
 * vs_vector_init:
 *     intializes a vector data structure.
 *     A vector will grow but not shrink if elements are added.
 *
 * Parameters:
 *              V: pointer to list to be initialized.
 *     buffersize: size of buffer (if known, then # of resizes are reduced)
 * Return Value:
 *     VS_OK on success,
 *     VS_ERROR on error.
 */
VS_API int vs_vector_init(VSVector *V, int buffersize);

/**
 * vs_vector_fini:
 *     finalizes a vector data structure. Frees all resources aquired,
 *     but *NOT* the data pointed by vector elements.
 *
 * Parameters:
 *     V: pointer to list to be finalized
 * Return Value:
 *     VS_OK on success,
 *     VS_ERROR on error.
 */
VS_API int vs_vector_fini(VSVector *V);

/**
 * vs_vector_del:
 *     like vs_vector_fini, but also deletes the data pointed by vector elements.
 *
 * Parameters:
 *     V: pointer to list to be finalized
 * Return Value:
 *     VS_OK on success,
 *     VS_ERROR on error.
 */
VS_API int vs_vector_del(VSVector *V);

/**
 * vs_vector_zero:
 *    deletes all data pointed to by the vector elements.
 *    sets the number of elements to 0 but does not delete buffer
*/
VS_API int vs_vector_zero(VSVector *V);

/**
 * vs_vector_size:
 *     gives the number of elements present in the vector
 *     (not the internal buffer size).
 *
 * Parameters:
 *     V: vector to be used.
 * Return Value:
 *    -1 on error,
 *    the number of elements otherwise
 */
VS_API int vs_vector_size(const VSVector *V);


/**
 * vs_vector_append:
 *     append an element to the vector.
 *     The element is added to the end of the vector.
 *
 * Parameters:
 *        V: pointer to vector to be used
 *     data: pointer to data to be appended or prepend.
 *           *PLEASE NOTE* that JUST THE POINTER is copied on the newly-added
 *           element. NO deep copy is performed.
 *           The caller has to allocate memory by itself if it want to
 *           add a copy of the data.
 * Return Value:
 *     VS_OK on success,
 *     VS_ERROR on error.
 */
VS_API int vs_vector_append(VSVector *V, void *data);

/**
 * vs_vector_append_dup:
 *  like vs_vector_append but copies data
 */
VS_API int vs_vector_append_dup(VSVector *V, void *data, int data_size);


/* vs_vector_set:
 *      the newly inserted element BECOMES the position `pos' in the vector.
 *      and the old item is returned
 */
VS_API void* vs_vector_set(VSVector *V, int pos, void *data);

/* vs_vector_set_dup:
 *      the newly inserted element is copied and BECOMES the position `pos' in the vector
 *      and the old item is returned
 */
VS_API void* vs_vector_set_dup(VSVector *V, int pos, void *data, int data_size);

/*
 * vs_vector_get:
 *     gives access to the data pointed by the element in the given position.
 *
 * Parameters:
 *       V: vector to be accessed.
 *     pos: position of the element on which the data will be returned.
 * Return Value:
 *     NULL on error (requested element doesn't exist)
 *     a pointer to the data belonging to the requested vector item.
 */
VS_API void *vs_vector_get(const VSVector *V, int pos);

/*
 * vs_vector_filter:
 *      returns a new vector with elements that fulfill predicate
 *      pred(param, elem)
 */
VS_API VSVector vs_vector_filter(const VSVector *V, short (*pred)(void*, void*), void* param);

/*
 * vs_vector_concat:
 *      returns a new vector with elements of vector V1 and V2 after another
 */
VS_API VSVector vs_vector_concat(const VSVector *V1, const VSVector *V2);


/**
   A simple fixed-size double vector
*/
typedef struct vsarray_ VSArray;
struct VS_API vsarray_ {
  double* dat;
  int len;
};

/** creates an VSArray from a double array */
VS_API VSArray vs_array(double vals[], int len);

/** allocates a new (zero initialized) double array */
VS_API VSArray vs_array_new(int len);

/** adds two vectors ands stores results into c (if zero length then allocated) */
VS_API VSArray* vs_array_plus(VSArray* c, VSArray a, VSArray b);

/** scales a vector by a factor and stores results into c (if zero length then allocated) */
VS_API VSArray* vs_array_scale(VSArray* c, VSArray a, double f);

/** create a new deep copy of the vector */
VS_API VSArray vs_array_copy(VSArray a);

/** sets all elements of the vector to 0.0 */
VS_API void vs_array_zero(VSArray* a);

/** swaps the content of the two arrays */
VS_API void vs_array_swap(VSArray* a, VSArray* b);

/** free data */
VS_API void vs_array_free(VSArray a);

/** print array to file */
VS_API void vs_array_print(VSArray a, FILE* f);

#endif /* VSVECTOR_H */

/*
 * Local variables:
 *   c-file-style: "stroustrup"
 *   c-file-offsets: ((case-label . *) (statement-case-intro . *))
 *   indent-tabs-mode: nil
 * End:
 *
 * vim: expandtab shiftwidth=4:
 */
