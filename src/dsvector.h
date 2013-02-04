/*
 * dsvector.h -- an dynamic array
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
#ifndef DSVECTOR_H
#define DSVECTOR_H

#include <stddef.h>

/**
   A vector for arbrary elements that resizes
*/
typedef struct dsvector_ DSVector;
struct dsvector_ {
  void**	data;
  int		buffersize;
  int		nelems;
};

/**
 * ds_vector_init:
 *     intializes a vector data structure.
 *     A vector will grow but not shrink if elements are added.
 *
 * Parameters:
 *              V: pointer to list to be initialized.
 *     buffersize: size of buffer (if known, then # of resizes are reduced)
 * Return Value:
 *     DS_OK on success,
 *     DS_ERROR on error.
 */
int ds_vector_init(DSVector *V, int buffersize);

/**
 * ds_vector_fini:
 *     finalizes a vector data structure. Frees all resources aquired,
 *     but *NOT* the data pointed by vector elements.
 *
 * Parameters:
 *     V: pointer to list to be finalized
 * Return Value:
 *     DS_OK on success,
 *     DS_ERROR on error.
 */
int ds_vector_fini(DSVector *V);

/**
 * ds_vector_del:
 *     like ds_vector_fini, but also deletes the data pointed by vector elements.
 *
 * Parameters:
 *     V: pointer to list to be finalized
 * Return Value:
 *     DS_OK on success,
 *     DS_ERROR on error.
 */
int ds_vector_del(DSVector *V);

/**
 * ds_vector_zero:
 *    deletes all data pointed to by the vector elements.
 *    sets the number of elements to 0 but does not delete buffer
*/
int ds_vector_zero(DSVector *V);

/**
 * ds_vector_size:
 *     gives the number of elements present in the vector
 *     (not the internal buffer size).
 *
 * Parameters:
 *     V: vector to be used.
 * Return Value:
 *    -1 on error,
 *    the number of elements otherwise
 */
int ds_vector_size(const DSVector *V);


/**
 * ds_vector_append:
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
 *     DS_OK on success,
 *     DS_ERROR on error.
 */
int ds_vector_append(DSVector *V, void *data);

/**
 * ds_vector_append_dup:
 *  like ds_vector_append but copies data
 */
int ds_vector_append_dup(DSVector *V, void *data, int data_size);


/* ds_vector_set:
 *      the newly inserted element BECOMES the position `pos' in the vector.
 *      and the old item is returned
 */
void* ds_vector_set(DSVector *V, int pos, void *data);

/* ds_vector_set_dup:
 *      the newly inserted element is copied and BECOMES the position `pos' in the vector
 *      and the old item is returned
 */
void* ds_vector_set_dup(DSVector *V, int pos, void *data, int data_size);


/* (to be implemented)
 * ds_vector_insert:
 *      the newly-inserted elements BECOMES the position `pos' on the vector.
 *      Position after the last -> the last.
 *      Position before the first -> the first.
 */
//int ds_vector_insert(DSVector *V, int pos, void *data);

/*
 * ds_vector_get:
 *     gives access to the data pointed by the element in the given position.
 *
 * Parameters:
 *       V: vector to be accessed.
 *     pos: position of the element on which the data will be returned.
 * Return Value:
 *     NULL on error (requested element doesn't exist)
 *     a pointer to the data belonging to the requested vector item.
 */
void *ds_vector_get(const DSVector *V, int pos);

/* to be implemented
 * ds_vector_pop:
 *     removes the element in the given position.
 *
 * Parameters:
 *       V: vector to be accessed.
 *     pos: position of the element on which the data will be returned.
 * Return Value:
 *     NULL on error (requested element doesn't exist)
 *     a pointer to the data assigned to the requested vector item.
 */
//void *ds_vector_pop(DSVector *V, int pos);


#endif /* DSVECTOR_H */

/*
 * Local variables:
 *   c-file-style: "stroustrup"
 *   c-file-offsets: ((case-label . *) (statement-case-intro . *))
 *   indent-tabs-mode: nil
 * End:
 *
 * vim: expandtab shiftwidth=4:
 */
