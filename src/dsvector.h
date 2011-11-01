/*
 * dsvector.h -- an dynamic array
 * (C) 2011 - Georg Martius <georg -dot- maritus -at- web -dot- de>
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

#ifndef DSVECTOR_H
#define DSVECTOR_H

#include <stddef.h>


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
int ds_vector_size(DSVector *V);


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
void *ds_vector_get(DSVector *V, int pos);

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
