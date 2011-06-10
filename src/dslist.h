/*
 * dslist.h -- a double linked list
 * taken from tclist.h -- a list for transcode / interface
 * (C) 2008-2010 - Francesco Romani <fromani -at- gmail -dot- com>
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

#ifndef DSLIST_H
#define DSLIST_H

#include <stddef.h>

typedef struct dslistitem_ DSListItem;
struct dslistitem_ {
  void        *data;
  DSListItem  *next;
  DSListItem  *prev;
};

typedef struct dslist_ DSList;
struct dslist_ {
  DSListItem  *head;
  DSListItem  *tail;
  DSListItem  *cache;
  int         use_cache;
  int         nelems;
};

/*
 * LIST INDEXING NOTE:
 * -----------------------------------------------------------------------
 * WRITEME
 */

/*
 * DSListVisitor:
 *     typedef for visitor function.
 *     A visitor function is called on elements of a given list (usually
 *     on each element) giving to it pointers to current element and custom
 *     user data. It is safe to modify the element data inside the visitor
 *     function, even free()ing it. But DO NOT modify the list *item*
 *     inside this function. Use the special crafted upper-level list
 *     functions for that.
 *
 * Parameters:
 *         item: pointer to the list item currently visited.
 *     userdata: pointer to custom, opaque caller-given data.
 * Return Value:
 *      0: success. Iteration continues to the next element, if any.
 *     !0: failure. Iteration stops here.
 */
typedef int (*DSListVisitor)(DSListItem *item, void *userdata);

/*
 * ds_list_init:
 *     intializes a list data structure.
 *     A list can use a deleted element cache. The deleted list elements
 *     (NOT the data pointed by them!) are not really released until the
 *     ds_list_fini() is called, but they are just saved (in the `cache'
 *     field.
 *     Use the cache feature if ALL the following conditions are met:
 *     - you will do a lot of insertion/removals.
 *     - you will stabilize the list size around a given size.
 *     Otherwise using the cache will not hurt, but it will neither help.
 *
 * Parameters:
 *             L: pointer to list to be initialized.
 *     elemcache: treated as boolean, enables or disable the internal cache.
 * Return Value:
 *     DS_OK on success,
 *     DS_ERROR on error.
 */
int ds_list_init(DSList *L, int elemcache);

/*
 * ds_list_fini:
 *     finalizes a list data structure. Frees all resources aquired,
 *     including any cached list element (if any), but *NOT* the data
 *     pointed by list elements.
 *
 * Parameters:
 *     L: pointer to list to be finalized
 * Return Value:
 *     DS_OK on success,
 *     DS_ERROR on error.
 */
int ds_list_fini(DSList *L);

/*
 * ds_list_size:
 *     gives the number of elements present in the list.
 *
 * Parameters:
 *     L: list to be used.
 * Return Value:
 *    -1 on error,
 *    the number of elements otherwise
 */
int ds_list_size(DSList *L);

/*
 * ds_list_foreach:
 *     applies a visitor function to all elements in the given lists,
 *     halting at first visit failed.
 *
 * Parameters:
 *             L: pointer to list to be visited.
 *           vis: visitor function to be applied.
 *      userdata: pointer to opaque data to be passed unchanged to visitor
 *                function at each call.
 * Return Value:
 *     0: if all elements are visited correctly.
 *    !0: the value returned by the first failed call to visitor function.
 */
int ds_list_foreach(DSList *L, DSListVisitor vis, void *userdata);

/*
 * ds_list_{append,prepend}:
 *     append or prepend an element to the list.
 *     The element is added on the {last,first} position of the list.
 *
 * Parameters:
 *        L: pointer to list to be used
 *     data: pointer to data to be appended or prepend.
 *           *PLEASE NOTE* that JUST THE POINTER is copied on the newly-added
 *           element. NO deep copy is performed.
 *           The caller has to allocate memory by itself if it want to
 *           add a copy of the data.
 * Return Value:
 *     DS_OK on success,
 *     DS_ERROR on error.
 */
int ds_list_append(DSList *L, void *data);
int ds_list_prepend(DSList *L, void *data);

/*
 * ds_list_insert:
 *      the newly-inserted elements BECOMES the position `pos' on the list.
 *      Position after the last -> the last.
 *      Position before the first -> the first.
 */
int ds_list_insert(DSList *L, int pos, void *data);

/*
 * ds_list_get:
 *     gives access to the data pointed by the element in the given position.
 *
 * Parameters:
 *       L: list to be accessed.
 *     pos: position of the element on which the data will be returned.
 * Return Value:
 *     NULL on error (requested element doesn't exist)
 *     a pointer to the data belonging to the requested list item.
 */
void *ds_list_get(DSList *L, int pos);

/*
 * ds_list_pop:
 *     removes the element in the given position.
 *
 * Parameters:
 *       L: list to be accessed.
 *     pos: position of the element on which the data will be returned.
 * Return Value:
 *     NULL on error (requested element doesn't exist)
 *     a pointer to the data assigned to the requested list item.
 */
void *ds_list_pop(DSList *L, int pos);

/*************************************************************************/

DSList *ds_list_new(int usecache);
void ds_list_del(DSList *L, int deepclean);

int ds_list_insert_dup(DSList *L, int pos, void *data, size_t size);
#define ds_list_append_dup(L, DATA, SIZE) ds_list_insert_dup(L, -1, DATA, SIZE)
#define ds_list_prepend_dup(L, DATA, SIZE) ds_list_insert_dup(L, 0,  DATA, SIZE)

#endif /* DSLIST_H */

/*
 * Local variables:
 *   c-file-style: "stroustrup"
 *   c-file-offsets: ((case-label . *) (statement-case-intro . *))
 *   indent-tabs-mode: nil
 * End:
 *
 * vim: expandtab shiftwidth=4:
 */
