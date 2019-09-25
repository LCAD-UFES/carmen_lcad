 /*********************************************************
 *
 * This source code is part of the Carnegie Mellon Robot
 * Navigation Toolkit (CARMEN)
 *
 * CARMEN Copyright (c) 2002 Michael Montemerlo, Nicholas
 * Roy, Sebastian Thrun, Dirk Haehnel, Cyrill Stachniss,
 * and Jared Glover
 *
 * CARMEN is free software; you can redistribute it and/or 
 * modify it under the terms of the GNU General Public 
 * License as published by the Free Software Foundation; 
 * either version 2 of the License, or (at your option)
 * any later version.
 *
 * CARMEN is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied 
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE.  See the GNU General Public License for more 
 * details.
 *
 * You should have received a copy of the GNU General 
 * Public License along with CARMEN; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place, 
 * Suite 330, Boston, MA  02111-1307 USA
 *
 ********************************************************/

#ifndef __INC_hash_h
#define __INC_hash_h

#ifdef __cplusplus
extern "C" {
#endif

struct cache_node {
  struct cache_node *next;
  void *key;
  void *value;
};

typedef struct cache_node *node_ptr;

typedef struct cache {
  struct cache_node **node_table;
  unsigned int size;
  unsigned int used;
  unsigned int mask;
  unsigned int last_bucket;
  unsigned int (*hash_func)();
  int (*compare_func)();
} *cache_ptr;

typedef unsigned int (*hash_func_type)(); 
typedef int (*compare_func_type)(); 

void hash_add(struct cache **, void *, void *);
void *hash_change_key_value(struct cache *, void *, void *);
void hash_delete(struct cache *);
struct cache *hash_new(unsigned int, unsigned int (*)(), int (*)());
struct cache_node *hash_next(struct cache *, struct cache_node *);
void hash_remove(struct cache *, void *);
void *hash_value_for_key(struct cache *, void *);

#ifdef __cplusplus
}
#endif

#endif /* __INC_hash_h */
