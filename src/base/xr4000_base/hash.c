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

#include <carmen/carmen.h>
#include <assert.h>
#include "hash.h"

struct cache *hash_new(unsigned int size, unsigned int (*hash_func)(), 
		       int (*compare_func)())
{
  struct cache *cache;
  assert(size);
  assert(!(size & (size - 1)));

  cache = (struct cache *) calloc(1, sizeof(struct cache));
  carmen_test_alloc(cache);

  cache->node_table = (struct cache_node **)
    calloc(size, sizeof(struct cache_node*));
  carmen_test_alloc(cache->node_table);

  cache->size = size;
  cache->mask = size - 1;
  cache->hash_func = hash_func;
  cache->compare_func = compare_func;
  
  return cache;
}


void hash_delete(struct cache *cache) 
{
  struct cache_node *node;
  
  while ((node = hash_next(cache, NULL)) != NULL) {
    hash_remove(cache, node->key);
  }
  free(cache->node_table);
  
  free(cache);
  
}

void hash_add(struct cache **cachep, void *key, void *value) 
{
  unsigned int indx; 
  struct cache_node *node; 
  struct cache_node *node1; 
  struct cache *new; 
#if 0
  struct cache *crap1;
  struct cache **crap2;
#endif

  indx = (*cachep)->hash_func(*cachep, key);
  node = (struct cache_node *) calloc(1, sizeof(struct cache_node));
  carmen_test_alloc(node);

  assert(node);
  node->key = key;
  node->value = value;
  node->next = (*cachep)->node_table[indx];
  (*cachep)->node_table[indx] = node;
  (*cachep)->used++;
  if ((((*cachep)->size * 75) / 100) <= (*cachep)->used) {
    node1 = NULL;
    new = hash_new((*cachep)->size * 2, (*cachep)->hash_func, 
		   (*cachep)->compare_func);
    while ((node1 = hash_next(*cachep, node1)) != NULL) {
      hash_add(&new, node1->key, node1->value);
    }
    hash_delete(*cachep);
    
    *cachep = new;
  }
  
}

void hash_remove(struct cache *cache, void *key) 
{
  unsigned int indx;
  struct cache_node *node;
  indx = cache->hash_func(cache, key);
  node = cache->node_table[indx];
  
  assert(node);

  if (cache->compare_func(node->key, key)) {
    cache->node_table[indx] = node->next;
    free(node);
  } else {

    struct cache_node *prev = node;
    char removed = 0;
    do {
      if (cache->compare_func(node->key, key)) {
	prev->next = node->next;
	removed = 1;
	free(node);
      } else {
	prev = node;
	node = node->next;
      }
    } while (!removed && (node != NULL));
    assert(removed);
  }
  cache->used--;
}

struct cache_node *hash_next(struct cache *cache, struct cache_node *node) 
{
  if (node == NULL) {
    cache->last_bucket = 0;
  }
  
  if (node != NULL) {
    if (node->next != NULL) {
      return node->next;
    } else {
      cache->last_bucket++;
      
    }
  }
  
  if (cache->last_bucket < cache->size) {
    while (cache->last_bucket < cache->size) {
      if (cache->node_table[cache->last_bucket] != NULL) {
	return cache->node_table[cache->last_bucket];
      } else {
	cache->last_bucket++;
      }
    }

    return 0;    
  } else
    return 0;
  
}

void *hash_value_for_key(struct cache *cache, void *key)
{
  struct cache_node *node; 
  void *retval;
  node = cache->node_table[cache->hash_func(cache, key)];
  retval = NULL;
  if (node != NULL) {
    do {
      
      if (cache->compare_func(node->key, key)) {
	retval = node->value;
      } else {
	node = node->next;
      }
      
      
    } while ((retval == NULL) && (node));
    
    
    
  } 
  
  return retval;
    
}



void *hash_change_key_value(struct cache *cache, void *key, void *new_value)
{
  struct cache_node *node;
  void *retval;
  node = cache->node_table[cache->hash_func(cache, key)];
  retval = NULL;
  if (node != NULL) {
    do {
      if (cache->compare_func(node->key, key)) {
	retval = node->value = new_value;
      } else 
	node = node->next;
    } while ((retval == NULL) && (node != NULL));
    

  }
  
  return retval;
}
