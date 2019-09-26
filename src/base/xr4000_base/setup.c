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

static struct cache *Table; 
static void Cleanup(char *String);
static unsigned int hash_string(register struct cache *cache, void *key);
static int compare_strings(register void *k1, register void *k2);

unsigned char SETUP_ReadSetup(char *filename) 
{
  char *buf; 
  char buf1[200], buf2[200], *SectionName;
  FILE *fp; 
  char *Token; 
  char *Key; 
  char *Value; 
  char *Aux; 
  long int Length; 
  long int KeyLength; 
  long int SectionNameLength =0; 
  long int TokenLength; 
  long int l; 

  fp = fopen(filename, "r");

  SectionName = calloc(200, 1);
  carmen_test_alloc(SectionName);

  if (fp == NULL) {
    printf("Unable to read setup file %s\n", filename);
    return 0;
  } 
  if (Table == NULL) {
    Table = hash_new(0x200, hash_string, compare_strings);
    
    assert(Table);
  }
  
  while (fgets(buf1, 199, fp) != 0) {
    buf = buf1;
    buf += strspn(buf, " \n\t");

    if (buf[0] == *";") {
      continue;
    } else {
      if (strchr(buf, *"[") != NULL) {
	Token = strtok(buf, "=\\[]=;\t\n");
	if (Token == NULL) {
	  printf("Incorrect section name format for section %s\n",
		 buf);
	  SectionName = NULL;
	} else { 
	  Cleanup(Token);
	  Length = strlen(Token);
	  SectionName[0] = *"[";
	  memcpy(SectionName + 1, Token, Length + 1);
	  *(SectionName + Length + 1) = *"]";
	  *(SectionName + Length + 2) = '\0';
	  SectionNameLength = Length + 2;
	} 
      } else if (strchr(buf, *"=") != NULL) {
	Aux = strstr(buf, "=") + 1;
	Aux += strspn(Aux, " \n\t");
	Length = strcspn(Aux, ";");
	Value = (char *)calloc(Length + 1, 1);
	carmen_test_alloc(Value);
	memcpy(Value, Aux, Length);
	Value[Length] = '\0';
	while (strchr(Value, *"\\") != NULL) {
	  Length = strcspn(Value, "\\");
	  if (fgets(buf2, 199, fp) != 0) {
	    l = strlen(buf2);
	    Value = realloc(Value, Length + l);
	    carmen_test_alloc(Value);
	    memcpy(Value + Length, buf2, l);
	    Length += l;
	  } 
	  Value[Length] = '\0';
	} 
	Cleanup(Value);
	Token = strtok(buf, "=\\[]=;\t\n");
	Cleanup(Token);

	TokenLength = strlen(Token);
	Key = (char *)calloc(SectionNameLength + TokenLength + 1, 1);
	carmen_test_alloc(Key);
	memcpy(Key, SectionName, SectionNameLength);
	KeyLength = SectionNameLength;
	memcpy(Key + KeyLength, Token, strlen(Token));
	KeyLength += strlen(Token);
	Key[KeyLength] = '\0';
	if (hash_value_for_key(Table, Key) != NULL) {
	  hash_change_key_value(Table, Key, Value);
	  free(Key);
	} else { 
	  hash_add(&Table, Key, Value);
	}
      } 
    }
  }
  
  free(SectionName);
  fclose(fp);
  return 1;
}


char *SETUP_GetValue(char *Key) 
{
  if (Table == NULL) 
    return NULL;
  
  return hash_value_for_key(Table, Key);
}

char *SETUP_ExtGetValue(char *Section, char *Key)
{
  char *tmp, *ret; 

  tmp = calloc(strlen(Section) + strlen(Key) + 3, 1);
  carmen_test_alloc(tmp);
  sprintf(tmp, "[%s]%s", Section, Key);
  ret = SETUP_GetValue(tmp);
  free(tmp);
  return ret;
}

void SETUP_SetValue(char *Key, char *UserValue) 
{
  char *Value; 
  char *OldValue; 
  char *NewKey; 

  assert(Key);
  assert(UserValue);

  Value = calloc(strlen(UserValue) + 1, 1);
  carmen_test_alloc(Value);
  if (Table == NULL) {
    Table = hash_new(0x200, hash_string, compare_strings);
    
    assert(Table);
  }
  strcpy(Value, UserValue);
  
  if ((OldValue = hash_value_for_key(Table, Key)) != NULL) {
    hash_change_key_value(Table, Key, Value);
    free(OldValue);
  } else { 
    NewKey = calloc(strlen(Key) + 1, 1);
    carmen_test_alloc(NewKey);
    strcpy(NewKey, Key);
    hash_add(&Table, NewKey, Value);
  }
}

static void Cleanup(char *String) 
{
  long int i; 
  long int j; 
  j = 0;
  i = 0;
  
  assert(String);
  
  while (String[i] != '\0') {
    if (strchr("=\\[]=;\t\n", String[i]) == NULL) {
      String[j] = String[i];
      i++;
      j++;
    } else { 
      i++;
    }
  }
  String[j] = '\0';
  j--;
  
  while (strchr(" \n\t", String[j]) != NULL) {
    String[j] = '\0';
    j--;
  }
  
  return;
}

long int SETUP_ItemCount(char *StemKey) 
{
  char *ItemKey; 
  long int ItemCount; 
 
  ItemCount = 0;
  ItemKey = (char *)calloc(strlen(StemKey) + 0x226, 1);
  carmen_test_alloc(ItemKey);
  
  sprintf(ItemKey, "%s%ld", StemKey, ItemCount);
  while (SETUP_GetValue(ItemKey) != NULL) {
    ItemCount++;
    sprintf(ItemKey, "%s%ld", StemKey, ItemCount);
  }
  return ItemCount;
}

unsigned int hash_string(register struct cache *cache, void *key)
{
  unsigned int ret; 
  unsigned int ctr; 
  char *ckey; 

  ret = 0;
  ctr = 0;
  ckey = (char *) key;
  while (*ckey != 0) {
    ret ^= (*ckey) << ctr;
    ckey++;
    ctr = (ctr + 1) & 0x03;
  }
  return ret & cache->mask;
}

int compare_strings(register void *k1, register void *k2) 
{
  if (k1 == k2)
    return 1;
  else if ((k1 == NULL) || (k2 == NULL))
    return 0;
  else {
    return (strcmp((char *) k1, (char *) k2)) ? 0 : 1;
  }
  
}
