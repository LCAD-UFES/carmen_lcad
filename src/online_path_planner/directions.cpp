#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <curl/curl.h>

#define GOOGLE_API_KEY "<<INSERT_YOUR_GOOGLE_API_HERE>>"
#define URL "https://maps.googleapis.com/maps/api/directions/json?origin=%s&destination=%s&key=%s"
#define MAX_LENGTH_STRING 4096

char* swap_spaces(const char* string){
  int i;
  char* newString = (char *) calloc (strlen(string) + 1, sizeof(char));

  for(i = 0; string[i] != '\0'; i++){
    newString[i] = (string[i] == ' ') ? '+' : string[i];
  }

  return newString;
}

typedef struct string {
  char *ptr;
  size_t len;
}string;

void init_string (string *str){
  str->len = 0;
  str->ptr = (char *)malloc(str->len+1);
  if (str->ptr == NULL) {
    fprintf(stderr, "malloc() failed\n");
    exit(EXIT_FAILURE);
  }
  str->ptr[0] = '\0';
}

size_t write_response(void *ptr, size_t size, size_t nmemb, string *s){
  size_t new_len = s->len + size*nmemb;
  s->ptr = (char *)realloc(s->ptr, new_len+1);
  if (s->ptr == NULL) {
    fprintf(stderr, "realloc() failed\n");
    exit(EXIT_FAILURE);
  }
  memcpy(s->ptr+s->len, ptr, size*nmemb);
  s->ptr[new_len] = '\0';
  s->len = new_len;

  return size*nmemb;
}

char* get_directions(const char* origin, const char* destination, const char* key){
  CURL *curl;
  CURLcode res;
  string str;
  char url[4096];

  sprintf(url, URL, swap_spaces(origin), swap_spaces(destination), key);
  printf("#####\n%s#####\n", url);

  init_string(&str);
 
  curl_global_init(CURL_GLOBAL_DEFAULT);
 
  curl = curl_easy_init();
  if(curl) {
    curl_easy_setopt(curl, CURLOPT_URL, url);
    curl_easy_setopt(curl, CURLOPT_SSL_VERIFYPEER, 0L);
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, write_response);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &str);
 
    /* Perform the request, res will get the return code */ 
    res = curl_easy_perform(curl);
    
    if(CURLE_OK == res) {
      char *ct;
      /* ask for the content-type */ 
      res = curl_easy_getinfo(curl, CURLINFO_CONTENT_TYPE, &ct);
 
      if((CURLE_OK == res) && ct)
        printf("We received Content-Type: %s\n", ct);
    } else {
      exit(fprintf(stderr, "curl_easy_perform() failed: %s\n", curl_easy_strerror(res)));
    }

    /* always cleanup */ 
    curl_easy_cleanup(curl);
  }
 
  curl_global_cleanup();

  return str.ptr;
}

int main(void){
  char origem[MAX_LENGTH_STRING], destino[MAX_LENGTH_STRING];
  char *json;

  printf("Origem: ");
  scanf("%[^\n]", origem);
  
  printf("Destino: ");
  scanf("\n%[^\n]", destino);

  json = get_directions(origem, destino, GOOGLE_API_KEY);

  printf("%s", json);
 
  return 0;
}