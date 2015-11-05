
#ifndef _VISUAL_GPS_H
#define _VISUAL_GPS_H

#include <curl/curl.h>
#include <stdio.h>

#ifdef __cplusplus
extern "C"
{
#endif

CURLcode perform_curl_download (CURL *curl);
int download_file(char* url, char* filename);
void open_binary_output_file(FILE **fp, char *filename);
void download_map_from_google_api (char *url, char* filename);
size_t write_data_callback(void *ptr, size_t size, size_t nmemb, FILE *stream);
void set_curl_options (CURL *curl, char *url, size_t callback_function(void*, size_t, size_t, FILE*), FILE* fp);
void compose_google_api_url (char *url, double latitude, double longitude, char *maptype, int zoom, int img_width, int img_height);
void get_image_from_gps(double latitude /* signed */, double longitude /* signed */, char* maptype /* satellite or roadmap */, int map_width /* map width */, char* filename /* output */);

#ifdef __cplusplus
}
#endif

#endif
