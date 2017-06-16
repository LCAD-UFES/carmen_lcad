#include <curl/curl.h>
#include <curl/easy.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "download_map.h"

/**
 * notes on scale look http://msdn.microsoft.com/en-us/library/bb259689.aspx
 */
#define GOOGLE_API_ZOOM_LEVEL 19

size_t write_data_callback(void *ptr, size_t size, size_t nmemb, FILE *stream)
{
	int written = fwrite(ptr, size, nmemb, stream);
	return written;
}

void open_binary_output_file(FILE **fp, char *filename)
{
	*fp = fopen(filename,"wb");

	if ((*fp) == NULL) {
		exit(printf("Error: O arquivo \"%s\" nao pode ser aberto!\n", filename));
	}
}

void set_curl_options (CURL *curl, char *url, size_t callback_function(void*, size_t, size_t, FILE*), FILE* fp)
{
	// insere a url de onde o arquivo sera baixado
	curl_easy_setopt(curl, CURLOPT_URL, url);
	// funcao de callback para escrita dos dados
	curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, callback_function);
	// ponteiro para onde os dados serao escritos
	curl_easy_setopt(curl, CURLOPT_WRITEDATA, fp);
}

CURLcode perform_curl_download (CURL *curl)
{
	CURLcode res = CURLE_FAILED_INIT;

	// realiza a operacao
	res = curl_easy_perform(curl);
	// fecha o canal de comunicacao
	curl_easy_cleanup(curl);

	return res;
}

int download_file(char* url, char* filename)
{
	FILE *fp;
	CURL *curl;
	CURLcode download_status = CURLE_FAILED_INIT;

	if ((curl = curl_easy_init()) != NULL)
	{
		open_binary_output_file(&fp, filename);
		set_curl_options (curl, url, write_data_callback, fp);
		download_status = perform_curl_download(curl);

		fclose(fp);
	}

	if (download_status == CURLE_OK)
		return 1;
	else
		return 0;
}

void compose_google_api_url (char *url, double latitude, double longitude, char *maptype, int zoom, int width, int height)
{
	sprintf(url,
		"http://maps.googleapis.com/maps/api/staticmap?center=%f,%f&zoom=%d&size=%dx%d&sensor=false&maptype=%s",
		latitude,
		longitude,
		zoom,
		width,
		height,
		maptype
	);

//	strcpy (url, "http://maps.googleapis.com/maps/api/staticmap?center=-20.276811,-40.301296&zoom=19&size=512x512&sensor=false&maptype=satellite");
}

void download_map_from_google_api (char *url, char *filename)
{
//	int begin = 0, end = 0;

//	begin = time (NULL);

	if (!download_file(url, filename))
	{
		printf("Error: download do arquivo \"%s\" nao pode ser realizado de \"%s\"!\n", filename, url);
		exit(1);
	}

//	end = time (NULL);

/*	printf(
		"download time: %d segundos\n",
		(end - begin)
	);
*/
}

void get_image_from_gps(double latitude /* signed */, double longitude /* signed */, char* maptype /* satellite or roadmap */, int map_width, char* filename /* output */)
{
	if (!strcmp(maptype, "satellite") || !strcmp(maptype, "roadmap"))
	{
		char *url = (char *) malloc (1024 * sizeof(char));

		compose_google_api_url (url, latitude, longitude, maptype, GOOGLE_API_ZOOM_LEVEL, map_width, map_width);
		download_map_from_google_api (url, filename);

		free(url);
	}
	else
	{
		printf("Error: tipo de mapa nao reconhecido \"%s\"\n", maptype);
		exit(-1);
	}
}
