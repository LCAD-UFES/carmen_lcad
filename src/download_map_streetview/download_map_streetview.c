#include <curl/curl.h>
#include <curl/easy.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "download_map_streetview.h"

/**
 * notes on scale look http://msdn.microsoft.com/en-us/library/bb259689.aspx
 */

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

void compose_google_api_url_streetview (char *url, double latitude, double longitude, int width /*640*/, int height/*480*/,char *key, int angle)
{
	sprintf(url,
			"http://maps.googleapis.com/maps/api/streetview?size=%dx%d&location=%f,%f&heading=%d&sensor=false&key=%s",
			width,
			height,
			latitude,
			longitude,
			angle,
			key
	);

	/*printf(url,
				"http://maps.googleapis.com/maps/api/streetview?size=%dx%d&location=%f,%f&heading=%d&sensor=false&key=%s",
				width,
				height,
				latitude,
				longitude,
				angle,
				key
		);

	*/
	//	strcpy (url, "http://maps.googleapis.com/maps/api/streetview?size=640x480&location=46.414382,10.013988&heading=135&pitch=0&sensor=false");
}

void download_map_streetview_from_google_api (char *url, char *filename)
{
	int begin = 0, end = 0;

	begin = time (NULL);

	if (!download_file(url, filename))
	{
		printf("Error: download do arquivo \"%s\" nao pode ser realizado de \"%s\"!\n", filename, url);
		exit(-1);
	}

	end = time (NULL);

	printf(
			" download time: %d segundos\n",
			(end - begin)
	);
}

void get_image_from_gps(double latitude /* signed */, double longitude /* signed */, int map_width, int  map_height, char* filename /* output */, char *key, int angle)
{
	char *url = (char *) malloc (1024 * sizeof(char));

	compose_google_api_url_streetview (url, latitude, longitude, map_width, map_height,key, angle);
	download_map_streetview_from_google_api (url, filename);

	free(url);

}
