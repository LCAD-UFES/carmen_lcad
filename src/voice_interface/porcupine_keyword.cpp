/*
 #
 # Project: Porcupine Alsa
 #
 # Licensed under the Apache License, Version 2.0 (the "License");
 # you may not use this file except in compliance with the License.
 # You may obtain a copy of the License at
 #
 #     http://www.apache.org/licenses/LICENSE-2.0
 #
 # Unless required by applicable law or agreed to in writing, software
 # distributed under the License is distributed on an "AS IS" BASIS,
 # WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 # See the License for the specific language governing permissions and
 # limitations under the License.
 */

#include <iostream>
#include <signal.h>
#include <alsa/asoundlib.h>
#include "pv_porcupine.h"

using namespace std;

// Change these to match your environment!
const char MODEL_PATH[] = "/data/voice_interface_data/porcupine_params.pv";
const char KEYWORD_PATH[] = "/data/voice_interface_data/hotword_ok_iara.ppn";
float sensitivity = 0.5f;

static pv_porcupine_t *porcupineObject;

const char snd_device[] = "default";

snd_pcm_t* capture_handle;
snd_pcm_hw_params_t* hw_params;
snd_pcm_info_t* s_info;
unsigned int srate = 16000;
unsigned int nchan = 1;


#define ERROR 1


int
init_soundcard()
{
	//connect to and set parameters for the sound device
	//returns any error setting or connecting

	int err = 0;

	if ((err = snd_pcm_open(&capture_handle, snd_device, SND_PCM_STREAM_CAPTURE, 0)) < 0)
	{
		cout << "cannot open audio device " << snd_device << " (" << snd_strerror(err) << ", " << err << ")" << endl;
		return (ERROR);
	}

	if ((err = snd_pcm_hw_params_malloc(&hw_params)) < 0)
	{
		cout << "cannot allocate hardware parameter structure (" << snd_strerror(err) << ", " << err << ")" << endl;
		return (ERROR);
	}

	if ((err = snd_pcm_hw_params_any(capture_handle, hw_params)) < 0)
	{
		cout << "cannot initialize hardware parameter structure (" << snd_strerror(err) << ", " << err << ")" << endl;
		return (ERROR);
	}

	if ((err = snd_pcm_hw_params_set_access(capture_handle, hw_params, SND_PCM_ACCESS_RW_INTERLEAVED)) < 0)
	{
		cout << "cannot set access type (" << snd_strerror(err) << ", " << err << ")" << endl;
		return (ERROR);
	}

	if ((err = snd_pcm_hw_params_set_format(capture_handle, hw_params, SND_PCM_FORMAT_S16_LE)) < 0)
	{
		cout << "cannot set sample format (" << snd_strerror(err) << ", " << err << ")" << endl;
		return (ERROR);
	}

	if ((err = snd_pcm_hw_params_set_rate_near(capture_handle, hw_params, &srate, 0)) < 0)
	{
		cout << "cannot set sample rate (" << snd_strerror(err) << ", " << err << ")" << endl;
		return (ERROR);
	}

	if ((err = snd_pcm_hw_params_set_channels(capture_handle, hw_params, nchan)) < 0)
	{
		cout << "cannot set channel count (" << snd_strerror(err) << ", " << err << ")" << endl;
		return (ERROR);
	}

	if ((err = snd_pcm_hw_params(capture_handle, hw_params)) < 0)
	{
		cout << "cannot set parameters (" << snd_strerror(err) << ", " << err << ")" << endl;
		return (ERROR);
	}

	if ((err = snd_pcm_prepare(capture_handle)) < 0)
	{
		cout << "cannot prepare audio interface for use (" << snd_strerror(err) << ", " << err << ")" << endl;
		return (ERROR);
	}

	if ((err = snd_pcm_start(capture_handle)) < 0)
	{
		cout << "cannot start soundcard (" << snd_strerror(err) << ", " << err << ")" << endl;
		return (ERROR);
	}

	snd_pcm_hw_params_free(hw_params);

	return (0);
}


int
start_porcupine()
{
	char model_path[1024];
	char keyword_path[1024];

	//retrieve the base path to Carmen home directory
	char *home_dir = getenv("CARMEN_HOME");
	if (home_dir == NULL)
	{
		printf("CARMEN_HOME not defined in start_porcupine()\n");
		return (ERROR);
	}

	//build model and keyword absolute paths	
	strcpy(model_path, home_dir);
	strcpy(keyword_path, home_dir);
	strcat(model_path, MODEL_PATH);
	strcat(keyword_path, KEYWORD_PATH);
	printf("*** %s *** \n", model_path);
	printf("*** %s *** \n", keyword_path);

	//initialize Porcupine wake word engine using the API version 1.9
	int32_t num_keywords = 1;
	const char *keyword_path_1 = (const char *) &keyword_path[0];
	const char *const *keyword_paths = &keyword_path_1;
	const float *sensitivities = (const float *) &sensitivity;

	pv_status_t status = pv_porcupine_init(model_path, num_keywords, keyword_paths, sensitivities, &porcupineObject);
	if (status == PV_STATUS_SUCCESS)
	{
		cout << "Porcupine initialized" << endl;
	}
	else
	{
		cout << "Failed to initialize Porcupine" << endl;
		return (ERROR);
	}

	return (0);
}


//int
//hotword_detection_old()
//{
//	//listen on the mic for the wake word
//
//	int err = 0;
//	const int buffer_size = pv_porcupine_frame_length();
//	short wav_data[buffer_size];
//	bool detected;
//
//	signal(SIGINT, catchcancel);
//
//	while (!stop)
//	{
//		err = snd_pcm_readi(capture_handle, wav_data, buffer_size)
//				!= buffer_size;
//		if (err < 0)
//		{
//			cout << "read from audio interface failed (" << snd_strerror(err)
//					<< ", " << err << ")" << endl;
//			if (err == -32) // Broken pipe
//			{
//				if ((err = snd_pcm_prepare(capture_handle)) < 0)
//				{
//					cout << "cannot prepare audio interface for use ("
//							<< snd_strerror(err) << ", " << err << ")" << endl;
//					return (ERROR);
//				}
//			}
//			else
//			{
//				return (ERROR);
//			}
//		}
//		else
//		{
//			pv_porcupine_process(porcupineObject, wav_data, &detected);
//
//			if (detected)
//			{
//				// Detected keyword. Do something!
//				cout << "Detected keyword!" << endl;
//				break;
//			}
//		}
//	}
//	return 0;
//}

/**
 * Reads an audio frame of the incoming audio stream and processes it to detect the hotword.
 *
 * @return Returns 1 if the hotword is detected, 0 if not and 2 on failure.
 */
int
hotword_detection()
{
	int ret = 0; // OK

	int32_t buffer_size = pv_porcupine_frame_length();
	int16_t *wav_data = (int16_t *) malloc(buffer_size * sizeof(int16_t));

	try
	{
		//read data from PCM audio interface 
		snd_pcm_sframes_t err = snd_pcm_readi(capture_handle, (void *) wav_data, buffer_size) != buffer_size;

		if (err < 0)
		{
			cout << "read from audio interface failed (" << snd_strerror(err) << ", " << err << ")" << endl;
			if (err == -32) // Broken pipe
			{
				//restore PCM audio interface for use	
				if ((err = snd_pcm_prepare(capture_handle)) < 0)
				{
					cout << "cannot prepare audio interface for use (" << snd_strerror(err) << ", " << err << ")" << endl;
					ret = 2; // Error
				}
			}
			else
			{
				ret = 2; // Error
			}
		}
		else
		{
			int32_t keyword_index = -1;

			//process data from PCM audio interface	
			pv_status_t status = pv_porcupine_process(porcupineObject, wav_data, &keyword_index);

			if (status == PV_STATUS_SUCCESS)
			{
				ret = (keyword_index >= 0) ? 1 : 0;				
			}
			else
			{
				cout << "cannot process audio data (" << pv_status_to_string(status) << ", " << status << ")" << endl;
				ret = 2; // Error
			}	
		}
	}
	catch(const std::exception& e)
	{
		cout <<  __func__ << ">> exception raised: " << e.what() << endl;
		ret = 2; // Error
	}
	catch(...) 
	{
		cout <<  __func__ << ">> unknown exception raised." << endl;
		ret = 2; // Error
	}

	free(wav_data);

	return ret;
}


int
finalize_porcupine()
{
	//disconnect from porcupine
	pv_porcupine_delete(porcupineObject);

	//disconnect from soundcard
	return snd_pcm_close(capture_handle);
}


char *
initialize_porcupine()
{
	cout << "Open default soundcard" << endl;
	if (init_soundcard() == ERROR)
		return ((char *) "Cound not init soundcard in initialize_porcupine()");

	cout << "Init Porcupine" << endl;
	if (start_porcupine() == ERROR)
		return ((char *) "Cound not start porcupine");

	if (porcupineObject == NULL)
		return ((char *) "porcupineObject == NULL");

	return (NULL); // Ok
}
