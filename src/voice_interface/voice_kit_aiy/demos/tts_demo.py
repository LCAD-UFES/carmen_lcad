import logging
import platform
import sys

import aiy.assistant.auth_helpers
from aiy.assistant.library import Assistant
import aiy.voicehat
from google.assistant.library.event import EventType
from google.cloud import texttospeech

import aiy.cloudspeech
import aiy.voicehat
import aiy.audio

import os
import re
import vlc


logging.basicConfig(
    level=logging.INFO,
    format="[%(asctime)s] %(levelname)s:%(name)s:%(message)s"
)

def main():
    if platform.machine() == 'armv6l':
        print('Cannot run hotword demo on Pi Zero!')
        exit(-1)

    credentials = aiy.assistant.auth_helpers.get_assistant_credentials()
    with Assistant(credentials) as assistant:
        for event in assistant.start():
            process_event(event)

def process_event(event):

	recognizer = aiy.cloudspeech.get_recognizer()
	status_ui = aiy.voicehat.get_status_ui()
	status_ui.status('starting')
	
#	recognizer.expect_hotword('Ola Iara')
#	recognizer.expect_phrase('Ola Iara')

	button = aiy.voicehat.get_button()
    
	if event.type == EventType.ON_START_FINISHED:
		status_ui.status('ready')
		if sys.stdout.isatty():
			print('Aperte o botão para começar')
			button.wait_for_press()
			words_said = input('Agora, escreva uma frase \n')
			status_ui.status('thinking')
			input_text = synthesize_text(words_said)

	elif event.type == EventType.ON_CONVERSATION_TURN_STARTED:
		status_ui.status('listening')

	elif event.type == EventType.ON_END_OF_UTTERANCE:
		status_ui.status('thinking')

	elif (event.type == EventType.ON_CONVERSATION_TURN_FINISHED
		or event.type == EventType.ON_CONVERSATION_TURN_TIMEOUT
		or event.type == EventType.ON_NO_RESPONSE):
		status_ui.status('ready')

	elif event.type == EventType.ON_ASSISTANT_ERROR and event.args and event.args['is_fatal']:
		sys.exit(1)


def synthesize_text(texto):
		"""Synthesizes speech from the input string of text."""
		
		client = texttospeech.TextToSpeechClient()
		input_text = texttospeech.types.SynthesisInput(text=texto)

		# Note: the voice can also be specified by name.
		# Names of voices can be retrieved with client.list_voices().
		voice = texttospeech.types.VoiceSelectionParams(
				language_code='pt-BR',
				ssml_gender=texttospeech.enums.SsmlVoiceGender.FEMALE)
		
		audio_config = texttospeech.types.AudioConfig(
				audio_encoding=texttospeech.enums.AudioEncoding.MP3)
				
		response = client.synthesize_speech(input_text, voice, audio_config)


		# The response's audio_content is binary.
		with open('output.mp3', 'wb') as out:
			out.write(response.audio_content)
			p = vlc.MediaPlayer("/home/pi/AIY-projects-python/src/tts/output.mp3")
			p.play()
			print('Conteúdo do áudio escrito em "output.mp3"')


if __name__ == '__main__':
	main()
