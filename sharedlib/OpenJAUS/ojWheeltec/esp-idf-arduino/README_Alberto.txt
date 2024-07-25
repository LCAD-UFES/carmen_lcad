# Para compilar uma aplicação Arduino no Espressif FreeRTOS (https://www.youtube.com/watch?v=XgAIaVV39tk)


## Preparacao do ambiente

Crie o diretorio base esp-idf-arduino (provavelmente já criado, já que é onde está este README_Alberto.txt) e instale nele versões 
apropriadas (compatíveis entre si) do esp-idf e do componente Arduino do esp-idf.

 mkdir esp-idf-arduino (leia as duas linhas acima)
 cd esp-idf-arduino (leia as três linhas acima)
 git clone https://github.com/espressif/esp-idf.git
 cd esp-idf
 git checkout 6a7d83af1984b93ebaefa7ca9be36304806c3dc8
 cd components
 git clone https://github.com/espressif/arduino-esp32.git arduino
 cd arduino
 git checkout 16a9cf781fafffedd70b794beed24853965d78ce
 git submodule update --init --recursive
 cd ../..
 git submodule update --init --recursive
./install.sh
 patch components/arduino/cores/esp32/esp32-hal-uart.c < ../esp32_idf.patch
 . ./export.sh


## Crie um diretorio para a aplicacao. 

Neste exemplo, vamos criar o diretório para a aplicação WIFI_LoRa_32FactoryTest.
Ela é parte (um exemplo) do pacote Heltec_ESP32 (https://github.com/HelTecAutomation/Heltec_ESP32.git).
(Você pode usar a aplicação WIFI_LoRa_32FactoryTest como template no lugar de esp-idf-arduino-template, abaixo)

 mkdir WIFI_LoRa_32FactoryTest
 cd WIFI_LoRa_32FactoryTest
 cp -r ../esp-idf-arduino-template/* .


Note que sua aplicacao original for FreeRTOS e voce quiser juntar ela com uma aplicacao Arduino, ela tem que ser .cpp . 
Se for .c, modifique para .cpp e mude sua "void app_main()" para  "extern "C" void app_main()".
Inclua seu codigo Arduino na CMakeLists.txt no idf_component_register()
Nao deixe de incluir #include "Arduino.h" em todos os fontes.


## Baixe o git de sua aplicação Arduino e as bibliotecas específicas que usa para o diretório main.

 cd main
 git clone https://github.com/HelTecAutomation/Heltec_ESP32.git
 git clone https://github.com/Heltec-Aaron-Lee/WiFi_Kit_series.git


## Altere o diretório, ajustando a aplicação para o Eclipse.

Copie o conteúdo do arquivo .ino da aplicação original (neste exemplo, WiFi_LoRa_32FactoryTest) e seus .h (images.h) para o diretorio main

 cp Heltec_ESP32/examples/Factory_Test/WiFi_LoRa_32FactoryTest/WiFi_LoRa_32FactoryTest.ino .
 cp Heltec_ESP32/examples/Factory_Test/WiFi_LoRa_32FactoryTest/images.h .

Altere o nome do arquivo .ino para .cpp

 mv WiFi_LoRa_32FactoryTest.ino WiFi_LoRa_32FactoryTest.cpp

Altere o CMakeLists.txt do diretorio do módulo para incluir os diretórios onde os includes devem ser buscados e coloque o nome de sua aplicação: 
 include_directories("main/Heltec_ESP32/src" "main/Heltec_ESP32/src/lora" "main/Heltec_ESP32/src/oled" "main/WiFi_Kit_series/esp32/variants/wifi_lora_32_V2")
 project(bl_local_interface)

Altere o CMakeLists.txt do diretorio main, main/CMakeLists.txt, para incluir os fontes: 
 
 list(APPEND COMPONENT_SRCS "WiFi_LoRa_32FactoryTest.cpp")

e os arquivos das bibliotecas a serem compilados:

 list(APPEND COMPONENT_SRCS 
             "Heltec_ESP32/src/BMP180.cpp"
             "Heltec_ESP32/src/heltec.cpp"
             "Heltec_ESP32/src/lora/LoRa.cpp"
             "Heltec_ESP32/src/oled/OLEDDisplay.cpp"
             "Heltec_ESP32/src/oled/OLEDDisplayUi.cpp"
             )
 
Indique, também, no main/CMakeLists.txt onde os includes devem ser buscados:

 list(APPEND COMPONENT_INCLUDES "Heltec_ESP32/src" "Heltec_ESP32/src/lora" "Heltec_ESP32/src/oled")

Altere o arquivo include.xml para apontar para os locais dos includes de sua aplicação. Este arquivo é usado pelo Eclipse para achar os includes, 
como mostrado abaixo.


## Crie (adicione) a aplicação no Eclipse

Execute o Eclipse no diretório principal da aplicação:

 . ../esp-idf/export.sh
 /opt/eclipse/eclipse &

Se você estiver trabalhando com a aplicação Arduino fora do astro, no Eclipse 
navegue File->New->Makefile Project with Existing Code e escolha o diretório WIFI_LoRa_32FactoryTest (use as opções default)
Se a aplicação estiver dentro do astro este passo não é necessário.

Abra o arquivo WiFi_LoRa_32FactoryTest.cpp no Eclipse e, depois, Project->Properties->C/C++ General->Paths and Symbols->Import Settings (este último é um botão). 
Escolha o xml WIFI_LoRa_32FactoryTest/include.xml

Clique com o botão da direita no nome do projeto no Eclipse (WIFI_LoRa_32FactoryTest) e escolha Index->Rebuid

Pode ser necessário reordenar as funções de seu arquivo WiFi_LoRa_32FactoryTest.cpp, já que a IDE do Arduino não liga de declarar uma função entes de usá-la,
mas o compilador do esp-idf liga. Além disso, pode ser necessário incluir #defines para o Eclipse mostrar corretamente os arquivos ou para eles compilarem.
Ver como compilar abaixo.


## Compile sua aplicação

No diretório principal dela (acima do diretório main), execute o comando:
 
 . ../esp-idf/export.sh
 idf.py build

Para flash:

 idf.py -p /dev/ttyUSB0 flash

Para monitor:

 idf.py -p /dev/ttyUSB0 monitor


## Mudando uma aplicação Arduino para ser completamente FreeRTOS (https://docs.espressif.com/projects/arduino-esp32/en/latest/esp-idf_component.html)

Copie a sua aplicação Arduino para um novo diretório:
 cp -r WIFI_LoRa_32FactoryTest bl_remote_interface
 cd bl_remote_interface
 rm -rf build

Execute o Eclipse no diretório principal da aplicação:

 . ../esp-idf/export.sh
 /opt/eclipse/eclipse &

Se você estiver trabalhando com a aplicação Arduino fora do astro, no Eclipse 
navegue File->New->Makefile Project with Existing Code e escolha o diretório WIFI_LoRa_32FactoryTest (use as opções default)
Se a aplicação estiver dentro do astro este passo não é necessário.

Mude o nome do arquivo WiFi_LoRa_32FactoryTest.cpp para um nome apropriado ao seu projeto:
 
 cd main
 mv WiFi_LoRa_32FactoryTest.cpp bl_remote_main.cpp
 
e abra o novo arquico no Eclipse.

Altere o CMakeLists.txt no diretório bl_remote_interface e no diretório bl_remote_interface/main para refletir o novo nome
da aplicação e arquivo main*.cpp

Abra e altere o arquivo xml bl_remote_interface/include.xml para refletir a mudança de nome do diretório da Aplicação.

Vá em Project->Properties->C/C++ General->Paths and Symbols->Import Settings (este último é um botão). 
Escolha o xml bl_remote_interface/include.xml

Clique com o botão da direita no nome do projeto no Eclipse (bl_remote_interface) e escolha Index->Rebuid (ou IIndex->Freshen All Files)

Inclua no fim do arquivo WiFi_LoRa_32FactoryTest.cpp a função app_main() como abaixo:

 extern "C" void app_main()
 {
	initArduino();

	setup();
	
	while (1)
	{
		loop();

		// Release the core for 1ms (necesary to avoid wdt (watchdog timer) complains).
		const TickType_t xDelay = 1 / portTICK_PERIOD_MS;
		vTaskDelay(xDelay);
	}
 }


Execute no diretório bl_remote_interface: 
 . ../esp-idf/export.sh
 idf.py menuconfig

Em Arduino Configuration  --->
desmarque a opção abaixo:

 [ ] Autostart Arduino setup and loop on boot

Saia e salve.

Depois, compile e de flash:

 idf.py build
 idf.py -p /dev/ttyUSB0 flash

Sua aplicação deve funcionar como antes. Contudo, agora você pode adicionar código padrão FreeRTOS :)


