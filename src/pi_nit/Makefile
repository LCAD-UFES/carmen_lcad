include ../Makefile.conf

MODULE_NAME = PI NIT
MODULE_COMMENT = Deteccao de pessoas em Raspberry Pi 5 + Hailo-8L via ZMQ

LINK ?= g++
CXXFLAGS += -std=c++11

# A partir do Ubuntu 22.04 o pkg-config do OpenCV chama-se opencv4; o nome antigo so existe em
# distribuicoes velhas. Mesmo padrao ja usado em grid_mapping, road_mapper e pi_imu.
IFLAGS += `pkg-config --cflags opencv4 2>/dev/null || pkg-config --cflags opencv`

LFLAGS += -lparam_interface -lipc -lglobal \
	-lcamera_drivers -lcamera_drivers_process_image \
	`pkg-config --libs opencv4 2>/dev/null || pkg-config --libs opencv` \
	-lzmq -lpthread -lm

# neural_detector_{interface,messages} nao existem como modulo proprio nesta arvore: a
# mensagem de deteccao vem junto com o pi_nit, que e' quem a publica. Se um dia entrar um
# modulo dono dessa interface, e' so tirar os tres arquivos daqui e linkar contra ele.
SOURCES = pi_nit_client_driver.cpp pi_nit_interface.cpp pi_nit_zmq_client.cpp pi_nit_tracker.cpp pi_nit_link_test.cpp pi_nit_camera_publisher.cpp neural_detector_interface.c

PUBLIC_INCLUDES = pi_nit_messages.h pi_nit_interface.h pi_nit_protocol.h neural_detector_messages.h neural_detector_interface.h

PUBLIC_LIBRARIES = libpi_nit_interface.a libneural_detector_interface.a

PUBLIC_BINARIES = pi_nit_client_driver pi_nit_link_test pi_nit_camera_publisher

TARGETS = libpi_nit_interface.a libneural_detector_interface.a pi_nit_client_driver pi_nit_link_test pi_nit_camera_publisher

libpi_nit_interface.a: pi_nit_interface.o

libneural_detector_interface.a: neural_detector_interface.o

pi_nit_client_driver: pi_nit_client_driver.o pi_nit_interface.o pi_nit_zmq_client.o pi_nit_tracker.o neural_detector_interface.o

pi_nit_link_test: pi_nit_link_test.o pi_nit_zmq_client.o


pi_nit_camera_publisher: pi_nit_camera_publisher.o

include ../Makefile.rules
