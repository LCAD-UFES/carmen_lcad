TARGET	= 	./lib/libopenJaus.so
CCFLAGS	=	-Wall -fPIC -fno-strict-aliasing -O2 -c -g -I ./include/ -I ../libjaus/include/
LFLAGS	=	-L ../libjaus/lib/
LIBS		=	-ljaus
SOURCES =	./src/componentLibrary/ojCmpt.c \
		./src/utils/multicastSocket.c \
		./src/utils/queue.c \
		./src/utils/inetAddress.c \
		./src/utils/timeLib.c \
		./src/utils/datagramPacket.c \
		./src/utils/datagramSocket.c \
		./src/nodeManagerInterface/nodeManagerInterface.c \
		./src/nodeManagerInterface/lmHandler.c \
		./src/nodeManagerInterface/defaultJausMessageProcessor.c \
		./src/nodeManagerInterface/scManager.c \
		./src/utils/FileLoader.cpp \
		./src/nodeManager/LocalComponent.cpp \
		./src/nodeManager/JausTransportInterface.cpp \
		./src/nodeManager/JausTransportQueue.cpp \
		./src/nodeManager/JausCommunicationManager.cpp \
		./src/nodeManager/JausComponentCommunicationManager.cpp \
		./src/nodeManager/NodeManager.cpp \
		./src/nodeManager/NodeManagerComponent.cpp \
		./src/nodeManager/SystemTree.cpp \
		./src/nodeManager/OjUdpComponentInterface.cpp \
		./src/nodeManager/CommunicatorComponent.cpp \
		./src/nodeManager/JausNodeCommunicationManager.cpp \
		./src/nodeManager/JudpInterface.cpp \
		./src/nodeManager/MessageRouter.cpp \
		./src/nodeManager/events/DebugEvent.cpp \
		./src/nodeManager/events/ErrorEvent.cpp \
		./src/nodeManager/events/ConfigurationEvent.cpp \
		./src/nodeManager/events/NodeManagerEvent.cpp \
		./src/nodeManager/events/JausMessageEvent.cpp \
		./src/nodeManager/events/SystemTreeEvent.cpp \
		./src/nodeManager/JausSubsystemCommunicationManager.cpp \
		./src/nodeManager/JausOpcUdpInterface.cpp

OBJECTS =	./Build/ojCmpt.o \
		./Build/multicastSocket.o \
		./Build/queue.o \
		./Build/inetAddress.o \
		./Build/timeLib.o \
		./Build/datagramPacket.o \
		./Build/datagramSocket.o \
		./Build/nodeManagerInterface.o \
		./Build/lmHandler.o \
		./Build/defaultJausMessageProcessor.o \
		./Build/scManager.o \
		./Build/FileLoader.o \
		./Build/LocalComponent.o \
		./Build/JausTransportInterface.o \
		./Build/JausTransportQueue.o \
		./Build/JausCommunicationManager.o \
		./Build/JausComponentCommunicationManager.o \
		./Build/NodeManager.o \
		./Build/NodeManagerComponent.o \
		./Build/SystemTree.o \
		./Build/OjUdpComponentInterface.o \
		./Build/CommunicatorComponent.o \
		./Build/JausNodeCommunicationManager.o \
		./Build/JudpInterface.o \
		./Build/MessageRouter.o \
		./Build/DebugEvent.o \
		./Build/ErrorEvent.o \
		./Build/ConfigurationEvent.o \
		./Build/NodeManagerEvent.o \
		./Build/JausMessageEvent.o \
		./Build/SystemTreeEvent.o \
		./Build/JausSubsystemCommunicationManager.o \
		./Build/JausOpcUdpInterface.o

default : all

all : $(TARGET)

clean :
	rm -f ./Build/*.o
	rm -f $(TARGET)

$(TARGET) : $(OBJECTS)
	g++ -shared $(LFLAGS) -o $(TARGET) $(OBJECTS) $(LIBS)

./Build/ojCmpt.o : ./src/componentLibrary/ojCmpt.c
	gcc $(CCFLAGS) -o ./Build/ojCmpt.o ./src/componentLibrary/ojCmpt.c

./Build/multicastSocket.o : ./src/utils/multicastSocket.c
	gcc $(CCFLAGS) -o ./Build/multicastSocket.o ./src/utils/multicastSocket.c

./Build/queue.o : ./src/utils/queue.c
	gcc $(CCFLAGS) -o ./Build/queue.o ./src/utils/queue.c

./Build/inetAddress.o : ./src/utils/inetAddress.c
	gcc $(CCFLAGS) -o ./Build/inetAddress.o ./src/utils/inetAddress.c

./Build/timeLib.o : ./src/utils/timeLib.c
	gcc $(CCFLAGS) -o ./Build/timeLib.o ./src/utils/timeLib.c

./Build/datagramPacket.o : ./src/utils/datagramPacket.c
	gcc $(CCFLAGS) -o ./Build/datagramPacket.o ./src/utils/datagramPacket.c

./Build/datagramSocket.o : ./src/utils/datagramSocket.c
	gcc $(CCFLAGS) -o ./Build/datagramSocket.o ./src/utils/datagramSocket.c

./Build/nodeManagerInterface.o : ./src/nodeManagerInterface/nodeManagerInterface.c
	gcc $(CCFLAGS) -o ./Build/nodeManagerInterface.o ./src/nodeManagerInterface/nodeManagerInterface.c

./Build/lmHandler.o : ./src/nodeManagerInterface/lmHandler.c
	gcc $(CCFLAGS) -o ./Build/lmHandler.o ./src/nodeManagerInterface/lmHandler.c

./Build/defaultJausMessageProcessor.o : ./src/nodeManagerInterface/defaultJausMessageProcessor.c
	gcc $(CCFLAGS) -o ./Build/defaultJausMessageProcessor.o ./src/nodeManagerInterface/defaultJausMessageProcessor.c

./Build/scManager.o : ./src/nodeManagerInterface/scManager.c
	gcc $(CCFLAGS) -o ./Build/scManager.o ./src/nodeManagerInterface/scManager.c

./Build/FileLoader.o : ./src/utils/FileLoader.cpp
	g++ $(CCFLAGS) -o ./Build/FileLoader.o ./src/utils/FileLoader.cpp

./Build/LocalComponent.o : ./src/nodeManager/LocalComponent.cpp
	g++ $(CCFLAGS) -o ./Build/LocalComponent.o ./src/nodeManager/LocalComponent.cpp

./Build/JausTransportInterface.o : ./src/nodeManager/JausTransportInterface.cpp
	g++ $(CCFLAGS) -o ./Build/JausTransportInterface.o ./src/nodeManager/JausTransportInterface.cpp

./Build/JausTransportQueue.o : ./src/nodeManager/JausTransportQueue.cpp
	g++ $(CCFLAGS) -o ./Build/JausTransportQueue.o ./src/nodeManager/JausTransportQueue.cpp

./Build/JausCommunicationManager.o : ./src/nodeManager/JausCommunicationManager.cpp
	g++ $(CCFLAGS) -o ./Build/JausCommunicationManager.o ./src/nodeManager/JausCommunicationManager.cpp

./Build/JausComponentCommunicationManager.o : ./src/nodeManager/JausComponentCommunicationManager.cpp
	g++ $(CCFLAGS) -o ./Build/JausComponentCommunicationManager.o ./src/nodeManager/JausComponentCommunicationManager.cpp

./Build/NodeManager.o : ./src/nodeManager/NodeManager.cpp
	g++ $(CCFLAGS) -o ./Build/NodeManager.o ./src/nodeManager/NodeManager.cpp

./Build/NodeManagerComponent.o : ./src/nodeManager/NodeManagerComponent.cpp
	g++ $(CCFLAGS) -o ./Build/NodeManagerComponent.o ./src/nodeManager/NodeManagerComponent.cpp

./Build/SystemTree.o : ./src/nodeManager/SystemTree.cpp
	g++ $(CCFLAGS) -o ./Build/SystemTree.o ./src/nodeManager/SystemTree.cpp

./Build/OjUdpComponentInterface.o : ./src/nodeManager/OjUdpComponentInterface.cpp
	g++ $(CCFLAGS) -o ./Build/OjUdpComponentInterface.o ./src/nodeManager/OjUdpComponentInterface.cpp

./Build/CommunicatorComponent.o : ./src/nodeManager/CommunicatorComponent.cpp
	g++ $(CCFLAGS) -o ./Build/CommunicatorComponent.o ./src/nodeManager/CommunicatorComponent.cpp

./Build/JausNodeCommunicationManager.o : ./src/nodeManager/JausNodeCommunicationManager.cpp
	g++ $(CCFLAGS) -o ./Build/JausNodeCommunicationManager.o ./src/nodeManager/JausNodeCommunicationManager.cpp

./Build/JudpInterface.o : ./src/nodeManager/JudpInterface.cpp
	g++ $(CCFLAGS) -o ./Build/JudpInterface.o ./src/nodeManager/JudpInterface.cpp

./Build/MessageRouter.o : ./src/nodeManager/MessageRouter.cpp
	g++ $(CCFLAGS) -o ./Build/MessageRouter.o ./src/nodeManager/MessageRouter.cpp

./Build/DebugEvent.o : ./src/nodeManager/events/DebugEvent.cpp
	g++ $(CCFLAGS) -o ./Build/DebugEvent.o ./src/nodeManager/events/DebugEvent.cpp

./Build/ErrorEvent.o : ./src/nodeManager/events/ErrorEvent.cpp
	g++ $(CCFLAGS) -o ./Build/ErrorEvent.o ./src/nodeManager/events/ErrorEvent.cpp

./Build/ConfigurationEvent.o : ./src/nodeManager/events/ConfigurationEvent.cpp
	g++ $(CCFLAGS) -o ./Build/ConfigurationEvent.o ./src/nodeManager/events/ConfigurationEvent.cpp

./Build/NodeManagerEvent.o : ./src/nodeManager/events/NodeManagerEvent.cpp
	g++ $(CCFLAGS) -o ./Build/NodeManagerEvent.o ./src/nodeManager/events/NodeManagerEvent.cpp

./Build/JausMessageEvent.o : ./src/nodeManager/events/JausMessageEvent.cpp
	g++ $(CCFLAGS) -o ./Build/JausMessageEvent.o ./src/nodeManager/events/JausMessageEvent.cpp

./Build/SystemTreeEvent.o : ./src/nodeManager/events/SystemTreeEvent.cpp
	g++ $(CCFLAGS) -o ./Build/SystemTreeEvent.o ./src/nodeManager/events/SystemTreeEvent.cpp

./Build/JausSubsystemCommunicationManager.o : ./src/nodeManager/JausSubsystemCommunicationManager.cpp
	g++ $(CCFLAGS) -o ./Build/JausSubsystemCommunicationManager.o ./src/nodeManager/JausSubsystemCommunicationManager.cpp

./Build/JausOpcUdpInterface.o : ./src/nodeManager/JausOpcUdpInterface.cpp
	g++ $(CCFLAGS) -o ./Build/JausOpcUdpInterface.o ./src/nodeManager/JausOpcUdpInterface.cpp
