# --- Caminhos Base ---
CARMEN_HOME = ../..
UNITREE_SDK_DIR = /usr/local

# --- Compilador e Flags ---
CXX = g++
CFLAGS = -Wall -O3 -std=c++17 -Wno-address-of-packed-member -fPIC

# --- Includes (Onde os arquivos .h estão) ---
IFLAGS = -I$(CARMEN_HOME)/include \
         -I$(UNITREE_SDK_DIR)/include \
         -I$(UNITREE_SDK_DIR)/include/ddscxx

# --- Bibliotecas (Onde os arquivos .so e .a estão) ---
LFLAGS = -L$(CARMEN_HOME)/lib \
         -L$(UNITREE_SDK_DIR)/lib \
         -L/usr/local/ipc-3.9.1/src/lib/Linux-4.14/ \
         -L$(CARMEN_HOME)/sharedlib/libcmt/

# --- Bibliotecas para Linkar (ORDEM CORRIGIDA) ---
# A ordem correta: Depende de IPC -> Global -> IPC
LIBS = -lunitree_sdk2 -lddscxx -lddsc -lxsens_mtig_interface -lxsens_interface -lparam_interface -lglobal -lipc -lrt -lpthread

# --- Arquivos Fontes e Alvos ---
TARGET = xsens_argos
OBJS = xsens_argos_main.o

# --- Regras Principais ---
all: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) $(CFLAGS) $(OBJS) -o $(TARGET) $(LFLAGS) $(LIBS)
	@echo "Copiando executável para bin/"
	cp $(TARGET) $(CARMEN_HOME)/bin/

%.o: %.cpp
	$(CXX) $(CFLAGS) $(IFLAGS) -c $< -o $@

clean:
	rm -f *.o $(TARGET) $(CARMEN_HOME)/bin/$(TARGET)