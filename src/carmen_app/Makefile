# carmen_app -- interface remota do carmen (Qt + OpenGL + ZMQ). Roda no notebook, sem carmen.
#
# Makefile independente: NAO esta' no PACKAGES do src/Makefile e nao usa o Makefile.conf do carmen
# (o app nao depende de nenhuma lib do carmen). Compilar so' aqui dentro:
#     make            -> ./carmen_app e o link em ../../bin/carmen_app (como os outros modulos)
#     make clean
#
# Usa o Qt6 se tiver (Ubuntu 26.04: qt6-base-dev), senao o Qt5 (20.04: qtbase5-dev).
# Tambem precisa de libzmq3-dev, zlib1g-dev e libgl-dev.

ifeq ($(shell pkg-config --exists Qt6Widgets && echo yes),yes)
QT_PKGS := Qt6Widgets Qt6OpenGLWidgets Qt6OpenGL
else
QT_PKGS := Qt5Widgets Qt5Gui
endif

CXX      ?= g++
CXXFLAGS += -std=c++17 -O2 -Wall -fPIC -MMD -MP $(shell pkg-config --cflags $(QT_PKGS) libzmq zlib gl)
LDLIBS   += $(shell pkg-config --libs $(QT_PKGS) libzmq zlib gl)

SOURCES = main.cpp main_window.cpp map_view_3d.cpp car_model.cpp dashboard.cpp connection.cpp \
	  map_image.cpp theme.cpp
OBJECTS = $(SOURCES:.cpp=.o)

TARGET  = carmen_app
BIN_DIR = ../../bin

all: $(BIN_DIR)/$(TARGET)

%.o: %.cpp
	@echo "    ---- Compilando $< ($(firstword $(QT_PKGS)))"
	@$(CXX) $(CXXFLAGS) -c $< -o $@

$(TARGET): $(OBJECTS)
	@echo "    ---- Linkando $(TARGET)"
	@$(CXX) -o $@ $(OBJECTS) $(LDLIBS)

# link relativo em bin/, como o PUBLIC_BINARIES dos outros modulos (relativo para nao quebrar
# em outra maquina)
$(BIN_DIR)/$(TARGET): $(TARGET)
	@ln -sf ../src/carmen_app/$(TARGET) $(BIN_DIR)/$(TARGET)
	@echo "    ---- $(BIN_DIR)/$(TARGET) -> src/carmen_app/$(TARGET)"

clean:
	rm -f $(TARGET) $(OBJECTS) $(OBJECTS:.o=.d) $(BIN_DIR)/$(TARGET)

-include $(OBJECTS:.o=.d)

.PHONY: all clean
