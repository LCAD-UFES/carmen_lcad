include ../Makefile.conf

MODULE_NAME = IMU TESTE
MODULE_COMMENT = Painel de conferencia da IMU do xsens

LINK = gcc

# GTK3 + cairo. O carmen ainda usa GTK2 no Makefile.conf, entao este modulo pede
# o GTK3 direto ao pkg-config em vez de usar o $(GTK_CONFIG) global.
IFLAGS += `pkg-config --cflags gtk+-3.0`
LFLAGS += -lglobal -lipc -lparam_interface -lxsens_interface -lrotation_geometry -lm
LFLAGS += `pkg-config --libs gtk+-3.0`

SOURCES = imu_teste_main.c

PUBLIC_INCLUDES =
PUBLIC_LIBRARIES =
PUBLIC_BINARIES = imu_teste

TARGETS = imu_teste

imu_teste: imu_teste_main.o

include ../Makefile.rules
