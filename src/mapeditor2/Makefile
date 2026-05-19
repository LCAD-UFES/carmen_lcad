include ../Makefile.conf

MODULE_NAME = MAP_EDITOR2
MODULE_COMMENT = "Ferramentas de leitura e escrita de mapas"

CC = g++
CFLAGS += -fpermissive

# --- CONFIGURAÇÃO DE BIBLIOTECAS ---
IFLAGS += `pkg-config opencv --cflags`
LFLAGS += -lmap_io -lmap_interface -lglobal_graphics \
      -lglobal -lipc -lparam_interface \
      -lpthread -lz -lm \
      `pkg-config opencv --libs`

# --- MUDANÇA AQUI: Listamos os dois arquivos novos ---
SOURCES = carmen_read.c carmen_save.c

# --- MUDANÇA AQUI: Definimos os dois executáveis como alvo ---
TARGETS = carmen_read carmen_save

# Regras individuais de ligação
carmen_read: carmen_read.o
carmen_save: carmen_save.o

include ../Makefile.rules