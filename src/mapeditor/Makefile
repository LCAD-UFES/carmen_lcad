include ../Makefile.conf

MODULE_NAME = MAP_EDITOR
MODULE_COMMENT = "Provides photoshop-like map editor."

CC = g++
CFLAGS += -fpermissive

ifndef NO_GRAPHICS
IFLAGS += `$(GTK_CONFIG) --cflags` -ggdb 
IFLAGS += `pkg-config opencv --cflags`
LFLAGS += -lmap_interface -lmap_io -lglobal_graphics \
	`$(GTK_CONFIG) --libs` -lglobal -lipc
LFLAGS += `pkg-config opencv --libs`
	
SOURCES = map_editor.c map_editor_drawing.c map_editor_menus.c \
	   map_editor_graphics.c
TARGETS = map_editor 
PUBLIC_BINARIES = map_editor
endif

# rules

map_editor: map_editor.o map_editor_drawing.o map_editor_graphics.o map_editor_menus.o 

test_menu: test_menu.o

include ../Makefile.rules
