include $(CARMEN_HOME)/src/Makefile.vars


FLAGSLINKER := 	-O4 \
		-lm \
		-std=c99 \
		-fopenmp \
		-Wall 

CFLAGS := -c 

ifndef NO_CUDA

FLAGSLINKER +=  -lcuda \
                -lcudart

CUDALIB64 :=/usr/local/cuda/lib64/
CUDAINCLUDE :=/usr/local/cuda/include/

#all: vg_ram.o vg_ram_GPU.o
#	gcc -o teste vg_ram.o vg_ram_GPU.o -I$(CUDAINCLUDE) -L$(CUDALIB64) $(FLAGSLINKER)

libstereovgram.a:  vg_ram_GPU.o vg_ram.o 
	ar -rc libstereovgram.a vg_ram.o vg_ram_GPU.o
	ranlib libstereovgram.a
	
vg_ram.o: vg_ram.c
	gcc $(CFLAGS) vg_ram.c  -I$(CUDAINCLUDE) -L$(CUDALIB64) $(FLAGSLINKER)
	
vg_ram_GPU.o: vg_ram_GPU.cu
	nvcc -arch=sm_50 -c vg_ram_GPU.cu
	
else

FLAGSLINKER += -DNO_CUDA

libstereovgram.a:  vg_ram.o
	ar -rc libstereovgram.a vg_ram.o
	ranlib libstereovgram.a
	
vg_ram.o: vg_ram.c
	gcc $(CFLAGS) vg_ram.c  $(FLAGSLINKER)


endif



clean:
	rm -rf output* *.o *.a *~

#include ../../Makefile.rules
