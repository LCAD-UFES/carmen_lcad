include ../../src/Makefile.conf


MODULE_NAME = LIB_STEREO_CSBP
MODULE_COMMENT = STEREO CSBP LIB 

CC = g++

TARGETS = libstereo.a

FLAGSLINKER := -lm \
			-std=c99 \
			-Wall 

CFLAGS := -c -fopenmp -O2 


ifndef NO_CUDA

FLAGSLINKER +=  -lcuda \
                -lcudart

CUDALIB64 :=/usr/local/cuda/lib64/
CUDAINCLUDE :=/usr/local/cuda/include/

libstereo.a:  qx_csbp.o  qx_csbp_GPU.o
	ar -rc libstereo.a  qx_csbp.o  qx_csbp_GPU.o
	ranlib libstereo.a


qx_csbp.o: qx_csbp.c
	gcc $(CFLAGS) qx_csbp.c -I$(CUDAINCLUDE) -L$(CUDALIB64) $(FLAGSLINKER)

else

FLAGSLINKER += -DNO_CUDA

libstereo.a:  qx_csbp.o qx_csbp_OMP.o
	ar -rc libstereo.a  qx_csbp.o qx_csbp_OMP.o
	ranlib libstereo.a


qx_csbp.o: qx_csbp.c 
	gcc $(CFLAGS) qx_csbp.c  $(FLAGSLINKER)
  
endif



qx_csbp_OMP.o: qx_csbp_OMP.c
	gcc $(CFLAGS) qx_csbp_OMP.c $(FLAGSLINKER)


	
qx_csbp_GPU.o: qx_csbp_GPU.cu
	nvcc -O -arch=sm_50 -c  qx_csbp_GPU.cu 


#clean:
#	rm -rf output* *.o *.a *~

include ../../src/Makefile.rules
