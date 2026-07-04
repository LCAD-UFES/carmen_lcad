g++ -o path_smoothing_study -std=c++0x -I/home/alberto/astro/include path_smoothing.cpp path_smoothing_study.cpp \
-L/home/alberto/astro/lib -L/usr/local/lib -lglobal -lgsl -lgslcblas
./path_smoothing_study > interp.dat
#graph -y -50 50 -T ps < interp.dat > interp.ps
graph -T ps < interp.dat > interp.ps
evince interp.ps 


