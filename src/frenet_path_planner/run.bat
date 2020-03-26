gcc -o cubic_spline_study cubic_spline_study.c -lm -lgsl -lcblas
./cubic_spline_study > interp.dat
#graph -y -70 70 -T ps < interp.dat > interp.ps
graph -T ps < interp.dat > interp.ps
evince interp.ps 


