all: mpc_brown
 
mpc_brown: main.o trajectories.o  coupled_lat_long.o decoupled_lat_long.o math.o model_predictive_control.o vehicle_dynamics.o vehicles.o
	g++ -o mpc_brown main.o trajectories.o  coupled_lat_long.o decoupled_lat_long.o math.o model_predictive_control.o vehicle_dynamics.o vehicles.o -W -Wall -ansi -pedantic -std=c++11
 
main.o: main.cpp
	g++ -c main.cpp -W -Wall -ansi -pedantic -std=c++11
 
trajectories.o: trajectories.cpp
	g++ -c trajectories.cpp -W -Wall -ansi -pedantic -std=c++11

coupled_lat_long.o: coupled_lat_long.cpp
	g++ -c trajectories.cpp -W -Wall -ansi -pedantic -std=c++11
	
decoupled_lat_long.o:decoupled_lat_long.cpp
	g++ -c decoupled_lat_long.cpp -W -Wall -ansi -pedantic -std=c++11

math.o: math.cpp
	g++ -c math.cpp -W -Wall -ansi -pedantic -std=c++11

model_predictive_control.o: model_predictive_control.cpp
	g++ -c model_predictive_control.cpp -W -Wall -ansi -pedantic -std=c++11

vehicle_dynamics.o: vehicle_dynamics.cpp
	g++ -c vehicle_dynamics.cpp -W -Wall -ansi -pedantic -std=c++11

vehicles.o: vehicles.cpp
	g++ -c vehicles.cpp -W -Wall -ansi -pedantic -std=c++11

clean:
	rm -rf *.o mpc_brown