#Obs 1: Medidas estão em metros
#Obs 2: Em um carro é possível considerar o centro do eixo, o centro da roda (em uma visão lateral)


robot_length               		                            Comprimento total do veliculo
robot_width                		                            Largura total do veículo sem retrovisores
robot_wheel_radius                                          Raio da roda


##Para veículos de tração / reboque. Se tiver o reboque especifico (replicar para quantos modelos forem necessários)
semi_trailer1_d                                             Distância do ponto de engate ao centro do eixo traseiro do semi-trailer/reboque
semi_trailer1_M                                             Distância do ponto de engate (no veículo de tração) ao centro do eixo traseiro (do proprio veículo que irá tracionar o semi-trailer/reboque)


semi_trailer1_width                                         Comprimento total do reboque/trailer
semi_trailer1_distance_between_axle_and_front               Distancia do eixo do trailer (normalmente tem apenas 1 eixo) até o eixo 
semi_trailer1_distance_between_axle_and_back                Distancia do eixo do trailer (normalmente tem apenas 1 eixo) até o traseira do reboque 

Se houver mais de 1 eixo especificar a distância entre os eixos e o eixo principal escolhido
###

robot_max_steering_angle                                    Máx angulo de esterçamento (média das duas rodas dianteiras)                        
robot_distance_between_front_and_rear_axles                 Distância entre a os eixos dianteiro e traseiro
robot_distance_between_rear_wheels             		        Distância entre as rodas traseiras (medido do centro de uma roda até o centro da outra)
robot_distance_between_rear_car_and_rear_wheels    	        Distância entre a traseira do veículo até o eixo traseiro.
robot_distance_between_front_car_and_front_wheels           Distancia entre a dianteira do veiculo até o eixo dianteiro 
robot_distance_between_rearview						    	Distancia entre os retrovisores (Largura do veiculo considerando os retrovisores)

robot_turning_radius								     	Raio de curvatura ou Raio de giro
robot_maximum_capable_curvature                             Curvatura máxima do veículo

robot_maximum_steering_command_rate		                    Velocidade máxima de giro do volante # meters/second


robot_maximum_speed_forward			                        Velocidade máxima                   # meters/second
robot_maximum_speed_reverse			                        Velocidade máxima em marcha ré      # meters/second    
robot_maximum_acceleration_forward		                    aceleração máxima                   # meters/second^2
robot_maximum_acceleration_reverse                          aceleração máxima em marcha ré      # meters/second^2
robot_maximum_deceleration_forward		                    desaceleração máxima                # meters/second^2
robot_maximum_deceleration_reverse		                    desaceleração máxima em marcha ré   # meters/second^2
