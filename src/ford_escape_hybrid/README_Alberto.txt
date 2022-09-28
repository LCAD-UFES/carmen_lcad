./ford_escape_hybrid_tune_pid -max_v 1.5 -max_phi 0.0 -timer_period 1.0 -t1 8.0 -t2 4.0 -t3 8.0
./ford_escape_hybrid_tune_pid -max_v 0.0 -max_phi 10.0 -timer_period 1.0 -t1 8.0 -frequency 0.125


## Plotando pid do audit/log
Para realizar o plot do pid do velocity e do steering (lendo as mensagens: FORD_ESCAPE_VELOCITY_PID_DATA_MESSAGE e FORD_ESCAPE_STEERING_PID_DATA_MESSAGE), execute:
	./plot_pid_from_log.sh /dados/audit/timestamp/timestamp/arquivo_audit.txt
Duas janelas do gnuplot vão abrir, a com título Velocity Pid possui a plotagem dos valores:
	 'current velocity' , 
     'desired velocity' ,
     'error t' ,
     'integral t' ,
     'derivative t' ,
     'throttle command',
     'brakes command' 
da mensagem "velocity_pid_data_message".
A janela com título Steering Pid possui a plotagem dos valores:
	'atan current curvature',
	'atan desired curvature',
	'error t',
	'integral t',
	'derivative t',
	'effort'
da mensagem "steering_pid_data_message".