- Os dados de treino sao, na verdade, advindos dos arquivos obtidos como abaixo (isso foi feito para nao ter que mudar o nome dos arquivos no svn):
	cp ~/PID_ANALISYS/results_pid-velocity-20140325-6.txt DadosPID/results_pid-velocity-20140320-6.txt 
	cp ~/PID_ANALISYS/results_pid-velocity-20140325-2.txt DadosPID/results_pid-velocity-20140320-7.txt 
	cp -r DadosPID DadosPID_bak2
	cat DadosPID_bak2/results_pid-velocity-20140320-6.txt DadosPID_bak2/results_pid-velocity-20140320-7.txt > DadosPID/results_pid-velocity-20140320-6.txt 
	cat DadosPID_bak2/results_pid-velocity-20140320-7.txt DadosPID_bak2/results_pid-velocity-20140320-6.txt > DadosPID/results_pid-velocity-20140320-7.txt 
	
