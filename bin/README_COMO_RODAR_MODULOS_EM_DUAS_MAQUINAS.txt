Para rodar modulos carmen em maquinas diferentes na rede usa-se o multicentral.
Usar apenas um central em uma das maquinas e setar a variavel de ambiente CENTRALHOST com o ip da maquina com central master, gera muita latencia.
Principalmente na troca de mensagens de camera.

Por isso foi criado um modulo ponte usando a biblioteca multicentral.
Desta forma rodamos um central em cada maquina, e enviamos pela rede, publicando apenas o que eh necessario.
No nosso caso, conectamos a bumblebee 3 na car02 e todos os modulos que usam camera, devem rodar na car02, e publicar apenas as mensagens do processamento para
a car01 se necessario.

$CARMEN_HOME/src/multicentral_bridge_traffic_light -central centrals_list.txt
esse modulo se subscreve no central da car01 e no central da car02.
Do central da car01 recebe global_pos e rddf_annotation. Do central da car02 recebe a bumblebee conectada na car02 e traffic_light.
Entao publica para a car01 a mensagem do traffic_light.

Para usar a ponte do traffic_light:
Na primeira maquina(car01) rode o central e o process
	./central
	./proccontrol process-ida-ate-a-ponte-pid.ini
Abra um novo terminal, acesse a segunda maquina(car02) via SSH com interface (-Y) e rode o process do traffic_light
	ssh -Y car02@192.168.0.108
	cd carmen_lcad/bin
	./central &
	./proccontrol process-multicentral_bridge_traffic_light.ini

Uma janela do proccontrol_gui abrira na primeira maquina (car01). Eh possivel gerenciar os modulos que estao rodando na outra maquina(car02).
Este process roda na outra maquina os seguintes modulos:
./param_daemon ../src/carmen-ford-escape.ini
./proccontrol_gui
./bridge_traffic_light -central centrals_list.txt 
./bumblebee_basic 3
./traffic_light 3
./bumblebee_basic_view 3
./traffic_light_view 3

Para mais informacoes veja o README dentro do modulo multicentral_bridge

Caso precise acessar remotamente e ver a interface da car02 use o Remmina na car01 e conecte via VNC a car02. Na car02 o Desktop Share deve estar ativado.
