Edite o FSTAB para que o HD novo seja montado no /dados
sudo gedit /etc/fstab

Os HDs (fisicos) estao numerados com um numero escrito a mao

Comente a montagem do HD que sera retirado e descomente o novo que sera instalado
Ex:
Para retirar o HD1 e instalar o HD2

####HDS para DATASET
#HD1
#/dev/disk/by-uuid/3E5526D6788881E7 /dados auto nosuid,nodev,nofail,x-gvfs-show 0 0

#HD2
UUID=36701A1F4D0C6B56 /dados auto nosuid,nodev,nofail,x-gvfs-show 0 0

#HD3
#UUID=117DF34A6D78D94E /dados auto nosuid,nodev,nofail,x-gvfs-show 0 0

Mate todos os processos
Desligue a maquina
Remova o HD 
Instale o novo HD
Ligue a maquina e verifique se todos os processos estao rodando corretamente
    -Checar se o HD esta montado no /dados
    -Checar Se a pasta MOL-exec-log esta criada no /dados e limpa
    -Checar Se o proccontrol_gui esta rodando
    -Checar o camera_viewer se a camera esta rodado normalmente
    -Checar se o log esta sendo gravado
	du -sh /dados/log_vale_mol_2020*

