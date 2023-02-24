#! /bin/bash

regras_filtro=\
(SUBSYSTEMS ATTRS{name} ATTRS{phys} ATTRS{uniq})

regras_dispositivo=\
()


remove_regras_dispositivos()
{
    sed -i "/$1/ d" /etc/udev/rules.d/99-usb-keyboard.rules
#   linhas_sysctl=$(cat /etc/udev/rules.d/99-usb-keyboard.rules | wc -l)
#   sed -i "$linhas_sysctl d" /etc/udev/rules.d/99-usb-keyboard.rules

    echo "O dispositivo foi removido."
}


registre_regras_dispositivo()
{
    linhas_sysctl=$(cat /etc/udev/rules.d/99-usb-keyboard.rules | wc -l)
    
    sed -i "$linhas_sysctl a ${regras_dispositivo[0]:6},${regras_dispositivo[1]:6},${regras_dispositivo[2]:6},${regras_dispositivo[3]:6}, SYMLINK+=\"$1\", MODE=\"0666\"" /etc/udev/rules.d/99-usb-keyboard.rules

    echo -e "Dispositivo registrado."
}


construct()
{
    total_regras_filtro=$((${#regras_filtro[@]}))
    
    for ((i=0; i< $total_regras_filtro; i++)); do
        regras_dispositivo[i]=$(udevadm info --attribute-walk --name=$1 | grep ${regras_filtro[i]}| grep -n ^ | grep -w ^1)

    done
}


esta_registrado()
{
    aux_teste_exist=$(udevadm info --attribute-walk --name=$1 | grep ATTRS{phys}| grep -n ^ | grep -w ^1| awk {'print $2'})
    teste_exist=$(cat /etc/udev/rules.d/99-usb-keyboard.rules | grep ${aux_teste_exist:11:-1})
    
    paramentro_existencia=0
    
    if test -z "$teste_exist"; then
        paramentro_existencia=1

    fi

    echo $paramentro_existencia
}


check_none()
{
	flag_existe=1

	if test $# -eq 2; then
	    if test -z "$1"; then
	        echo "$2"
	
	        exit
	
	    fi
	else
        if test -z "$1"; then
        	flag_existe=0
        
        fi
        
        echo "$flag_existe"
        
	fi
}


existe_file()
{
	if test -f "$1"; then
		echo "$2"
		
	fi
}


check_user()
{
	user=$(whoami)
	
	if test "$user" != "root"; then
	 echo "Para ralizar esse comando é necessário executar este script usando permissões de super usuario."
	
	 exit
	 
	fi
}


dispositivos_conectado()
{
	readarray -t vet_dispositivos_conectados < <(ls $1 |grep kbd |grep -v pci)
	
	flag_existe=$(check_none $vet_dispositivos_conectados)
	
	if test $flag_existe -eq 1; then
		total_dispositivos_conectados=$((${#vet_dispositivos_conectados[@]}))
		
	    for((i=0; i < total_dispositivos_conectados; i++)); do
	        echo -e "	$1/${vet_dispositivos_conectados[i]}"
	
	    done
	
	else
		echo -e "	Não existe nenhum dispositivo conectado."
		
	fi
	
	echo -e ""
}


file_creator()
{
	verificacao_existencia=$(ls /etc/udev/rules.d/ |grep 99-usb-keyboard.rules)
	
	flag_existe=$(check_none $verificacao_existencia)
	
	if test $flag_existe -eq 0; then
		touch /home/lume/carmen_lcad/src/lume_extra_keys/99-usb-keyboard.rules
		
		echo "#" > /home/lume/carmen_lcad/src/lume_extra_keys/99-usb-keyboard.rules
		
		mv /home/lume/carmen_lcad/src/lume_extra_keys/99-usb-keyboard.rules /etc/udev/rules.d/
		
		
	fi
}


main()
{	
	if test "$1" = "-add"; then
		check_none "$2" "Nenhum dispositivo inserido"
		check_none "$3" "Nome do link não inserido"
		
		check_user
	
		existe_file $2 "dispositivo não existe ou não esta conectado."
		
		
		file_creator
		
	    paramentro_existencia=$(esta_registrado $2)
	    
	    if test "$paramentro_existencia" -ne 1; then
	        echo "Dispositivo ja esta registrado."
	        
	        exit
	    fi
	
	    construct $2
	    registre_regras_dispositivo $3
	    
	    exit
	
	elif test "$1" = "-rm"; then
		check_none "$2" "Nenhum Nome de link inserido"
		
		check_user
	
	    dispositivo_remov=$(cat /etc/udev/rules.d/99-usb-keyboard.rules |grep -w $2)
	    
	    check_none "$dispositivo_remov" "dispositivo não está registrado."
	
	    remove_regras_dispositivos "$2"
	
	elif test "$1" = "-list_c"; then
	    by_id="/dev/input/by-id"
	    by_path="/dev/input/by-path"
		
	    echo -e "by-id: "
	    
	    dispositivos_conectado "$by_id"
	    
	    echo -e "by-path: "
	    
	    dispositivos_conectado "$by_path"
	
	elif test "$1" = "-list_r"; then
		check_user
		
		file_creator
	
	    readarray -t mostra_nomes_dispositivos < <(cat /etc/udev/rules.d/99-usb-keyboard.rules |grep SYMLINK | tr "," "\n" | grep SYMLINK)
		
		paramentro_existencia=$(check_none $mostra_nomes_dispositivos)
		
		if test $paramentro_existencia -eq 1; then
		    echo "Dispositivos registrados: "
		
		    total_mostra_nomes_dispositivos=$((${#mostra_nomes_dispositivos[@]}))
		    
		    for((i=0; i < total_mostra_nomes_dispositivos; i++)); do
		        echo -e "   ${mostra_nomes_dispositivos[i]:11:-1}"
		
		    done
		    
		    echo -e
		  	
		  	exit
		  	  
		fi
		
		echo -e "Não existe nenhum dispositivo registrado."
	
	elif test "$1" = "-h" || test "$#" -eq 0; then
	    echo -e " Para registrar um novo dispositivo execute o script com -add, ex: sudo bash config_extra_keys.bash -add /dev/input/by-id/usb-5131_2019-event-kbd teclado_principal (aqui vc
 pode inserir um nome da sua preferencia).
	
	* Para remover um dispositivo, execute o script com -rm, ex: sudo bash config_extra_keys.bash -rm teclado_principal.
	* Para listar os dispositivos disponiveis para registro, execute o script com -list_c, ex: bash config_extra_keys.bash -list_c.
	* Para lista os dispositivos já registrados, execute o script com -list_r, ex: sudo bash config_extra_keys.bash -list_r.
	* Para listar os dispositivos disponiveis para uso, execute o comando ls /dev e procure o dispositivo cadastrado.
	    
 obs: para mais informações consulte o README.md do modulo lume_extra_keys."
	
	else
	    echo "Essa opção não foi encontrada, excute o script usando -h para mais informações, ex: bash config_extra_keys.bash -h."
	    
	fi
}

main $@