#! /bin/bash

user_sem_tratamento=$(env |grep SUDO_USER)

USER=${user_sem_tratamento:10}
CARMEN_HOME="/home/$USER/carmen_lcad"


exist_name_interface() {
 confirmacao=0

 for i in $@; do
  test_existencia_interface=$(ifconfig | grep "$i" | awk '{ print $1 }')

  if test "$test_existencia_interface" != ""; then
   if test "${test_existencia_interface:0:-1}" != "$i"; then
    echo "A interface $i nao existe nesse dispositivo"

    confirmacao=1

   fi
  
  else
   echo "A interface $i nao existe nesse dispositivo"

   confirmacao=1

  fi
 done
  
 if test "$confirmacao" -eq 1; then
  exit

 fi
}


solicitacao_shutdown(){
 echo -ne "\no computador precisa ser reiniciado, desejafazer isso agora [yes/no]: " 
 read escolha

 if test "$escolha" = "yes" || test "$escolha" = "y"; then
 shutdown
 fi

 exit
}


ordem_parametros(){

 flag_confirma=0

 type_interface=\
(enp eno)
 
 all_interface=$((${#type_interface[@]}))

 for ((i=0; i < all_interface; i++)); do
  if test "${type_interface[i]}" == "${1:0:3}"; then
   flag_confirma=1

   break;

  fi
 done

 if test $flag_confirma -eq 0; then
   echo "Os parametros não estão na ordem certa, exemplo de entrada valida: config_rede_car.bash enp1s0 wlp2s0"
  
   exit

  fi
}


if test "$1" = "-h" || test $# -eq 0; then
 echo -e "Os elementos wlp2s0 enp1s0 sao nomes das interfaces de rede presente no seu Pc, para saber quais sao esses
 nomes no seu pc siga o roteiro a seguir:

 caso nao tenha o net-Tools:
  sudo apt-get update
  sudo apt-get install net-Tools
 
 caso ja tenha:
  ifconfig
 
 copie os nomes e acrescente no comando de execucao do script, exemplo: ./config_rede_car.bash enp1s0 wlp2s0\n"

 echo "Para adicionar um novo dispositivo a rede já exitente execute o script usando -n, ex: sudo bash config_rede_car.bash -n enp1s0 wlp2s0.
Para resetar todas as configurações já inseridas execute o script usando -r, ex: sudo bash config_rede_car.bash -r."

 exit
fi

user=$(whoami)

if test "$user" != "root"; then
 echo "É necessário executar este script usando permissões de super usuario."

 exit

fi

if test "$1" = "-n" && test "$#" -eq 3; then
 exist_name_interface $2 $3 

 ordem_parametros $2

 echo -e "\n########################    criando um NAT do $3 para a subrede da $2    ########################"
 echo "-----------------------------------------------------------------------------------------------------------"
 echo "iptables -A FORWARD -o $3 -i $2 -s 192.168.1.0/24 -m conntrack --ctstate NEW -j ACCEPT"

 iptables -A FORWARD -o "$3" -i "$2" -s 192.168.1.0/24 -m conntrack --ctstate NEW -j ACCEPT

echo -e "\n-----------------------------------------------------------------------------------------------------------"
echo "iptables -t nat -A POSTROUTING -o $2 -j MASQUERADE"

 iptables -t nat -A POSTROUTING -o "$3" -j MASQUERADE

echo -e "\n-----------------------------------------------------------------------------------------------------------"
echo "iptables-save | tee /etc/iptables.sav "

 iptables-save | tee /etc/iptables.sav

solicitacao_shutdown


elif test "$1" = "-r"; then
 vet_config=\
("net.ipv4.ip_forward=1" "net.ipv4.conf.default.forwarding=1" "net.ipv4.conf.all.forwarding=1")
  
 estado=\
("disable" "stop")

 echo -e "\n-----------------------------------------------------------------------------------------------------------"
 echo "sudo systemctl ${estado[1]} restore-iptables-connection.service"

 systemctl ${estado[1]} restore-iptables-connection.service

 echo -e "\n-----------------------------------------------------------------------------------------------------------"
 echo "sudo systemctl ${estado[0]} restore-iptables-connection.service"

 systemctl ${estado[0]} restore-iptables-connection.service

 echo -e "\n-----------------------------------------------------------------------------------------------------------"
 
 sh -c "echo 0 > /proc/sys/net/ipv4/ip_forward"

  total_vet=$((${#vet_config[@]}))
 for ((i=0; i < $total_vet; i++)); do
  echo "Deleted -> ${vet_config[i]}"

  sed -i "/${vet_config[i]}/ d" /etc/sysctl.conf

 done
 
 linhas_sysctl=$(cat /etc/sysctl.conf | wc -l)
 sed -i "$linhas_sysctl d" /etc/sysctl.conf
 
 rm /etc/systemd/system/restore-iptables-connection.service
 rm /usr/bin/restore-iptable.sh

 solicitacao_shutdown
fi

exist_name_interface $1 $2

ordem_parametros $1

echo -e "\n##########################    criando um NAT do ""$2"" para a subrede da "$1"    ##########################"
echo "-----------------------------------------------------------------------------------------------------------"
echo "iptables -A FORWARD -o $2 -i $1 -s 192.168.1.0/24 -m conntrack --ctstate NEW -j ACCEPT"

iptables -A FORWARD -o "$2" -i "$1" -s 192.168.1.0/24 -m conntrack --ctstate NEW -j ACCEPT

echo -e "\n -----------------------------------------------------------------------------------------------------------"
echo "iptables -A FORWARD -m conntrack --ctstate ESTABLISHED,RELATED -j ACCEPT"

iptables -A FORWARD -m conntrack --ctstate ESTABLISHED,RELATED -j ACCEPT

echo -e "\n-----------------------------------------------------------------------------------------------------------"
echo "iptables -t nat -F POSTROUTING"

iptables -t nat -F POSTROUTING

echo -e "\n-----------------------------------------------------------------------------------------------------------"
echo "iptables -t nat -A POSTROUTING -o $2 -j MASQUERADE"

iptables -t nat -A POSTROUTING -o "$2" -j MASQUERADE

echo -e "\n-----------------------------------------------------------------------------------------------------------"
echo "iptables-save | tee /etc/iptables.sav "

iptables-save | tee /etc/iptables.sav


echo -e "\n\n##########################    copiando restore-iptable.sh para o /usr/bin    ##########################"
echo "-----------------------------------------------------------------------------------------------------------"
echo "cp $CARMEN_HOME/src/utilities/scripts/restore-iptable.sh /usr/bin/"

cp $CARMEN_HOME/src/utilities/scripts/restore-iptable.sh /usr/bin/

echo -e "\n-----------------------------------------------------------------------------------------------------------"
echo "chmod +x  /usr/bin/restore-iptable.sh"

chmod +x  "/usr/bin/restore-iptable.sh"


echo -e "\n\n##############  Instalando o serviço systemd e habilitando o para início automático no boot  ##############"

echo -e "\n-----------------------------------------------------------------------------------------------------------"
echo "sudo cp $CARMEN_HOME/src/utilities/scripts/restore-iptables-connection.service /etc/systemd/system/"

cp $CARMEN_HOME/src/utilities/scripts/restore-iptables-connection.service /etc/systemd/system/ 

echo -e "\n-----------------------------------------------------------------------------------------------------------"
echo "sudo chmod 664 /etc/systemd/system/restore-iptables-connection.service"

chmod 664 /etc/systemd/system/restore-iptables-connection.service

echo -e "\n-----------------------------------------------------------------------------------------------------------"
echo "sudo systemctl daemon-reload"

systemctl daemon-reload

echo -e "\n-----------------------------------------------------------------------------------------------------------"
echo "sudo systemctl enable restore-iptables-connection.service"

systemctl enable restore-iptables-connection.service

echo -e "\n-----------------------------------------------------------------------------------------------------------"
echo "sudo systemctl start restore-iptables-connection.service"

systemctl start restore-iptables-connection.service

echo -e "\n-----------------------------------------------------------------------------------------------------------"
echo "sudo sh -c echo 1 > /proc/sys/net/ipv4/ip_forward"

sh -c "echo 1 > /proc/sys/net/ipv4/ip_forward"

echo -e "\n-----------------------------------------------------------------------------------------------------------"
echo "tornando permanente..."

linhas_sysctl=$(cat /etc/sysctl.conf | wc -l)
sed -i "$linhas_sysctl s/$/\n/" /etc/sysctl.conf

linhas_sysctl=$(expr $linhas_sysctl + 1)
sed -i "$linhas_sysctl a net.ipv4.ip_forward=1\nnet.ipv4.conf.default.forwarding=1\nnet.ipv4.conf.all.forwarding=1" /etc/sysctl.conf

solicitacao_shutdown