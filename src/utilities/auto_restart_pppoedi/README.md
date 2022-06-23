
# Auto-restart computer and connect to PPPoE DI

After a power failure, it is convenient to automatically restart a computer and connect to PPPoE DI.

## Configuration

1. Configure o BIOS da sua máquina para automaticamente religar após uma queda de energia. No BIOS GIGABYTE, esta opção está em:

> _Settings_ > _Platform Power_ > _Power Loading_ > _Auto_

2. Instale o PPPoEDI fornecido pelo LAR/UFES. Você deve obter um usuário e senha do LDAP DI (LAR/UFES) para poder fazer o login no PPPoE. Fonte: <https://git.inf.ufes.br/LAR-UFES/pppoe-di>

3. Instale o AnyDesk. Fonte: <https://anydesk.com>

4. Configure o AnyDesk para ter uma password fixa. Na versão 6, está em:

> _Settings_ > _Security_ > _Enable unattended access_ , _Set password for unattended access_

5. Copie o scrip: 

> sudo cp $CARMEN_HOME/src/utilities/auto_restart_pppoedi/start_pppoedi.sh  /usr/local/bin/

Alternativamente, edite o script com as linhas abaixo: 

> sudo gedit  /usr/local/bin/start_pppoedi.sh

```
#!/bin/bash
LDAP_DI_user="type here your LDAP DI user (get from LAR/UFES)"
LDAP_DI_password="type here your password"
while [ 1 ]
do
    PPP=$(ifconfig | grep UP | grep POINTOPOINT | grep RUNNING)
    if ! [ -n "$PPP" ]; then
        /usr/local/bin/pppoedi-cli  ${LDAP_DI_user}  ${LDAP_DI_password}
    fi
    sleep 60
done
```

6. Certifique que o script esteja executável: 

> sudo chmod +x  /usr/local/bin/start_pppoedi.sh

7. Certifique que o systemd esteja instalado: 

> sudo apt update
> sudo apt install systemd

8. Copie o arquivo: 

> sudo cp $CARMEN_HOME/src/utilities/auto_restart_pppoedi/pppoedi.service  /etc/systemd/system/

Alternativamente, edite o arquivo com as linhas abaixo: 

> sudo gedit  /etc/systemd/system/pppoedi.service

```
[Unit]
Description=PPPoEDI run
After=network.target

[Service]
Environment="PATH=/bin"
ExecStart=/usr/local/bin/start_pppoedi.sh
StartLimitInterval=60
StartLimitBurst=10

[Install]
WantedBy=default.targe
```

9. Instale o serviço systemd e habilite-o para início automático no boot:

> sudo  systemctl  daemon-reload
> sudo  systemctl  enable pppoedi.service
> sudo  systemctl  start  pppoedi.service

A configuração está concluída. Faça um reboot para testar se a máquina fica remotamente acessível. Caso queira conhecer outras opções válidas no systemd: 

> man  systemd.service

