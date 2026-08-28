Para atualizar o firmware do Canable para candlelight:
 Use o site: https://canable.io/updater/
  Tem que ser no chrome. 
  Funcinou no Ubuntu 18.04. Ler atentamente o site e seguir o procedimento exatamente como indicado!
  Escolha o firmware: candlelight (68df7d5 1/3/21) BETA
  Este metodo da um serial unico a cada vez que o firmware eh atualizado.

 Alternativamente (o CANable vai ficar sempre com o Serial Number: 000...01):
1 - Vá para o Windows e instale as ferramentas contidas no instalador DfuSe_Demo_V3.0.6_Setup.exe que está neste diretório
2 - Insira o canable em uma USB com o jumper BOOT na posição de boot. Isso deve instalar o driver da STM
3 - Execute o programa DfuSe Demo (o pdf DfuSe_Demo.pdf é o manual deste programa)
4 - Na interface deve aparecer o dispositivo STM que é empregado no Canable
5 - Na interface, na seção "Upgrade or Verify Action", clique o botão "Choose..." para escolher o arquivo .dfu com o firmware
6 - Escolha o arquivo canable-candlelight-firmware.dfu e mande fazer o upgrade (é muito rápido o upgrade)
7 - Saia do programa e peça ao Windows para desconetar o Canable da USB
8 - Mude o jumper BOOT de volta para a posição normal de uso do Canable

Note que você pode gerar um arquivo .dfu a partir de um arquivo .bin usando uma ferramenta descrita em DfuSe_Demo.pdf e que é instalada junto com
o DfuSe Demo.

Documentação anterior:

  Procedimento para instalacao de firmware descrito em: https://github.com/cxandy/AZSMZ-USB2CAN/tree/master/Firmware (baixar 
  software para Windows do site: https://github.com/cxandy/AZSMZ-USB2CAN/blob/master/Firmware/DfuSe_Demo_V3.0.5_Setup.exe 
  e usar o firmware: https://github.com/cxandy/AZSMZ-USB2CAN/blob/master/Firmware/gsusb_canable.dfu)

