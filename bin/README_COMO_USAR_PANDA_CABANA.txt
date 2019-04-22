# Como usar Panda e Cabana no Ubuntu
# 
# https://community.comma.ai/wiki/index.php/Panda
# https://community.comma.ai/wiki/index.php/Cabana
#
# Caso ainda nao exista o arquivo /etc/udev/rules.d/11-panda.rules , crie-o da seguinte forma:
#
#   sudo -i
#   echo 'SUBSYSTEMS=="usb", ATTR{idVendor}=="bbaa", ATTR{idProduct}=="ddcc", MODE:="0666"' > /etc/udev/rules.d/11-panda.rules
#   exit
#
# Conecte o Panda na interface OBD-II do carro e conecte um cabo USB macho/macho no Panda e no computador
#
# Execute o Google Chrome com os seguintes parametros:
#
#   sudo /opt/google/chrome/chrome -enable-features=WebUSB --no-sandbox
#
#
# Entre na aplicacao web Cabana:
#
#   https://community.comma.ai/cabana

