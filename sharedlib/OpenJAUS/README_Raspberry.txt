== Para colocar para operar o OpenJaus no Raspberry Pi ==
- Para incluir uma rede WiFi, edite /etc/wpa_supplicant/wpa_supplicant.conf e inclua sua rede
sudo nano /etc/wpa_supplicant/wpa_supplicant.conf
- Em seguida, baixe e suba a interface de rede
sudo ifdown wlan1
sudo ifup wlan1

- Instale a lib curses
sudo apt-get install libncurses5-dev

- Baixe a OpenJaus (apenas) do github do LCAD (https://stackoverflow.com/questions/7106012/download-a-single-folder-or-directory-from-a-github-repo)
sudo apt-get install subversion
cd
svn checkout https://github.com/LCAD-UFES/carmen_lcad/trunk/sharedlib/OpenJAUS

- Depois de baixado o OpenJAUS como acima, todas as mudanças futuras no github podem ser incorporadas com:
svn up

- Para subir coisas para o git use o commit do svn (que já sobe as mudanças). Exemplo:
svn commit -m "adicao de ojNodeManager/IARAnodeManager.conf"

- Compile o OpenJAUS
cd OpenJAUS
make
