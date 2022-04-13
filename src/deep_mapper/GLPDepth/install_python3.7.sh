sudo apt-get install build-essential checkinstall
sudo apt-get install libreadline-gplv2-dev libncursesw5-dev libssl-dev libsqlite3-dev tk-dev libgdbm-dev libc6-dev libbz2-dev
sudo apt-get install libffi-dev
sudo apt-get install liblzma-dev

cd /usr/src
sudo wget https://www.python.org/ftp/python/3.7.7/Python-3.7.7.tgz
sudo tar xzvf Python-3.7.7.tgz
cd Python-3.7.7
sudo ./configure --enable-optimizations
sudo make
sudo make altinstall

sudo apt install python3-pip