sudo apt update
sudo apt install build-essential checkinstall zlib1g-dev libncurses5-dev libncursesw5-dev \
 libgdbm-dev libnss3-dev libssl-dev libreadline-dev libffi-dev wget \
 libsqlite3-dev tk-dev libc6-dev libbz2-dev

sudo ldconfig

cd /tmp
sudo wget https://www.python.org/ftp/python/3.7.7/Python-3.7.7.tgz
sudo tar -xf Python-3.7.7.tgz
cd Python-3.7.7
#sudo ./configure --enable-shared --enable-profiling --enable-optimizations
sudo ./configure --enable-optimizations
sudo make altinstall


