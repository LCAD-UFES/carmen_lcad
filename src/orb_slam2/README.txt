
# INSTALL DEPENDENCY - PANGOLIN
cd ~/packages_carmen
git clone https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin
mkdir build
cd build
cmake ..
cmake --build .
sudo make install

# INSTALL ORB_SLAM2
cd ~/packages_carmen
git clone https://github.com/raulmur/ORB_SLAM2.git ORB_SLAM2
cd ORB_SLAM2
chmod +x build.sh
./build.sh

if occurs the error  usleep was not defined.
add #include <unistd.h> in the files ~/packages_carmen/ORB_SLAM2/include/System.h
								include/LoopClosing.h
								include/LocalMapping.h
if the error persists try to back the Pangolin version

cd ~/packages_carmen/ORB_SLAM2
rm -rf build

cd ~/packages_carmen/Pangolin
rm -rf build
git checkout v0.5
git pull origin v0.5
mkdir build
cd build
cmake ..
cmake --build .
sudo make install

cd ~/packages_carmen/ORB_SLAM2
mkdir build
chmod +x build.sh
./build.sh

# ADD THE FOLLOWING LINE IN ~/.bashrc
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/packages_carmen/ORB_SLAM2/lib:~/packages_carmen/ORB_SLAM2/Thirdparty/DBoW2/lib

# RUN 
./orb_slam2_main <camera_id> <output file to save poses>
Example: 
./orb_slam2_main 3 camera_poses.txt
