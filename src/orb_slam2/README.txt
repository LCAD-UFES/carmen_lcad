
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

# ADD THE FOLLOWING LINE IN ~/.bashrc
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/packages_carmen/ORB_SLAM2/lib:~/packages_carmen/ORB_SLAM2/Thirdparty/DBoW2/lib

# RUN 
./orb_slam2_main <camera_id> <output file to save poses>
Example: 
./orb_slam2_main 3 camera_poses.txt
