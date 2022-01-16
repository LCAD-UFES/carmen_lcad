# DeepMapper

DeepMapper is a Carmen subsystem to estimate depth maps from monocular camera images using Neural Networks. 

## Pretrained model

The pretrained models "AdaBins_nyu.pt" and "AdaBins_kitti.pt" are available at [here](https://1drv.ms/u/s!AuWRnPR26byUmfRxBQ327hc8eXse2Q?e=AQuYZw).
* Download these weights and save them at "$CARMEN_HOME/src/deep_mapper/pretrained/", it is needed for running the Neural Network.

## Preparing the environment

```shell
    cd $CARMEN_HOME/src/deep_mapper/
```
```shell
    make
```
If your system already has CUDA 10.0 e CUDNN compatibles installed, run the command:
```shell
    ./install_dependencies.sh
```

* For using this subsytem is required: CUDA_10.0 (and CUDNN compatible), Python 3.5 (or superior), pip and virtualenv.

### In case your machine doesn't have CUDA and CUDNN
* Run the command and all dependencies are gonna be installed:
```shell
    ./install_dependencies.sh 1
```
The parameter 1 is required to install CUDA and CUDNN.


## For using the module in Carmen_LCAD

 Once the weights are saved at "pretrained" folder and the project CARMEN_LCAD compiled and all dependencies installed, <br/>
 run the commands bellow to use. 
 
### Include the directories in your PYTHONPATH
#### First option
```shell
 echo "export PYTHONPATH=$CARMEN_HOME/src/deep_mapper/:$CARMEN_HOME/src/deep_mapper/models/:$PYTHONPATH" >> ~/.bashrc
 source ~/.bashrc
```
#### Second option - Through Text Editor:
```shell
    gedit ~/.bashrc
```
And include these lines in the end of file:
```shell
    #Deep mapper
    export PYTHONPATH=$CARMEN_HOME/src/deep_mapper:$CARMEN_HOME/src/deep_mapper/models:$PYTHONPATH
```
Save and close the Text Editor. For reload the environment variables, run:
```shell
    source ~/.bashrc
```

### Configuring a process.ini (optional)
It is necessary ajust some process.ini to load inside a proccontrol process. Edit the choosen process and add the line:
```
deep_map_ART    depth_map   0   0   ./deep_mapper  -camera_id 1  # using with ART and Intelbras
deep_map_IARA   depth_map   0   0   ./deep_mapper  -camera_id 3  # using with IARA and Bumblebee 
```

### Execution
Execute these steps:
```shell
cd $CARMEN_HOME/bin/
./central
```
```shell
cd $CARMEN_HOME/bin/
./proccontrol process-playback-fovea.ini
```
```shell
cd $CARMEN_HOME/bin/
source $CARMEN_HOME/src/deep_mapper/venv/bin/activate
(venv) lcad@lcad:~/carmen_lcad/bin$ ./deep_mapper -camera_id 3
```
After loading the pretrained weights, just play and run. The results are showed.


## Testing the network with a video

## Visualize the images side by side - original/depth
* Run the command:
```
source $CARMEN_HOME/src/deep_mapper/venv/bin/activate
python infer_video.py --model kitti --input test_video.mp4
deactivate
```
Obs.: test_video.mp4 is some choosen video.

* We recommend use an GPU TitanV with 11GB, otherwise it can be slow.

# Original Article
[AdaBins](https://arxiv.org/abs/2011.14141)
