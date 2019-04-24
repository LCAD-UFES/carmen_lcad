# Laser-new - laser server

Laser server

to run all lasers configured:
`./laser`

to run a specific laser
`./laser -id [laser_id]`

Whenever a new laser is added, it is necessary to increase the `laser_num_laser_devices` parameter on carmen_ford_escape.ini


## Driver Hokuyo 
[Hokuyo UTM-30LX](https://www.hokuyo-aut.jp/02sensor/07scanner/utm_30lx.html)

The message published is carmen_laser_laser_message

### Settings
Check the carmen-ford-escape.ini if you want to modify some of the parameters

The configuration of laser follow the pattern laser_laser[ID]_[options]

The laser_laser1 are the simulated laser (sick)

The Hokuyo front laser is the ID = 2 and the hokuyo rear laser is ID = 3, them:

This lines configures laser 2
```bash
laser_laser2_dev           /dev/ttyACM0
laser_laser2_type          HOKUYOURG
laser_laser2_baud          500000
laser_laser2_resolution    0.5
laser_laser2_fov           270
laser_laser2_flipped    0
laser_laser2_use_remission  none        # none / direct / normalized
```

and, this lines configures the laser 3

```bash
laser_laser3_dev           /dev/ttyACM1
laser_laser3_type          HOKUYOURG
laser_laser3_baud          500000
laser_laser3_resolution    0.5
laser_laser3_fov           270
laser_laser3_flipped    0
laser_laser3_use_remission  none        # none / direct / normalized
```

To log the laser just activate in carmen-ford-escape.ini

`logger_laser   on`

##### Position settings

- The position of the hokuyo laser are relative with the bullbar postion. 
Just put the laser id on desired bullbar corner position using the ini parameters:
Exemple, to put the hokuyo id 2 on front bullbar left corner, modify the parameter:
```bash
front_bullbar_left_corner_laser_id	2
```
to put the hokuyo id 3 on rear bullbar right corner, modify the parameter with the respective laser_id:
```bash
rear_bullbar_right_corner_laser_id	3
```
Is possible configure the position of laser modifing the parameters
```bash
[rear|front]_bullbar_[right|left]_corner_x	0
[rear|front]_bullbar_[right|left]_corner_y	-0.772
[rear|front]_bullbar_[right|left]_corner_z	0.2
[rear|front]_bullbar_[right|left]_corner_yaw	0.785398
[rear|front]_bullbar_[right|left]_corner_pitch	0.0
[rear|front]_bullbar_[right|left]_corner_roll	0.0
[rear|front]_bullbar_[right|left]_corner_laser_id	3
```
- REMEBER!: the laser are in position 0 relative with the bullbar position, if the bullbar potision change, the laser position also will.
- Whenever a new laser is added, it is necessary to increase the `laser_num_laser_devices` parameter on carmen_ford_escape.ini

### Run

Inside $CARMEN_HOME/bin/ 

Run the Central:
`./central`

run the param server (./param_daemon carmen-ford-escape.ini or process)

run the laser server with id laser parameter:

 `./laser -id 2`

for one laser, for two use:

 `./laser -id 2 3`

Most process already have the option to run laser -id 2, check if is start and the lasers ID.

To visualize the rays use the viewer 3D in options -> SICK or SICK rays
