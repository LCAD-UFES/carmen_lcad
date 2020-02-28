# LibInplace_ABN

## Spline generator based on images

* make libinplace_abn - it will make libinplace_abn.a
```
$ cd $CARMEN_HOME/sharedlib/inplace_abn/scripts/ 
:~/carmen_lcad/sharedlib/inplace_abn/scripts$ make
```

* make rddf_from_image
```
$ cd $CARMEN_HOME/src/rddf_from_image
:~/carmen_lcad/src/rddf_from_image$ make
```

## Installing dependencies

* Create virtualenv
```
$ cd $CARMEN_HOME/sharedlib/inplace_abn/ 
:~/carmen_lcad/sharedlib/inplace_abn$ ./create_env.sh
```

* Download original inplace_abn and install on virtual environment
```
$ cd ~
$ git clone https://github.com/mapillary/inplace_abn.git
$ cd inplace_abn
~/inplace_abn$ source $CARMEN_HOME/sharedlib/inplace_abn/inplace_env/bin/activate
(inplace_env) ~/inplace_abn$ python setup.py install
```

### Using inplace_abn with carmen:

Copy the weights for $CARMEN_HOME/sharedlib/inplace_abn/.

##### Run Central
```
:~/carmen_lcad/bin$ ./central
```

##### Run some proccontrol
```
:~/carmen_lcad/bin$ ./proccontrol process-volta_da_ufes.ini
```

##### Run rddf_from_image:
```
:~/carmen_lcad/bin$ ./rddf_from_image 3
```

Push Playback button. The results from rddf generator will be at navigator.
