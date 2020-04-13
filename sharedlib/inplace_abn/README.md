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

Weights: https://drive.google.com/file/d/1SJJx5-LFG3J3M99TrPMU-z6ZmgWynxo-/view

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

#### Run tests for generating predictions:

First, use png images. Use the code present in $CARMEN_HOME/src/utilities/convert_log_images

```
:/dados$ grep BUMB log.txt > log_filtrado.txt
```

```
:~/carmen_lcad/bin$ cd $CARMEN_HOME/src/utilities/convert_log_images
:~/carmen_lcad/src/utilities/convert_log_images$ make
:~/carmen_lcad/src/utilities/convert_log_images$ ./to_png_new_log /dados/log_ufes_aeroporto_filtrado.txt /dados/png_log_ufes_aeroporto-20200325 -path /dados/log_ufes_aeroporto-20200325.txt
```

Use the neural network for generate the predictions:
```
python test_vistas.py /path/to/model.pth.tar /dados/png_log_ufes_aeroporto-20200325 /path/to/output/folder --output-mode palette
```

--output-mode:
```
-- palette: color coded predictions
-- raw: gray-scale predictions
-- prob: gray-scale predictions plus probabilities
```



