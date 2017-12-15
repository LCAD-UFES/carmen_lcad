# Results of ENet Caffe Training

## Usage

Before start the training, save the solver parameters: 
```bash
 $ cd $CARMEN_HOME/sharedlib/ENet/results
 $ ./save_train_params.sh ../prototxts/enet_solver_encoder.prototxt

```
 $ File parameters_<timestamp>.txt saved. 

Start ENet Caffe training procedure immune to hangup signal: 
```bash
 $ cd $CARMEN_HOME/sharedlib/ENet/results
 $ nohup ../caffe-enet/build/tools/caffe train -solver ../prototxts/enet_solver_encoder.prototxt &> results_<timestamp>.txt &

```

Monitor ENet Caffe training running: 
```bash
 $ cd $CARMEN_HOME/sharedlib/ENet/results
 $ ./show_last_iteration.sh results_<timestamp>.txt

```

Plot ENet Caffe training accuracy: 
```bash
 $ cd $CARMEN_HOME/sharedlib/ENet/results
 $ ./plot_accuracy.sh results_<timestamp>.txt

```

