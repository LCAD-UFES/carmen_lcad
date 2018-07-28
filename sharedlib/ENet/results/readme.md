# Results of ENet Caffe Training

## Usage

Before start the training, save the solver parameters: 
```bash
 $ cd $CARMEN_HOME/sharedlib/ENet/results
 $ # Before the encoder training:
 $ ./save_train_params.sh ../prototxts/enet_solver_encoder.prototxt
 $ # Before the encoder+decoder training:
 $ ./save_train_params.sh ../prototxts/enet_solver_encoder_decoder.prototxt

 Expected output:
 File parameters_<timestamp>.txt saved. 
 Suggested output log filename: results_<timestamp>.txt 

```

Start ENet Caffe training procedure immune to hangup signal: 
```bash
 $ cd $CARMEN_HOME/sharedlib/ENet/results
 $ # Start the encoder training:
 $ nohup env GLOG_minloglevel=0 ../caffe-enet/build/tools/caffe train -solver ../prototxts/enet_solver_encoder.prototxt -gpu 0 &> results_<timestamp>.txt &
 $ # Start the encoder+decoder training (after the encoder training is finished):
 $ nohup env GLOG_minloglevel=0 ../caffe-enet/build/tools/caffe train -solver ../prototxts/enet_solver_encoder_decoder.prototxt -weights ../weights/snapshots_encoder/<NAME>.caffemodel -gpu 0 &> results_<timestamp>.txt &

```

If the training was manually stopped (Ctrl+C or kill -s SIGINT <pid>) or reached the maximum number of iterations or the system crashed, the training may be resumed using any solverstate file: 
```bash
 $ cd $CARMEN_HOME/sharedlib/ENet/results
 $ # To resume the encoder training:
 $ nohup env GLOG_minloglevel=0 ../caffe-enet/build/tools/caffe train -solver ../prototxts/enet_solver_encoder.prototxt -snapshot ../weights/snapshots_encoder/<NAME>.solverstate -gpu 0 &> results_<timestamp>_<NAME>.txt &
 $ # To resume the encoder+decoder training:
 $ nohup env GLOG_minloglevel=0 ../caffe-enet/build/tools/caffe train -solver ../prototxts/enet_solver_encoder_decoder.prototxt -snapshot ../weights/snapshots_decoder/<NAME>.solverstate -gpu 0 &> results_<timestamp>_<NAME>.txt &

```

Monitor ENet Caffe training progress during the procedure or afterwards: 
```bash
 $ cd $CARMEN_HOME/sharedlib/ENet/results
 $ ./show_last_iteration.sh results_<timestamp>.txt
 
 Expected output:
 I1214 17:18:08.072273 33879 solver.cpp:337] Iteration 0, Testing net (#0)
 I1215 19:08:36.788460 33879 solver.cpp:228] Iteration 8880, loss = 0.36872
 I1215 19:08:36.788607 33879 solver.cpp:244]     Train net output #0: accuracy = 0.822133
 I1215 19:08:36.788635 33879 solver.cpp:244]     Train net output #1: loss = 0.36872 (* 1 = 0.36872 loss)
 I1215 19:12:03.510736 33879 solver.cpp:228] Iteration 8900, loss = 0.367749
 I1215 19:12:03.510918 33879 solver.cpp:244]     Train net output #0: accuracy = 0.822356
 I1215 19:12:03.510946 33879 solver.cpp:244]     Train net output #1: loss = 0.367749 (* 1 = 0.367749 loss)
 I1215 19:15:31.435148 33879 solver.cpp:228] Iteration 8920, loss = 0.381509
 I1215 19:15:31.435283 33879 solver.cpp:244]     Train net output #0: accuracy = 0.819111
 I1215 19:15:31.435308 33879 solver.cpp:244]     Train net output #1: loss = 0.381509 (* 1 = 0.381509 loss)
 I1215 19:18:57.878795 33879 solver.cpp:228] Iteration 8940, loss = 0.370622
 I1215 19:18:57.879169 33879 solver.cpp:244]     Train net output #0: accuracy = 0.819956
 I1215 19:18:57.879230 33879 solver.cpp:244]     Train net output #1: loss = 0.370622 (* 1 = 0.370622 loss)
 I1215 19:18:57.879573 33879 sgd_solver.cpp:106] Iteration 8940, lr = 5e-05
 max_iter: 10080

```

Plot ENet Caffe training accuracy during the procedure or afterwards: 
```bash
 $ cd $CARMEN_HOME/sharedlib/ENet/results
 $ ./plot_accuracy.sh results_<timestamp>.txt
 
 Expected output:

```
 ![sample chart](sample_plot_image.jpg)
