echo "Test Weights BN" && date
$CARMEN_HOME/sharedlib/ENet/caffe-enet/build/tools/caffe test \
   -model $CARMEN_HOME/src/road_mapper/data/bn_conv_merged_model_test.prototxt \
   -weights $CARMEN_HOME/src/road_mapper/data/bn_conv_merged_weights.caffemodel \
   -iterations 1386 -gpu 0 &> $CARMEN_HOME/sharedlib/ENet/results/results_test_weights_bn2test.txt
echo "Train Weights BN" && date
$CARMEN_HOME/sharedlib/ENet/caffe-enet/build/tools/caffe test \
   -model $CARMEN_HOME/src/road_mapper/data/bn_conv_merged_model_train.prototxt \
   -weights $CARMEN_HOME/src/road_mapper/data/bn_conv_merged_weights.caffemodel \
   -iterations 5523 -gpu 0 &> $CARMEN_HOME/sharedlib/ENet/results/results_test_weights_bn2train.txt
echo "Test Weights Encoder" && date
$CARMEN_HOME/sharedlib/ENet/caffe-enet/build/tools/caffe test \
   -model $CARMEN_HOME/sharedlib/ENet/prototxts/enet_test_encoder.prototxt \
   -weights $CARMEN_HOME/sharedlib/ENet/weights/snapshots_encoder/enet_iter_10605.caffemodel \
   -iterations 1386 -gpu 0 &> $CARMEN_HOME/sharedlib/ENet/results/results_test_weights_encoder2test.txt
echo "Train Weights Encoder" && date
$CARMEN_HOME/sharedlib/ENet/caffe-enet/build/tools/caffe test \
   -model $CARMEN_HOME/sharedlib/ENet/prototxts/enet_train_encoder2.prototxt \
   -weights $CARMEN_HOME/sharedlib/ENet/weights/snapshots_encoder/enet_iter_10605.caffemodel \
   -iterations 5523 -gpu 0 &> $CARMEN_HOME/sharedlib/ENet/results/results_test_weights_encoder2train.txt
echo "Test Weights Decoder" && date
$CARMEN_HOME/sharedlib/ENet/caffe-enet/build/tools/caffe test \
   -model $CARMEN_HOME/sharedlib/ENet/prototxts/enet_test_encoder_decoder.prototxt \
   -weights $CARMEN_HOME/sharedlib/ENet/weights/snapshots_decoder/enet_iter_15150.caffemodel \
   -iterations 1386 -gpu 0 &> $CARMEN_HOME/sharedlib/ENet/results/results_test_weights_decoder2test.txt
echo "Train Weights Decoder" && date
$CARMEN_HOME/sharedlib/ENet/caffe-enet/build/tools/caffe test \
   -model $CARMEN_HOME/sharedlib/ENet/prototxts/enet_train_encoder_decoder2.prototxt \
   -weights $CARMEN_HOME/sharedlib/ENet/weights/snapshots_decoder/enet_iter_15150.caffemodel \
   -iterations 5523 -gpu 0 &> $CARMEN_HOME/sharedlib/ENet/results/results_test_weights_decoder2train.txt
echo "Test Weights BN (Batch = 1)" && date
$CARMEN_HOME/sharedlib/ENet/caffe-enet/build/tools/caffe test \
   -model $CARMEN_HOME/src/road_mapper/data/bn_conv_merged_model_test2.prototxt \
   -weights $CARMEN_HOME/src/road_mapper/data/bn_conv_merged_weights.caffemodel \
   -iterations 22176 -gpu 0 &> $CARMEN_HOME/sharedlib/ENet/results/results_test_weights_bn2test_batch1.txt
echo "Test Weights BN (Guarapari)" && date
$CARMEN_HOME/sharedlib/ENet/caffe-enet/build/tools/caffe test \
   -model $CARMEN_HOME/src/road_mapper/data/bn_conv_merged_model_test_guarapari2.prototxt \
   -weights $CARMEN_HOME/src/road_mapper/data/bn_conv_merged_weights.caffemodel \
   -iterations 1239 -gpu 0 &> $CARMEN_HOME/sharedlib/ENet/results/results_test_weights_bn2test_guarapari2.txt
echo "Test Weights BN (Guarapari full)" && date
$CARMEN_HOME/sharedlib/ENet/caffe-enet/build/tools/caffe test \
   -model $CARMEN_HOME/src/road_mapper/data/bn_conv_merged_model_test_guarapari3.prototxt \
   -weights $CARMEN_HOME/src/road_mapper/data/bn_conv_merged_weights.caffemodel \
   -iterations 222 -gpu 0 &> $CARMEN_HOME/sharedlib/ENet/results/results_test_weights_bn2test_guarapari3.txt
echo "Test Weights BN (Guarapari full Batch = 1)" && date
$CARMEN_HOME/sharedlib/ENet/caffe-enet/build/tools/caffe test \
   -model $CARMEN_HOME/src/road_mapper/data/bn_conv_merged_model_test_guarapari3_batch1.prototxt \
   -weights $CARMEN_HOME/src/road_mapper/data/bn_conv_merged_weights.caffemodel \
   -iterations 3556 -gpu 0 &> $CARMEN_HOME/sharedlib/ENet/results/results_test_weights_bn2test_guarapari3_batch1.txt
echo "Finish" && date

