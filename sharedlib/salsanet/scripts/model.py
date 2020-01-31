# TF SHORTCUTS
import tensorflow as tf
import tensorflow.contrib as tc
conv2d_layer = tc.layers.conv2d
conv2d_trans_layer = tc.layers.conv2d_transpose
leakyRelu = tf.nn.leaky_relu
maxpool_layer = tc.layers.max_pool2d
dropout_layer = tf.layers.dropout
batchnorm = tc.layers.batch_norm

def resBlock(input_layer, filter_nbr, dropout_rate, kernel_size=(3, 3), stride=1, layer_name="rb", training=True,
             pooling=True, repetition=1):
    with tf.variable_scope(layer_name):

        resA = input_layer

        for i in range(repetition):
            shortcut = conv2d_layer(resA, filter_nbr, kernel_size=(1, 1), stride=stride, activation_fn=leakyRelu,
                                    scope=layer_name + '_s_%d' % (i + 0))

            resA = conv2d_layer(resA, filter_nbr, kernel_size, normalizer_fn=batchnorm,
                                activation_fn=leakyRelu,
                                normalizer_params={'is_training': training},
                                scope=layer_name + '_%d_conv1' % (i + 0))

            resA = conv2d_layer(resA, filter_nbr, kernel_size, normalizer_fn=batchnorm,
                                activation_fn=leakyRelu,
                                normalizer_params={'is_training': training},
                                scope=layer_name + '_%d_conv2' % (i + 0))

            resA = tf.add(resA, shortcut)

        if pooling:
            resB = dropout_layer(resA, rate=dropout_rate, name="dropout")
            resB = maxpool_layer(resB, (2, 2), padding='same')

            print(str(layer_name) + str(resB.shape.as_list()))
            return resB, resA
        else:
            resB = dropout_layer(resA, rate=dropout_rate, name="dropout")
            print(str(layer_name) + str(resB.shape.as_list()))
            return resB


def upBlock(input_layer, skip_layer, filter_nbr, dropout_rate, kernel_size=(3, 3), layer_name="dec", training=True):
    with tf.variable_scope(layer_name + "_up"):
        upA = conv2d_trans_layer(input_layer, filter_nbr, kernel_size, 2, normalizer_fn=batchnorm,
                                 activation_fn=leakyRelu,
                                 normalizer_params={'is_training': training}, scope="tconv")
        upA = dropout_layer(upA, rate=dropout_rate, name="dropout")

    with tf.variable_scope(layer_name + "_add"):
        upB = tf.add(upA, skip_layer, name="add")
        upB = dropout_layer(upB, rate=dropout_rate, name="dropout_add")

    with tf.variable_scope(layer_name + "_conv"):
        upE = conv2d_layer(upB, filter_nbr, kernel_size, normalizer_fn=batchnorm,
                           activation_fn=leakyRelu,
                           normalizer_params={'is_training': training}, scope="conv1")
        upE = conv2d_layer(upE, filter_nbr, kernel_size, normalizer_fn=batchnorm,
                           activation_fn=leakyRelu,
                           normalizer_params={'is_training': training}, scope="conv2")
        upE = conv2d_layer(upE, filter_nbr, kernel_size, normalizer_fn=batchnorm,
                           activation_fn=leakyRelu,
                           normalizer_params={'is_training': training}, scope="conv3")
        upE = dropout_layer(upE, rate=dropout_rate, name="dropout_conv")

        print(str(layer_name) + str(upE.shape.as_list()))

        return upE


def create_SalsaNet(input_img, num_classes=3, dropout_rate=0.5, is_training=False, kernel_number=32):

    print ("--------------- SalsaNet model --------------------")
    print("input", input_img.shape.as_list())

    down0c, down0b = resBlock(input_img, filter_nbr=kernel_number, dropout_rate=dropout_rate, kernel_size=3, stride=1, layer_name="res0", training=is_training, repetition=1)
    down1c, down1b = resBlock(down0c, filter_nbr=2 * kernel_number, dropout_rate=dropout_rate, kernel_size=3, stride=1, layer_name="res1", training=is_training, repetition=1)
    down2c, down2b = resBlock(down1c, filter_nbr=4 * kernel_number, dropout_rate=dropout_rate, kernel_size=3, stride=1, layer_name="res2", training=is_training, repetition=1)
    down3c, down3b = resBlock(down2c, filter_nbr=8 * kernel_number, dropout_rate=dropout_rate, kernel_size=3, stride=1, layer_name="res3", training=is_training, repetition=1)
    down4b = resBlock(down3c, filter_nbr=8 * kernel_number, dropout_rate=dropout_rate, kernel_size=3, stride=1, layer_name="res4", training=is_training, pooling=False, repetition=1)

    up3e = upBlock(down4b, down3b,  filter_nbr=8 * kernel_number, dropout_rate=dropout_rate, kernel_size=(3, 3), layer_name="up3", training=is_training)
    up2e = upBlock(up3e, down2b,  filter_nbr=4 * kernel_number, dropout_rate=dropout_rate, kernel_size=(3, 3), layer_name="up2", training=is_training)
    up1e = upBlock(up2e, down1b,  filter_nbr=2 * kernel_number, dropout_rate=dropout_rate, kernel_size=(3, 3), layer_name="up1", training=is_training)
    up0e = upBlock(up1e, down0b,  filter_nbr= kernel_number, dropout_rate=dropout_rate, kernel_size=(3, 3), layer_name="up0", training=is_training)

    with tf.variable_scope('logits'):
        logits = conv2d_layer(up0e, num_classes, [1, 1], activation_fn=None)
        print("logits", logits.shape.as_list())

    return logits
    print ("--------------------------------------------------")
