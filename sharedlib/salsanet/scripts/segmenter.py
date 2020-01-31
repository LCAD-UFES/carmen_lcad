import tensorflow as tf
import os
import time
from datetime import datetime
from utils import *
from model import *
import numpy as np 

# ##############################################################################
# SEGMENTATION CLASS
# ##############################################################################
class SegmenterNet(object):
    def __init__(self, cfg, model_ckp_name=""):
        """ Initializes a Segmentation Model Class """

        # MODEL SETTINGS
        self.cfg = cfg

        if model_ckp_name is not "":
            self.checkpoint_file = model_ckp_name
            self.log_dir =  self.cfg.log_path + "inference_results"
            if not os.path.exists(self.log_dir):
                os.makedirs(self.log_dir)
        else:
            # DIRECTORIES TO STORE OUTPUTS
            if not self.cfg.log_name =="":
                self.log_dir = self.cfg.log_path + "{}".format(datetime.now().strftime("%Y%m%d-%H%M%S")) + "_" + self.cfg.log_name
            else:
                self.log_dir = self.cfg.log_path + "{}".format(datetime.now().strftime("%Y%m%d-%H%M%S"))
            if not os.path.exists(self.log_dir):
                os.makedirs(self.log_dir)
                self.checkpoint_file = os.path.join(self.log_dir , "model.chk")
            else:
                self.checkpoint_file = os.path.join(self.log_dir , "model.chk")

        # Create log file
        log_filename = os.path.join(self.log_dir, "net_parameters.txt")
        self.log_file = open(log_filename, 'w+')

        self.init_network_model()

    def init_network_model(self):
        self.graph = tf.Graph()
        with self.graph.as_default():
            self.create_input_ops()

            self.logits = create_SalsaNet(self.input_img, self.cfg.NUM_CLASS,dropout_rate=self.cfg.DROPOUT_PROB, is_training=self.is_training)

            self.store_network_parameters()

            with tf.name_scope("preds") as scope:
                self.preds = tf.to_int32(tf.argmax(self.logits, axis=-1), name=scope)

            self.create_loss_ops()
            self.create_optimization_ops()
            self.create_evaluation_metric_ops()
            self.create_summary_ops()

    def create_input_ops(self):
        with tf.variable_scope("parameters"):
            input_img_shape = (None, self.cfg.IMAGE_HEIGHT, self.cfg.IMAGE_WIDTH, self.cfg.IMAGE_CHANNEL)
            output_img_shape = (None, self.cfg.IMAGE_HEIGHT, self.cfg.IMAGE_WIDTH)
            self.input_img = tf.placeholder(tf.float32, shape=input_img_shape, name="input_img")
            self.output_img = tf.placeholder(tf.int32, shape=output_img_shape, name="output_img")
            self.weight_img = tf.placeholder(tf.float32, shape=output_img_shape, name="weight_img")
            self.dropout = tf.placeholder_with_default(self.cfg.DROPOUT_PROB, shape=None, name="dropout")
            self.is_training = tf.placeholder_with_default(False, shape=(), name="is_training")

            self.global_step = tf.Variable(0, name='global_step', trainable=False)
            self.learning_rate = tf.train.exponential_decay(learning_rate=self.cfg.LEARNING_RATE,
                                            global_step=self.global_step,
                                            decay_steps=self.cfg.LR_DECAY_CYCLE,
                                            decay_rate=self.cfg.LR_DECAY_FACTOR,
                                            staircase=True,
                                            name="learningrate")

            # Create a summary to monitor the learning rate
            tf.summary.scalar("learning_rate", self.learning_rate)

    def store_network_parameters(self):

        self.log_file.write("\n" +  ("#" * 70) + "\n" + ("#" * 29)  + " parameters " + ("#" * 29) + "\n" + ("#" * 70) + "\n")

        for k, v in sorted(self.cfg.items()):
            text_to_write = str(k) + " : " + str(v) + "\n"
            self.log_file.write(text_to_write)

        self.log_file.write("\n" +  ("#" * 70) +  "\n" + ("#" * 70) + "\n")
        self.log_file.close()

    def create_loss_ops(self):
        # LOSS FUNCTION
        with tf.variable_scope('loss') as scope:
            unrolled_logits = tf.reshape(self.logits, (-1, self.cfg.NUM_CLASS))
            unrolled_labels = tf.reshape(self.output_img, (-1,))
            unrolled_weights = tf.reshape(self.weight_img, (-1,))
            cross_entropy = tf.losses.sparse_softmax_cross_entropy(labels=unrolled_labels, logits=unrolled_logits, weights=unrolled_weights)
            self.loss  = tf.reduce_mean(cross_entropy)

            # Create a summary to monitor the loss
            tf.summary.scalar("loss", self.loss)

    def create_optimization_ops(self):
        # OPTIMIZATION METHOD
        with tf.variable_scope('opt') as scope:
            self.optimizer = tf.train.AdamOptimizer(self.learning_rate, name="optimizer")
            update_ops = tf.get_collection(tf.GraphKeys.UPDATE_OPS)
            with tf.control_dependencies(update_ops):
                self.train_op = self.optimizer.minimize(self.loss, global_step=self.global_step, name="train_op")

    def create_evaluation_metric_ops(self):
        # EVALUATION METRIC - Intersection over Union IoU
        with tf.name_scope("evaluation") as scope:
            # Define the evaluation metric and update operations
            self.evaluation, self.update_evaluation_vars = tf.metrics.mean_iou(
                tf.reshape(self.output_img, [-1]),
                tf.reshape(self.preds, [-1]),
                num_classes=self.cfg.NUM_CLASS,
                name=scope)
            # Isolate metric's running variables & create their initializer and reset operators
            evaluation_vars = tf.get_collection(tf.GraphKeys.LOCAL_VARIABLES, scope=scope)
            self.reset_evaluation_vars = tf.variables_initializer(var_list=evaluation_vars)

    def create_summary_ops(self):
        with tf.name_scope('summary'):
            self.summary_writer = tf.summary.FileWriter(self.log_dir, graph=self.graph)
            self.saver = tf.train.Saver(tf.global_variables(),name="saver", max_to_keep=1)
            self.summary_op = tf.summary.merge_all()

    def train_segmenter(self, training_data_path, validation_data_path):

        with tf.Session(graph=self.graph) as sess:
            self.initialize_vars(sess)

            for epoch in range(1, self.cfg.NUM_EPOCHS+1):
                timeStart = time.time()

                # generate batches
                training_batches, n_training_samples = generate_lidar_batch_function(training_data_path, channel_nbr=self.cfg.IMAGE_CHANNEL, class_nbr= self.cfg.NUM_CLASS, loss_weights=self.cfg.CLS_LOSS_WEIGHTS, augmentation=self.cfg.DATA_AUGMENTATION)
                validation_batches, n_validation_samples = generate_lidar_batch_function(validation_data_path, channel_nbr=self.cfg.IMAGE_CHANNEL, class_nbr= self.cfg.NUM_CLASS, loss_weights=self.cfg.CLS_LOSS_WEIGHTS, augmentation=self.cfg.DATA_AUGMENTATION)

                # Num batches per epoch
                n_batches = int(np.ceil(n_training_samples / self.cfg.BATCH_SIZE))

                # Iterate through each mini-batch
                for step in range(n_batches):

                    # get next batch data
                    X_batch, Y_batch, W_batch = next(training_batches(self.cfg.BATCH_SIZE))

                    if self.cfg.DEBUG_MODE:
                        print('X_batch', X_batch.shape, X_batch.dtype, X_batch.min(), X_batch.max())
                        print('Y_batch', Y_batch.shape, Y_batch.dtype, Y_batch.min(), Y_batch.max())
                        print('W_batch', W_batch.shape, W_batch.dtype, W_batch.min(), W_batch.max()) 

                    # Runtime metadata
                    run_options = tf.RunOptions(trace_level=tf.RunOptions.FULL_TRACE)
                    run_metadata = tf.RunMetadata()

                    # Train
                    feed_dict = {self.input_img:X_batch, self.output_img:Y_batch, self.weight_img:W_batch, self.is_training:True}
                    loss, _, summary = sess.run([self.loss, self.train_op, self.summary_op], feed_dict=feed_dict, options=run_options, run_metadata=run_metadata)

                    tag_name= 'epoch {} step {}'.format(epoch, step)
                    self.summary_writer.add_summary(summary, n_batches*(epoch-1)+step)

                    # force tensorflow to synchronise summaries
                    self.summary_writer.flush()

                    # Print feedback every so often
                    if self.cfg.PRINT_EVERY is not None and (step+1)%self.cfg.PRINT_EVERY==0:
                        timeElapsed = time.time() - timeStart
                        print(" EPOCH {}/{} step: {: 5d} Batch loss: {:3.5f} Time avg: {:3.5f} sec".format(epoch, self.cfg.NUM_EPOCHS, step+1, loss, timeElapsed/self.cfg.PRINT_EVERY))
                        timeStart = time.time()

                # Evaluate on train and validation sets after each epoch
                train_iou, train_loss, train_ious, train_precs, train_recalls = self.evaluate(training_batches, n_training_samples, sess)
                valid_iou, valid_loss, valid_ious, valid_precs, valid_recalls = self.evaluate(validation_batches, n_validation_samples, sess)

                # print scores
                self.print_evaluation_scores(train_iou, train_loss, train_ious, train_precs, train_recalls, tag="Training")
                self.print_evaluation_scores(valid_iou, valid_loss, valid_ious, valid_precs, valid_recalls, tag="Validation")

                # keep summary data after each epoch
                self.save_summaries(sess, train_loss, train_iou, valid_loss, valid_iou, train_ious, train_precs, train_recalls, valid_ious, valid_precs, valid_recalls, epoch)

    def initialize_vars(self, session):
        if tf.train.checkpoint_exists(self.checkpoint_file):
            print("- Restoring parameters from saved checkpoints")
            print("  -", self.checkpoint_file)
            self.saver.restore(session, self.checkpoint_file)
        else:
            print("Initializing weights to random values")
            session.run(tf.global_variables_initializer())

    def predict(self, batch_data, session):
        # MAKE PREDICTIONS ON SINGLE BATCH DATA
        feed_dict = {self.input_img:batch_data, self.is_training:False}
        batch_preds = session.run(self.preds, feed_dict=feed_dict)
        preds = batch_preds.squeeze()

        return preds

    def predict_single_image(self, input_img, session):
        # MAKE PREDICTIONS ON SINGLE IMAGE DATA

        # expand image dimension
        temp_img = np.zeros((1, input_img.shape[0], input_img.shape[1], input_img.shape[2]))
        temp_img[0, :, :, :] = input_img

        # MAKE PREDICTIONS ON SINGLE IMAGE
        feed_dict = {self.input_img: temp_img, self.is_training: False}
        timeStart = time.time()
        pred_img = session.run(self.preds, feed_dict=feed_dict)
        timeElapsed = (time.time() - timeStart)*1000.0
        print("predict_single_image took : {:3.5f} msec".format(timeElapsed))

        return pred_img[0]

    def evaluate(self, batch_data, data_size, session):
        # EVALUATE ON BATCH DATA
        total_loss = 0
        tps = []
        fps = []
        fns = []
        n_samples = data_size
        n_batches = int(np.ceil(n_samples/self.cfg.BATCH_SIZE)) # Num batches needed

        # Reset the running variables for evaluation metric
        session.run(self.reset_evaluation_vars)

        # Iterate through each mini-batch
        for step in range(n_batches):
            # get next batch data
            X_batch, Y_batch, W_batch = next(batch_data(self.cfg.BATCH_SIZE))
            feed_dict = {self.input_img:X_batch, self.output_img:Y_batch, self.weight_img:W_batch, self.is_training:False}

            # Get loss, and update running variables for evaluation metric
            loss, preds, confusion_mtx = session.run([self.loss, self.preds, self.update_evaluation_vars], feed_dict=feed_dict)
            total_loss += loss

            #iou computation
            tp, fp, fn = self.evaluate_iou(Y_batch, preds, self.cfg.NUM_CLASS)
            tps.append(tp)
            fps.append(fp)
            fns.append(fn)


        tps = np.array(tps)
        fps = np.array(fps)
        fns = np.array(fns)
        epsilon = 1e-12
        ious = tps.astype(np.float) / (tps + fns + fps + epsilon)
        precision = tps.astype(np.float) / (tps + fps + epsilon)
        recall = tps.astype(np.float) / (tps + fns + epsilon)

        mean_ious = np.mean(ious, axis=0)
        mean_prec = np.mean(precision, axis=0)
        mean_recall = np.mean(recall, axis=0)

        # Get the updated score from the running metric
        score = session.run(self.evaluation)
        # Average the loss
        avg_loss = total_loss/float(n_batches)

        return score, avg_loss, mean_ious, mean_prec, mean_recall

    def evaluate_iou(self, label, pred, n_class):

        assert label.shape == pred.shape, \
            'label and pred shape mismatch: {} vs {}'.format(
                label.shape, pred.shape)

        tps = np.zeros(n_class)
        fns = np.zeros(n_class)
        fps = np.zeros(n_class)

        for cls_id in range(n_class):
            tp = np.sum(pred[label == cls_id] == cls_id)
            fp = np.sum(label[pred == cls_id] != cls_id)
            fn = np.sum(pred[label == cls_id] != cls_id)

            tps[cls_id] = tp
            fps[cls_id] = fp
            fns[cls_id] = fn

        return tps, fps, fns

    def expand_image_dimension(self, input_img):
        # return pred results as colored rgb images
        n_samples = input_img.shape[0]
        output_img = np.zeros((n_samples, input_img.shape[1], input_img.shape[2], 3))
        for i in range(0,n_samples):
            label_map = input_img[i,:,:]
            color_img = np.zeros((input_img.shape[1], input_img.shape[2], 3))
            for j in range(0,self.cfg.NUM_CLASS):
                color_img[label_map==j,:] = self.cfg.CLS_COLOR_MAP[j]

            output_img[i,:,:,:] =color_img

        return output_img

    def save_summaries(self, sess, train_loss, train_iou, valid_loss, valid_iou, train_mean_ious, train_precs, train_recalls, valid_mean_ious, valid_precs, valid_recalls, epoch):

        # Save checkpoints
        self.saver.save(sess, self.checkpoint_file, global_step=epoch, write_meta_graph=True)

        # Save training and validation summaries
        summary = tf.Summary()
        summary.value.add(tag='Training/Training Loss', simple_value=float(train_loss))
        summary.value.add(tag='Validation/Validation Loss', simple_value=float(valid_loss))
        summary.value.add(tag='Training/Training IOU', simple_value=float(train_iou))
        summary.value.add(tag='Validation/Validation IOU', simple_value=float(valid_iou))


        for i in range(0,self.cfg.NUM_CLASS):
            tag_name = 'Training/' + self.cfg.CLASSES[i] + '/IOU'
            summary.value.add(tag=tag_name, simple_value=float(train_mean_ious[i]))
            tag_name = 'Training/' + self.cfg.CLASSES[i] + '/Prec'
            summary.value.add(tag=tag_name, simple_value=float(train_precs[i]))
            tag_name = 'Training/' + self.cfg.CLASSES[i] + '/Recall'
            summary.value.add(tag=tag_name, simple_value=float(train_recalls[i]))

            tag_name = 'Validation/' + self.cfg.CLASSES[i] + '/IOU'
            summary.value.add(tag=tag_name, simple_value=float(valid_mean_ious[i]))
            tag_name = 'Validation/' + self.cfg.CLASSES[i] + '/Prec'
            summary.value.add(tag=tag_name, simple_value=float(valid_precs[i]))
            tag_name = 'Validation/' + self.cfg.CLASSES[i] + '/Recall'
            summary.value.add(tag=tag_name, simple_value=float(valid_recalls[i]))


        self.summary_writer.add_summary(summary, epoch)

        # force tensorflow to synchronise summaries
        self.summary_writer.flush()

    def print_evaluation_scores(self, iou, loss, ious, precs, recalls, tag="Training"):

        if tag=="Training":
            s = "TR IOU: {: 3.3f} TR IOU: {: 3.3f} TR LOSS: {: 3.5f} "
        else:
            s = "VR IOU: {: 3.3f} VR IOU: {: 3.3f} VR LOSS: {: 3.5f} "
        print(s.format(iou, np.mean(ious), loss))

        for i in range(0,self.cfg.NUM_CLASS):
            s = self.cfg.CLASSES[i] + " PREC: {: 3.3f} " + self.cfg.CLASSES[i] + " REC: {: 3.3f} " + self.cfg.CLASSES[i] + " IOU: {: 3.3f}"
            print(s.format(precs[i], recalls[i], ious[i]))
