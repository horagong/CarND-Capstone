from styx_msgs.msg import TrafficLight
import os
import tensorflow as tf
import numpy as np
import rospy
#import visualization_utils as vis_util

class TLClassifier(object):
    def __init__(self):
        #DONE load classifier
        rospy.logdebug('light_classifier: %s', os.getcwd())
        path_to_ckpt = os.path.join('..', 'data', 'udacity_frozen_inference_graph.pb')
        #self.count = 0

        detection_graph = tf.Graph()
        config = tf.ConfigProto()
        config.gpu_options.allow_growth = True
        # for JIT optimization
        config.graph_options.optimizer_options.global_jit_level = tf.OptimizerOptions.ON_1
        with detection_graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(path_to_ckpt, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')

            self.sess = tf.Session(graph=detection_graph, config=config) 
            # Definite input and output Tensors for detection_graph
            self.image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')
            # Each box represents a part of the image where a particular object was detected.
            self.detection_boxes = detection_graph.get_tensor_by_name('detection_boxes:0')
            # Each score represent how level of confidence for each of the objects.
            # Score is shown on the result image, together with the class label.
            self.detection_scores = detection_graph.get_tensor_by_name('detection_scores:0')
            self.detection_classes = detection_graph.get_tensor_by_name('detection_classes:0')
            self.num_detections = detection_graph.get_tensor_by_name('num_detections:0')

    def class_to_ID(self, cls):
        if cls == 2:
            return TrafficLight.RED
        elif cls == 3:
            return TrafficLight.YELLOW
        elif cls == 4:
            return TrafficLight.GREEN
        else:
            return TrafficLight.UNKNOWN

    def get_classification(self, image_np):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #DONE implement light color prediction
        #image = Image.open(image_path)
        # the array based representation of the image will be used later in order to prepare the
        # result image with boxes and labels on it.
        #image_np = self.load_image_into_numpy_array(image)
        # Expand dimensions since the model expects images to have shape: [1, None, None, 3]
        #cv2.imwrite('{}.png'.format(self.count), image_np)

        image_np_expanded = np.expand_dims(image_np, axis=0)
        # Actual detection.
        (boxes, scores, classes, num) = self.sess.run(
          [self.detection_boxes, self.detection_scores, self.detection_classes, self.num_detections],
          feed_dict={self.image_tensor: image_np_expanded})
        '''
        # Visualization of the results of a detection.
        vis_util.visualize_boxes_and_labels_on_image_array(
          image_np,
          np.squeeze(boxes),
          np.squeeze(classes).astype(np.int32),
          np.squeeze(scores),
          {1: {'id': 1, 'name': u'None'}, 2: {'id': 2, 'name': u'Red'}
            , 3: {'id': 3, 'name': u'Yellow'}, 4: {'id': 4, 'name': u'Green'}},
          use_normalized_coordinates=True,
          line_thickness=8)
        cv2.imwrite('{}_XXX_{}.png'.format(self.count, np.squeeze(classes)[0]), image_np)
        self.count += 1
        '''
        ID = self.class_to_ID(np.squeeze(classes)[0])
        score = np.squeeze(scores)[0]
        #rospy.loginfo('light_classifier: color=%s, prob=%s', ID, score)

        if score > 0.5:
            return ID

        return TrafficLight.UNKNOWN
