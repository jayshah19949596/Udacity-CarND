from styx_msgs.msg import TrafficLight
import tensorflow as tf
import numpy
import os
import cv2
import rospy

MAX_IMAGE_WIDTH = 300
MAX_IMAGE_HEIGHT = 300
RECORD_IMAGES = False

class TLClassifier(object):
    def __init__(self):
        self.ModelGraph = None
        self.TFSession = None
        self.TrafficLights = {1: TrafficLight.RED,
                              2: TrafficLight.YELLOW,
                              3: TrafficLight.GREEN,
                              4: TrafficLight.UNKNOWN}

        PathToTheModel = os.path.dirname(os.path.realpath(__file__)) + "/frozen_graph.pb"

        NetWorkConfig = tf.ConfigProto()
        NetWorkConfig.graph_options.optimizer_options.global_jit_level = tf.OptimizerOptions.ON_1

        self.ModelGraph = tf.Graph()
        with tf.Session(graph=self.ModelGraph, config=NetWorkConfig) as sess:
            self.TFsession = sess
            GraphDef = tf.GraphDef()
            with tf.gfile.GFile(PathToTheModel, 'rb') as fid:
                SerializedGraph = fid.read()
                GraphDef.ParseFromString(SerializedGraph)
                tf.import_graph_def(GraphDef, name='')


    def get_classification(self, image):
        """Determines the color of the traffic light in the image
        Args:
            image (cv::Mat): image containing the traffic light
        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """
        TrafficLightIndex, Probability = self.Predict(image)

        if TrafficLightIndex is not None:
            rospy.logdebug("class: %d, probability: %f", TrafficLightIndex, Probability)

        return TrafficLightIndex

 

    def Predict(self, CapturedImage, MinScore=0.5):
        image_tensor = self.ModelGraph.get_tensor_by_name('image_tensor:0')
        detection_boxes = self.ModelGraph.get_tensor_by_name('detection_boxes:0')
        detection_scores = self.ModelGraph.get_tensor_by_name('detection_scores:0')
        detection_classes = self.ModelGraph.get_tensor_by_name('detection_classes:0')

        CapturedImage = cv2.resize(CapturedImage, (MAX_IMAGE_WIDTH, MAX_IMAGE_HEIGHT))
        CapturedImage = cv2.cvtColor(CapturedImage, cv2.COLOR_BGR2RGB)
        
        (Boxes, Scores, Classes) = self.TFSession.run(
            [detection_boxes, detection_scores, detection_classes],
            feed_dict={image_tensor: numpy.expand_dims(CapturedImage, axis=0)})

        Scores = numpy.squeeze(Scores)
        Classes = numpy.squeeze(Classes)
        Boxes = numpy.squeeze(Boxes)

        for i, box in enumerate(Boxes):
            if Scores[i] > MinScore:
                light_class = self.TrafficLights[Classes[i]]
                rospy.logdebug("Traffic Light Class detected: %d", light_class)
                return light_class, Scores[i]

        return None, None