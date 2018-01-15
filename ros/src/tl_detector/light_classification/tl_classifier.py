from styx_msgs.msg import TrafficLight
import cv2
import rospkg
import numpy as np
import tensorflow as tf
from keras.models import load_model

class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        ros_path = rospkg.get_ros_root()
        rospack = rospkg.RosPack()
        rospath = rospack.get_path('tl_detector')
        print("Current path: " +rospath)
        self.model = load_model(rospath + '/prototype_traffic_light_classifier.h5')
        self.model._make_predict_function()
        self.graph = tf.get_default_graph()

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction
        
        img = cv2.resize(src=image, dsize=(400,400))
        img = img.astype(float)
        img = img / 255.0

        img = img[np.newaxis,:,:,:]

        with self.graph.as_default():
            predictions = self.model.predict(img)
            predicted_cat = np.argmax(predictions,axis=1)

            print('Predicted state: ', predicted_cat[0])
            light = predicted_cat[0]

            if(light==1):
                return TrafficLight.RED
        return TrafficLight.UNKNOWN
