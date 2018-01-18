from styx_msgs.msg import TrafficLight
import rospy
import cv2
import rospkg
import numpy as np
import tensorflow as tf

import keras
from keras.preprocessing.image import ImageDataGenerator
from keras.callbacks import EarlyStopping, ModelCheckpoint
from keras.optimizers import SGD
from keras.optimizers import Adam
from keras.optimizers import RMSprop
from keras.models import load_model, Sequential
from keras.preprocessing.image import load_img, img_to_array
from keras.models import Model
from keras.layers import Input, Activation, merge
from keras.layers import Flatten, Dropout, Dense
from keras.layers import Convolution2D, MaxPooling2D
from keras.layers import AveragePooling2D
from keras.optimizers import Adam
import keras.backend as K
K.set_image_dim_ordering('tf')


IN_IMAGE_WIDTH  = 400
IN_IMAGE_HEIGHT = 300
class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        ros_path = rospkg.get_ros_root()
        rospack = rospkg.RosPack()
        rospath = rospack.get_path('tl_detector')
        print("Current path: " +rospath)
        self.model = load_model(rospath + '/250-resnet-18.h5')
        print("Model successfully loaded!")
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
        
        img = cv2.resize(src=image, dsize=(IN_IMAGE_HEIGHT,IN_IMAGE_WIDTH))
        img = img.astype(float)
        img = img / 255.0

        img = img[np.newaxis,:,:,:]

        with self.graph.as_default():
            predictions = self.model.predict(img)
            predicted_cat = np.argmax(predictions,axis=1)

            light = predicted_cat[0]
            rospy.logwarn("Predicted = %i ", light)
            if(light==0):
                return TrafficLight.GREEN
            elif(light==1):
                return TrafficLight.YELLOW
            elif(light==2):
                return TrafficLight.RED
        return TrafficLight.UNKNOWN
