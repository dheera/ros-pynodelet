#!/usr/bin/env python3

import pynodelet.Nodelet
import numpy
import rospy
from std_msgs.msg import Float32MultiArray

class Nodelet(pynodelet.Nodelet):
    def __init__(*args, **kwargs):
        pynodelet.Nodelet.__init__(*args, **kwargs)
        return

    def callback_demo(self, msg):
        """
        Multiply incoming floats by 100 and republish.
        """
        data = numpy.frombuffer(msg.data, dtype = np.float32)

        rospy.loginfo("received an array of size %d with mean = %f std = %f" % \
                (data.size, np.mean(data), np.std(data))
        )

    def onInit(*args, **kwargs):
        pynodelet.Nodelet.onInit(*args, **kwargs)

        pynodelet.Nodelet.Subscriber("/demo2", Float32MultiArray, self.callback_demo)
        rospy.spin()

