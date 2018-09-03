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

        msg2 = Float32MultiArray()
        data2 = data.copy() * 100
        msg2.data = data2
        self.pub_demo2.publish(msg2)

    def onInit(*args, **kwargs):
        pynodelet.Nodelet.onInit(*args, **kwargs)

        pynodelet.Nodelet.Subscriber("/demo", Float32MultiArray, self.callback_demo)
        self.pub_demo2 = pynodelet.Nodelet.Publisher("/demo2", Float32MultiArray, queue_size = 1)
        rospy.spin()

