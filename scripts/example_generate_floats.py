#!/usr/bin/env python3

import pynodelet.Nodelet
import numpy
import rospy
from std_msgs.msg import Float32MultiArray

class Nodelet(pynodelet.Nodelet):
    def __init__(*args, **kwargs):
        pynodelet.Nodelet.__init__(*args, **kwargs)
        return

    def onInit(*args, **kwargs):
        pynodelet.Nodelet.onInit(*args, **kwargs)

        self.pub_demo = pynodelet.Nodelet.Publisher("foo", Float32MultiArray, queue_size = 1)

        # Generate 10000 random floats at a rate of 10Hz and publish it

        rate = rospy.Rate(10)
        while rospy.is_shutdown():
            rate.sleep()
            msg = Float32MultiArray()
            msg.data = numpy.random.randn(10000).ascontiguousarray().tostring()
            self.pub_demo.publish(msg)

