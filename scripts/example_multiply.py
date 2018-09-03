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
        Multiply incoming floats by 5 and republish.
        """
        data = numpy.frombuffer(msg.data, dtype = np.float32)

        msg_out = Float32MultiArray()
        data_out = data.copy() * 5
        msg_out.data = data_out
        self.pub_demo.publish(msg_out)

    def onInit(*args, **kwargs):
        pynodelet.Nodelet.onInit(*args, **kwargs)

        self.factor = rospy.get_param("~factor", 10)
        self.topic_in = rospy.get_param("~topic_in", "foo")
        self.topic_out = rospy.get_param("~topic_out", "%s_%d" % (self.topic_in, self.factor))

        pynodelet.Nodelet.Subscriber(self.topic_in, Float32MultiArray, self.callback_demo)
        self.pub_demo2 = pynodelet.Nodelet.Publisher(self.topic_out, Float32MultiArray, queue_size = 1)
        rospy.spin()

