#!/usr/bin/env python3

def module_from_file(module_name, file_path):
    spec = importlib.util.spec_from_file_location(module_name, file_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module

class Nodelet(object):
    def __init__(self, nodelet_manager):
        self.nodelet_manager = nodelet_manager
        return

    def init(self, name):
        self.nodelet_name = name
        self.onInit()
        return

    def onInit(self):
        # override this
        return

    def Subscriber(self, topic_name, topic_type, callback):
        return self.nodelet_manager.Subscriber(topic_name, topic_type, callback)

    def Publisher(self, topic_name, topic_type, queue_size = None):
        return self.nodelet_manager.Publisher(topic_name, topic_type, queue_size = queue_size)

class NodeletSubscriber(object):
    """
    A wrapper that mimics the API of rospy.Subscriber but deals with directly
    sending messages to nodelets within the same nodelet manager. Instantiation however
    differs from rospy.Subscriber; nodelets should call Nodelet.Subscriber to get the same API
    as rospy.Subscriber.
    """
    def __init__(self, topic_name, topic_type, callback):
        self.topic_name = topic_name
        self.topic_type = topic_type
        self.callback = callback
        self.last_msg = None
        self.sub = rospy.Subscriber(topic_name, topic_type, self.callback_once)
        return

    def callback_once(self, msg):
        """
        Calls self.callback() but only once for a given message.
        Duplicate calls will occur if an internal node publishes via TCP and via memory at the same time.
        """
        if msg is self.last_msg: # important: must be "is" not "=="
            return

        self.last_msg = msg
        self.callback(msg)

class NodeletPublisher(object):
    """
    A wrapper that mimics the publish() API of rospy.Publisher but deals with directly
    sending messages to nodelets within the same nodelet manager. Instantiation however
    differs from rospy.Publisher; nodelets should call Nodelet.Publisher to get the same API
    as rospy.Publisher.
    """
    def __init__(self, nodelet_manager, topic_name, topic_type, queue_size = None):
        self.nodelet_manager = nodelet_manager
        self.topic_name = topic_name
        self.topic_type = topic_type
        self.pub = rospy.Publisher(topic_name, topic_type, queue_size = queue_size)
        return

    def publish(self, msg):
        # publish over TCP only if there are connections to topic from outside the nodelet manager
        if self.pub.get_num_connections() > len(self.nodelet_manager.subscribers[self.topic_name]):
            self.pub.publish(msg)

        # always publish to nodelets within the nodelet manager
        if self.topic_name in self.nodelet_manager.subscribers:
            for subscriber in self.nodelet_manager.subscribers[self.topic_name]:
                subscriber.callback_once(msg)

class NodeletManager(object):

    def __init__(self, name = None):
        self.modules = {}
        self.publishers = {}
        self.subscribers = {}
        self.nodelet_manager_name = name

    def loadNodelet(_pkg, _type, name):
        NodeletClass = nodule_from_file("Nodelet", get_file(_pkg, _type))
        nodelet = NodeletClass()
        self.modules[name] = nodelet
        nodelet.init()

    def Subscriber(topic_name, topic_type, callback):
        if self.subscribers.get(topic_name) is None:
            self.subscribers[topic_name] = []
        s = NodeletSubscriber(topic_name, topic_type, callback)
        self.subscribers[topic_name].append(s)
        return s

    def Publisher(topic_name, topic_type, queue_size = 1):
        if self.publishers.get(topic_name) is None:
            self.publishers[topic_name] = []
        p = NodeletPublisher(self, topic_name, topic_type, queue_size = queue_size)
        self.publishers[topic_name].append(p)
        return p

