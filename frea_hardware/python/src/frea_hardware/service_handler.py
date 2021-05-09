import rospy
from std_msgs.msg import Float32, UInt8MultiArray


class ServiceHandler:
    def __init__(self, head_communicator):
        """
        @brief Class to handle the ROS communication interface

        @param head_communicator The HeadCommunicator class that handles
                                 communication with the Arduino
        """
        self._hc = head_communicator

        rospy.Subscriber(
            "/frea/position/head_joint/command", Float32, self.neckCallback)
        rospy.Subscriber(
            "/frea/position/mouth_joint/command", Float32,
            self.mouthCallback)
        rospy.Subscriber(
            "/frea/position/left_ear_joint/command", Float32,
            self.leftEarCallback)
        rospy.Subscriber(
            "/frea/position/right_ear_joint/command", Float32,
            self.rightEarCallback)
        rospy.Subscriber(
            "/frea/lights/head_lights", UInt8MultiArray,
            self.pixelCallback)

        self._head_pub = rospy.Publisher(
            "/frea/position/head_joint/state", Float32, queue_size=10)
        self._mouth_pub = rospy.Publisher(
            "/frea/position/mouth_joint/state", Float32, queue_size=10)
        self._left_ear_pub = rospy.Publisher(
            "/frea/position/left_ear_joint/state", Float32, queue_size=10)
        self._right_ear_pub = rospy.Publisher(
            "/frea/position/right_ear_joint/state", Float32, queue_size=10)
    
    def neckCallback(self, msg):
        self._hc.setHeadRoll(msg.data)
        # Servos have no feedback so we assume we got there instantly
        self._head_pub.publish(msg)

    def mouthCallback(self, msg):
        self._hc.setMouthPitch(msg.data)
        # Servos have no feedback so we assume we got there instantly
        self._mouth_pub.publish(msg)

    def rightEarCallback(self, msg):
        self._hc.setRightEarPitch(self, msg)
        # Servos have no feedback so we assume we got there instantly
        self._right_ear_pub.publish(msg)
    
    def leftEarCallback(self, msg):
        self._hc.setLeftEarPitch(self, msg)
        # Servos have no feedback so we assume we got there instantly
        self._left_ear_pub.publish(msg)

    def pixelCallback(self, msg):
        if len(msg.data) != 27:
            rospy.logerr(
                "HSV values for 9 pixels should be 27 values! (received: "
                +str(len(msg.data)) +")")
            return
        h = []
        s = []
        v = []
        i = 0
        while i < 27:
            h.append(msg.data[i])
            i += 1
            s.append(msg.data[i])
            i += 1
            v.append(msg.data[i])
            i += 1
        self._hc.setPixelColours(h, s, v)
