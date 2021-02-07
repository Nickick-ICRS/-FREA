import rospy
from std_srvs.srv import Empty
from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3

class DartConnection():
    def __init__(self, max_retry=20):
        self._max_retry = max_retry
        self.unpause = rospy.ServiceProxy('/frea_dart/unpause', Empty)
        self.pause = rospy.ServiceProxy('/frea_dart/pause', Empty)
        self.step = rospy.ServiceProxy('/frea_dart/step', Empty)
        self.reset_world = rospy.ServiceProxy(
            '/frea_dart/reset', Empty)

        self.init_values()
        self.pauseSim()

    def pauseSim(self):
        rospy.wait_for_service('/frea_dart/pause')
        paused_done = False
        counter = 0
        while not paused_done and not rospy.is_shutdown():
            if counter < self._max_retry:
                try:
                    self.pause()
                    paused_done = True
                except rospy.ServiceException as e:
                    counter += 1
                    rospy.logerr('/frea_dart/pause service call failed')
            else:
                msg = 'Max retries ' + str(self._max_retry) + ' reached.'
                rospy.logerr(msg)
                assert(False, msg)

    def unpauseSim(self):
        rospy.wait_for_service('/frea_dart/unpause')
        unpaused_done = False
        counter = 0
        while not unpaused_done and not rospy.is_shutdown():
            if counter < self._max_retry:
                try:
                    self.unpause()
                    unpaused_done = True
                except rospy.ServiceException as e:
                    counter += 1
                    rospy.logerr('/frea_dart/unpause service call failed')
            else:
                msg = 'Max retries ' + str(self._max_retry) + ' reached.'
                rospy.logerr(msg)
                assert(False, msg)

    def resetSim(self):
        try:
            self.reset_world()
        except rospy.ServiceException as e:
            rospy.logerr('/frea_dart/reset_world service call failed')

    def init_values(self):
        self.resetSim()
