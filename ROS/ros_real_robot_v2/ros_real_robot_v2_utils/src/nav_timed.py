#!/usr/bin/env python

# Direct robot to move via geometry_msgs/Twist messages for some time duration

import rospy
from geometry_msgs.msg import Twist

class NavTimed():
    def __init__(self, duration = 8, linearVelocity = 0.1, angularVelocity = 0, rate = 20):

        self._cmdVelPublisher = rospy.Publisher('cmd_vel', Twist, queue_size = 10)
        self._timeStart = rospy.Time.now()

        self._ctrlC = False
        self._done = False

        rospy.on_shutdown(self._onShutdownHook)

        if rospy.get_param('use_sim_time', False):
            rospy.logdebug('use_sim_time is true, waiting for the first clock message...')
            while not self._timeStart:
                self._timeStart = rospy.Time.now()
            rospy.logdebug('Got first clock message!')

        self._duration = duration
        self._linearVelocity = linearVelocity
        self._angularVelocity = angularVelocity
        self._rate = rate
        self._numMessages = 0


    def _onShutdownHook(self):
        rospy.logdebug("Shutting down nav_timed")
        self.stopRobot()
        self._ctrlC = True
        self._done = True
        rospy.sleep(1)


    def stopRobot(self):
        myRate = rospy.Rate(10)

        while not self._ctrlC:
            connections = self._cmdVelPublisher.get_num_connections()
            if connections > 0:
                self._cmdVelPublisher.publish(Twist())
                rospy.logdebug("stop message published")
                break
            else:
                myRate.sleep()


    def update(self):
        duration = (rospy.Time.now() - self._timeStart).to_sec()
        if duration >= self._duration:
            rospy.loginfo('Finished! duration = %d, num_messages = %d' %(duration, self._numMessages) )
            self._done = True
            return

        cmdVelMessage = Twist()
        cmdVelMessage.linear.x = self._linearVelocity
        cmdVelMessage.angular.z = self._angularVelocity
        self._cmdVelPublisher.publish(cmdVelMessage)
        self._numMessages = self._numMessages + 1


    def spin(self):
        # rospy.logdebug("Starting nav_timed")
        rate = rospy.Rate(self._rate)

        if self._ctrlC:
            return

        if self._done:
            # rospy.logdebug("Stopping nav_timed")
            self._cmdVelPublisher.publish(Twist())
            rospy.sleep(1)
            return

        while not self._done:
            self.update()
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                rospy.loginfo('ROSInterruptException! Expected if use_sim_time is true.')

        self.spin()


if __name__ == '__main__':
    rospy.init_node('nav_timed')
    navTimed = NavTimed();
    navTimed.spin()

