#! /usr/bin/env python

# Compare the messages we send in /cmd_vel with the messages we compute in /odom
# We expect odom.twist.twist.linear.x to be the speed sent via /cmd_vel
# We expect odom.twist.twist.angular.z to be the turn sent via /cmd_vel

# The robot will move forward for X seconds, then turn left (counter-clockwise) for Y seconds,
#       then move backward for X seconds, then turn right (clockwise) for Y seconds

# We are computing the averages for each action and output it to the screen

import rospy, time
from nav_msgs.msg import Odometry

from nav_timed import NavTimed

class CmdVelOdomTest0(object):

    def __init__(self):
        self._timeStart = None
        self._timeExpectedEnd = None

        rospy.Subscriber('/odom', Odometry, self._odomCallback)
        self._odomLinearMsgs = []
        self._odomAngularMsgs = []
        self._linearVelocity = 0
        self._angularVelocity = 0


    def _odomCallback(self, msg):
        if self._timeStart is None or self._timeExpectedEnd is None:
            return

        duration = (rospy.Time.now() - self._timeStart).to_sec()
        if duration < 2:
            return
        duration = (self._timeExpectedEnd - rospy.Time.now()).to_sec()
        if duration < 2:
            return

        self._odomLinearMsgs.append(msg.twist.twist.linear.x)
        self._odomAngularMsgs.append(msg.twist.twist.angular.z)


    def _printAveragesAndReset(self):
        linearNum = len(self._odomLinearMsgs)
        linearAvg = sum(self._odomLinearMsgs) / linearNum
        if self._linearVelocity:
            linearErr = 100 - (linearAvg * 100 / self._linearVelocity)
        else:
            linearErr = 0

        angularNum = len(self._odomAngularMsgs)
        angularAvg = sum(self._odomAngularMsgs) / angularNum
        if self._angularVelocity:
            angularErr = 100 - (angularAvg * 100 / self._angularVelocity)
        else:
            angularErr = 0

        rospy.loginfo('linearAvg:  %.4f, linearErr:  %.2f%%, linearNum:  %d' %(linearAvg,  linearErr,  linearNum ) )
        rospy.loginfo('angularAvg: %.4f, angularErr: %.2f%%, angularNum: %d' %(angularAvg, angularErr, angularNum) )

        self._odomLinearMsgs = []
        self._odomAngularMsgs = []


    def main(self):
        duration = 8
        rate = 20

        self._linearVelocity = 0.16
        self._angularVelocity = 0
        raw_input('============= Move forward for %ds with speed %.4f m/s?' %(duration, self._linearVelocity) )
        self._timeStart = rospy.Time.now()
        self._timeExpectedEnd = self._timeStart + rospy.Duration(duration)
        navTimed = NavTimed(duration, self._linearVelocity, self._angularVelocity, rate)
        navTimed.spin()
        self._printAveragesAndReset()

        self._linearVelocity = 0
        self._angularVelocity = 0.8
        raw_input('============= Turn left for %ds with speed %.4f rad/s?' %(duration, self._angularVelocity) )
        self._timeStart = rospy.Time.now()
        self._timeExpectedEnd = self._timeStart + rospy.Duration(duration)
        navTimed = NavTimed(duration, self._linearVelocity, self._angularVelocity, rate)
        navTimed.spin()
        self._printAveragesAndReset()

        self._linearVelocity = -0.16
        self._angularVelocity = 0
        raw_input('============= Move backwards for %ds with speed %.4f m/s?' %(duration, self._linearVelocity) )
        self._timeStart = rospy.Time.now()
        self._timeExpectedEnd = self._timeStart + rospy.Duration(duration)
        navTimed = NavTimed(duration, self._linearVelocity, self._angularVelocity, rate)
        navTimed.spin()
        self._printAveragesAndReset()

        self._linearVelocity = 0
        self._angularVelocity = -0.8
        raw_input('============= Turn right for %ds with speed %.4f rad/s?' %(duration, self._angularVelocity) )
        self._timeStart = rospy.Time.now()
        self._timeExpectedEnd = self._timeStart + rospy.Duration(duration)
        navTimed = NavTimed(duration, self._linearVelocity, self._angularVelocity, rate)
        navTimed.spin()
        self._printAveragesAndReset()


if __name__ == '__main__':
    rospy.init_node('cmd_vel_odom_test0')
    cmdVelOdomTest0 = CmdVelOdomTest0()
    try:
        cmdVelOdomTest0.main()
    except rospy.ROSInterruptException:
        pass

