#! /usr/bin/env python

import rospy

from sensor_msgs.msg import LaserScan

from std_msgs.msg import Int16MultiArray

def clbk_laser(msg):
    # 360 / 3 = 120
    regions = [
        min(min(msg.ranges[0:10]), 10),
        min(min(msg.ranges[180:190]), 10),
        min(min(msg.ranges[350:359]), 10),
    ]
    if(regions[0]>2):
        right = True
    else:
        right = False
    if(regions[1]>2):
        forward = True
    else:
        forward=False
    if(regions[2]>2):
        left = True
    else:
        left =False

    actions = [left,forward,right]  #possible actions
    possible_actions =   Int16MultiArray()  #possible actions but for the topic

    possible_actions.data = actions

    rospy.loginfo(actions)  #I f we want to show
    pub = rospy.Publisher('possible_actions',Int16MultiArray,queue_size=10)


    pub.publish(possible_actions)


def main():
    rospy.init_node('reading_laser')

    sub = rospy.Subscriber('/scan', LaserScan, clbk_laser)

    rospy.spin()

if __name__ == '__main__':
    main()
