#!/usr/bin/env python
import getch
import rospy
from std_msgs.msg import Float64, String, Int8

def controller():
    speed = rospy.Publisher('/vesc/commands/motor/speed', Float64, queue_size=1)
    position = rospy.Publisher('/vesc/commands/servo/position', Float64, queue_size=1)

    rospy.init_node('key_controller',anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        k = ord(getch.getch())
        if (k == 113):
            rospy.loginfo("Exit..")
            exit()

        elif (k == 65):
            rospy.loginfo("front")
            position.publish(0.5304)
            speed.publish(10000)

        elif (k == 66):
            rospy.loginfo("stop")
            position.publish(0.5304)
            speed.publish(0)

        elif (k == 67):
            rospy.loginfo("right")
            position.publish(0.7104)
            speed.publish(8000)

        elif (k == 68):
            rospy.loginfo("left")
            position.publish(0.3504)
            speed.publish(8000)

# quit 113
# up 65
# down 66
# left 68
# right 67

if __name__=='__main__':
    try:
        controller()
    except rospy.ROSInterruptException:
        pass
