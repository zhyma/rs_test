#!/usr/bin/env python3

## re-initializing the RL environment (reset the rod and the cable)

import rospy
from yumi_gazebo.msg import CylinderProperties

class env_reset():
    def __init__(self):
        self.pub = rospy.Publisher('/set_rod_properties', CylinderProperties, queue_size = 10)
        rospy.sleep(1)

    def publish(self, x=0.3, y=0, z=0.3, r=0.02, l=0.05):
        data = CylinderProperties()
        data.position.x = x
        data.position.y = y
        data.position.z = z
        data.orientation.x = 0
        data.orientation.y = 0
        data.orientation.z = 0.707
        data.orientation.w = 0.707
        data.r = r
        data.l = l
        self.pub.publish(data)
        print("environment re-initialized!")

if __name__ == '__main__':
    rospy.init_node('reset_env', anonymous = True)
    env_pub = env_reset()
    env_pub.publish(x=0.3, y=0, z=0.3, r=0.02, l=0.1)