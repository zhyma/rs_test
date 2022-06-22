#!/usr/bin/env python3

import rospy
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import Pose

from gazebo_msgs.srv import GetLinkState

class cable_detection():

    def __init__(self, no_of_links):
        # You need to initializing a node before instantiate the class
        self.links = []
        self.no_of_links = no_of_links
        pass

    def get_links(self):
        rospy.wait_for_service('/gazebo/get_link_state')
        self.links.clear()

        read_state = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
        for i in range(self.no_of_links): 
            state = read_state(link_name='link_'+str(i))
            # if state is not None:
            self.links.append(state)

        pass


if __name__ == '__main__':
    rospy.init_node("cable_detector", anonymous=True)
    cable = cable_detection(50)
    cable.get_links()
    print(len(cable.links))
    print(cable.links[0])

