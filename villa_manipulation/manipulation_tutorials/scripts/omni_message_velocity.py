#!/usr/bin/env python
import roslib
import rospy
import geometry_msgs.msg
import controller_manager_msgs.srv

rospy.init_node('test')

# initialize ROS publisher
pub = rospy.Publisher('/hsrb/command_velocity', geometry_msgs.msg.Twist, queue_size=10)

# wait to establish connection between the controller
while pub.get_num_connections() == 0:
    rospy.sleep(0.1)

# make sure the controller is running
rospy.wait_for_service('/hsrb/controller_manager/list_controllers')
list_controllers = rospy.ServiceProxy('/hsrb/controller_manager/list_controllers', controller_manager_msgs.srv.ListControllers)
running = False
while running == False:
    rospy.sleep(0.1)
    for c in list_controllers().controller:
        if c.name == 'omni_base_controller' and c.state == 'running':
            running = True

# fill ROS message
tw = geometry_msgs.msg.Twist()
tw.linear.x = 0.0
tw.angular.z=0.2

# publish ROS message
pub.publish(tw)
