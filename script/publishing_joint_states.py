#! /usr/bin/python
#!/usr/bin/env python3
#!/usr/bin/env python2

import sys
import time
import numpy as np
import rospy
from sensor_msgs.msg import JointState
import matplotlib.pyplot as plt
from matplotlib import animation

pos = []
t = []
fig, ax = plt.subplots()
ax.set_xlim(0, 100000)
ax.set_ylim(0, 100)
line, = ax.plot(0, 0)


def draw_graph(i):
    global pos
    global t
    line.set_xdata(t)
    line.set_ydata(pos)
    # return line,

    plt.cla()
    plt.plot(t, pos)



def Joint_States_for_Metal_Robot_Callback_function(joint_state_data):
    global pos
    global t

    Name_of_the_Joint = joint_state_data.name[0]
    if Name_of_the_Joint == "Back_right_wheel_joint":
        # print('Position= {0} and Velocity={1} of the BRWJ'.format(joint_state_data.position[0],joint_state_data.velocity[0]))
        print('Position of the BRWJ=', joint_state_data.position[0])
        print('Sequence=', joint_state_data.header.seq)
        pos.append(joint_state_data.position[0])
        t.append(joint_state_data.header.seq)


if __name__ == '__main__':

    #global water_detected

    rospy.init_node('Publishing_Joint_States_Node')

    reciever = rospy.Subscriber("/joint_states", JointState,
                                Joint_States_for_Metal_Robot_Callback_function, queue_size=10)

    # pub_move = rospy.Publisher(dronetype+'/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
    #anim = animation.FuncAnimation(fig, draw_graph, interval=1)
    anim=animation.FuncAnimation(plt.gcf(),draw_graph,interval=1)
    plt.show()

    rospy.spin()
 

 
