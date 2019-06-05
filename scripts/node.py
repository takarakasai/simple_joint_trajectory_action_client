#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import JointState
from controller_manager_msgs.utils \
    import ControllerLister, ControllerManagerLister
from controller_manager_msgs.utils \
    import filter_by_type, filter_by_state

from simple_joint_trajectory_action_client.srv\
    import JointTrajectoryService, JointTrajectoryServiceRequest, JointTrajectoryServiceResponse

import actionlib
import control_msgs.msg 

is_ready = False
joint_states = {}
joint2controller = {}
controller2joints = {}
action_clients = {}

node_name = "simple_joint_trajectory_action_client"

def callback(js):
    for i, name in enumerate(js.name):
        joint_states[name] = js.position[i]

    global is_ready
    if not is_ready:
        rospy.loginfo("Received 1st joint_states")
        for i, name in enumerate(js.name):
            rospy.loginfo("  {:>30} :{:+5.2f} [rad], {:+7.2f} [deg]".format(
                name, js.position[i], np.rad2deg(js.position[i])))
        is_ready = True

def goal_cb(joint_values):
    print joint_values

def request_cb(req):
    if not is_ready:
        rospy.logerr("not initialized with joint states yet.")
        return JointTrajectoryServiceResponse(False)

    controllers = []
    msgs = {}
    for i, name in enumerate(req.name):
        controller_name = joint2controller[name]

        # FIXME: not efficient
        if controller_name not in controllers:
            controllers.append(controller_name)
            msgs[controller_name] = control_msgs.msg.FollowJointTrajectoryGoal();

            # start node
            point = control_msgs.msg.trajectory_msgs.msg.JointTrajectoryPoint()
            point.time_from_start.secs = 0
            point.time_from_start.nsecs = 0
            msgs[controller_name].trajectory.points.append(point)

            # intermediate nodes
            point = control_msgs.msg.trajectory_msgs.msg.JointTrajectoryPoint()
            point.time_from_start = req.duration.data * 1 / 2
            msgs[controller_name].trajectory.points.append(point)

            # end node
            point = control_msgs.msg.trajectory_msgs.msg.JointTrajectoryPoint()
            point.time_from_start = req.duration.data
            msgs[controller_name].trajectory.points.append(point)

        msgs[controller_name].trajectory.joint_names.append(name)
        req_position = req.position[i]
        cur_position = joint_states[name]
        rospy.loginfo("{} : {} --> {} at sec:{} nsec:{}".format(
                name, np.rad2deg(cur_position), np.rad2deg(req_position),
                req.duration.data.secs, req.duration.data.nsecs))
        msgs[controller_name].trajectory.points[0].positions.append(cur_position)
        msgs[controller_name].trajectory.points[1].positions.append((req_position - cur_position) * 1 / 2 + cur_position)
        msgs[controller_name].trajectory.points[2].positions.append(req_position)

    for controller, msg in msgs.items():
        for jname in controller2joints[controller]:
            if jname not in msg.trajectory.joint_names:
                cur_position = joint_states[jname]
                msg.trajectory.joint_names.append(jname)
                # nodes
                msgs[controller_name].trajectory.points[0].positions.append(cur_position)
                msgs[controller_name].trajectory.points[1].positions.append(cur_position)
                msgs[controller_name].trajectory.points[2].positions.append(cur_position)

    for controller, msg in msgs.items():
        # print msg
        rospy.loginfo("Sending Request to {}".format(controller))
        action_clients[controller].send_goal(msg)

    for controller, msg in msgs.items():
        client = action_clients[controller]
        rospy.loginfo("Waiting Result to {}".format(controller))
        client.wait_for_result()
        rospy.loginfo("        Result : {}".format(client.get_result()))

    return JointTrajectoryServiceResponse(True)
    
def node():
    rospy.init_node(node_name, anonymous=True)

    # FIXME: handle multiple controller manager
    # ms = ControllerManagerLister()

    cs = ControllerLister()()
    jtc_list = filter_by_type(cs, 'JointTrajectoryController', match_substring=True)
    running_jtc_list = filter_by_state(jtc_list, 'running')
    for con in running_jtc_list:
        # FIXME: handle multiple claimed_resources
        controller2joints[con.name] = []
        for resource in con.claimed_resources[0].resources:
          joint2controller[resource] = con.name
          controller2joints[con.name].append(resource)

        name = '/' + con.name + '/follow_joint_trajectory'
        action_clients[con.name] = actionlib.SimpleActionClient(name, control_msgs.msg.FollowJointTrajectoryAction)

    for name, ac in action_clients.items():
        rospy.loginfo("waiting for {}".format(name))
        ac.wait_for_server()

    rospy.Subscriber("/joint_states", JointState, callback)
    rospy.Subscriber(node_name + "/goals", JointState, goal_cb)
    rospy.Service(node_name + "/requests", JointTrajectoryService, request_cb)

    rospy.spin()

if __name__ == '__main__':
    node()
