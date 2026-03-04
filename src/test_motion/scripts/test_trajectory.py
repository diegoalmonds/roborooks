#!/usr/bin/env python3

import rospy
from intera_motion_interface import (
    MotionTrajectory,
    MotionWaypoint,
    MotionWaypointOptions
)

import intera_interface
from intera_interface import CHECK_VERSION

class MotionTest(object):
    def __init__(self):
        self._done = False
        self._limb = intera_interface.Limb()
        self.joint_names = self._limb.joint_names()

        self._rs = intera_interface.RobotEnable(CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()
        rospy.sleep(1)
        print("Running. Ctrl-c to quit")

    def move(self):
        traj = MotionTrajectory(limb=self._limb)
        wpt_opts = MotionWaypointOptions()
        wpt = MotionWaypoint(options=wpt_opts, limb=self._limb)

        wpt.set_joint_angles(joint_angles = [0, 0, 0, 0, 0, 0, 0]) # CHANGE JOINT ANGLES TO SOME ARBIRRARY POSITION
        traj.append_waypoint(wpt)
        wpt.set_joint_angles(joint_angles = [0, 0, 0, 0, 0, 0, 0]) # CHANGE JOINT ANGLES TO SOME ARBIRRARY POSITION
        traj.append_waypoint(wpt)

        # Send and execute the trajectory
        result = traj.send_trajectory()
        if result is None:
            rospy.logerr("Trajectory execution failed!")
        elif result.result:
            rospy.loginfo("Trajectory execution succeeded!")
        else:
            rospy.logwarn("Trajectory execution returned False")
        
def main():
    print("Initializing node...")
    rospy.init_node("motion_test")

    motion_test = MotionTest()
    print("Moving")
    motion_test.move()
    print("Done.")

if __name__ == '__main__':
    main()