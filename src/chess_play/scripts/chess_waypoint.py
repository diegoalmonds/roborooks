#!/usr/bin/env python3

import rospy
import intera_interface
import rospkg
import yaml
import os
import serial

from intera_motion_interface import (
    MotionWaypoint,
    MotionTrajectory,
    MotionWaypointOptions
)

from geometry_msgs.msg import Pose, Point, Quaternion
from board_cv.msg import Move

piece_heights = {
    'P': 0.067,
    'R': 0.059,
    'N': 0.045,
    'B': 0.035,
    'Q': 0.03,
    'K': 0.0275
}

PORT = "/dev/ttyACM0"
BAUD = 115200

ser = serial.Serial(PORT, BAUD, timeout=1)

def magnet_on():
    cmd = "PIN 9 ON\n"
    ser.write(cmd.encode())
    print("MAGNET ON")

def magnet_off():
    cmd = "PIN 9 OFF\n"
    ser.write(cmd.encode())
    print("MAGNET OFF")

class ChessWaypointSystem:
    def __init__(self):
        rospy.loginfo("Initializing Sawyer chess waypoint system...")
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path("position_calibration")
        self._yaml_path = os.path.join(pkg_path, "config", "board_positions.yaml")
        self._board_positions = self._load_yaml()

        self._limb = intera_interface.Limb()
        self._traj = MotionTrajectory()
        self._wpt_opts = MotionWaypointOptions(max_joint_speed_ratio = 0.4)
        self._wpt = MotionWaypoint(options=self._wpt_opts, limb=self._limb)
        self._ai_move_sub = rospy.Subscriber(
            "/ai_move",
            Move,
            self._perform_ai_move
        )

        self._rs = intera_interface.RobotEnable(intera_interface.CHECK_VERSION)
        self._rs.enable()

    def _load_yaml(self):
        with open(self._yaml_path, 'r') as f:
            return yaml.safe_load(f)

    def _perform_ai_move(self, msg):
        rospy.loginfo("Received AI move.")
        print(msg)
        move_type = msg.move_type
        move_notation = msg.notation
        from_square = msg.from_square
        to_square = msg.to_square
        from_piece = msg.from_piece.upper()
        to_piece = msg.to_piece.upper()
        promotion = msg.promotion_piece
        rospy.loginfo(f"Executing move: {move_notation}")
        if 'checkmate' == move_type:
            self._checkmate()
        elif 'capture' == move_type: # capture
            self._discard(square=to_square, piece=to_piece)
            self._move(from_square=from_square, to_square=to_square, piece=from_piece)
        elif "promotion" == move_type:
            self._promote(from_square=from_square, to_square=to_square, from_piece=from_piece, promotion_piece=promotion)
        else: # castle or regular move
            self._move(from_square=from_square, to_square=to_square, piece=from_piece)
        rospy.loginfo("Sending trajectory now...")
        rospy.loginfo(f"Trajectory object: {self._traj}")
        move = self._traj.send_trajectory()
        if move is None:
            rospy.loginfo(f"Move executed returned None for: {move_notation}")
        elif move.result:
            rospy.loginfo(f"Successfully executed move: {move_notation}")
        else:
            rospy.loginfo(f"Failed to execute move: {move_notation}")
        
        self._traj.clear_waypoints()


    def _move(self, from_square, to_square, piece):
        rospy.loginfo(f"Moving from {from_square} to {to_square}")
        rospy.loginfo(f"From square joint angles: {self._board_positions[from_square]['joint_angles']}")
        rospy.loginfo(f"To square joint angles: {self._board_positions[to_square]['joint_angles']}")
        
        # START SQUARE
        self._send_single_waypoint(self._board_positions[from_square]['joint_angles'])
        self._pick(piece=piece, square=from_square, prev_pos=self._board_positions[from_square]['joint_angles'])

        # END SQUARE
        self._send_single_waypoint(self._board_positions[to_square]['joint_angles'])
        self._pick(piece=piece, square=to_square, prev_pos=self._board_positions[to_square]['joint_angles'], release=True)

        # RESET
        self._send_single_waypoint(self._board_positions['away']['joint_angles'])

    def _discard(self, square, piece):
        """
        Discards a piece by moving it to a designated discard area.

        :param square: the square from which the piece is being discarded (e.g., 'e4')
        :param piece: the type of chess piece being discarded (e.g., 'pawn', 'rook', 'knight', 'bishop', 'queen', 'king')
        """
        
        self._send_single_waypoint(self._board_positions[square]['joint_angles'])
        self._pick(piece=piece, square=square, prev_pos=self._board_positions[square]['joint_angles'])
        self._send_single_waypoint(self._board_positions['discard']['joint_angles'])
        self._pick(piece=piece, square="discard", prev_pos=self._board_positions['discard']['joint_angles'], release=True)

    def _pick(self, square, piece, prev_pos, release=False):
        """
        Lower or raise the end effector by a pre-determined amount based on the piece type and provides the option to grasp or release the piece.
        
        :param piece: the type of chess piece being manipulated (e.g., 'pawn', 'rook', 'knight', 'bishop', 'queen', 'king')
        :param release: whether to lower and grab the piece (False) or to lower and release the piece (True), defaults to False
        :param prev_pos: the joint angles of the previous position, used as the seed for IK calculations to ensure more consistent solutions
        """
        
        # create joint dictionary for IK seed
        joint_keys = ['right_j0', 'right_j1', 'right_j2', 'right_j3', 'right_j4', 'right_j5', 'right_j6']
        ik_seed = dict(zip(joint_keys, prev_pos))
        
        pos = self._board_positions[square]['position']
        pick_pose = Pose()
        pick_pose.position = Point(
            x = pos['x'],
            y = pos['y'],
            z = pos['z'] - piece_heights[piece] - (pos['z'] - .36) # subtract .36 as baseline height
        )
        
        ori = self._board_positions[square]['orientation']
        pick_pose.orientation = Quaternion(
            x=ori['x'],
            y=ori['y'],
            z=ori['z'],
            w=ori['w']
        )

        joint_solution = self._limb.ik_request(pick_pose, seed=ik_seed) # use previous joint angles as seed for IK
        if joint_solution:
            angles = list(joint_solution.values())
            self._send_single_waypoint(angles = angles)
        else:
            rospy.logerr("No IK solution found for pick pose of piece %s at square %s", piece, square)
        
        # toggle magnet based on pick or release 
        if release:
            magnet_off()
        else:
            magnet_on()
        
        # raise end effector back up after pick/release
        pick_pose.position.z = pos['z']
        ik_seed = dict(zip(joint_keys, joint_solution.values()))
        joint_solution = self._limb.ik_request(pick_pose, seed=ik_seed) # use previous joint solution as seed for raised pose
        if joint_solution:
            self._send_single_waypoint(angles = list(joint_solution.values()))
        else:
            rospy.logerr("No IK solution found for raised pose of piece %s at square %s", piece, square)
            
        # RESET  
        self._send_single_waypoint(self._board_positions['base']['joint_angles'])

    def _promote(self, from_square, to_square, from_piece, promotion_piece):
        self._move(from_square=from_square, to_square=to_square, from_piece=from_piece, piece="P") # move pawn to promotion square
        self._discard(square=to_square, piece="P") # discard pawn
        self._move(from_square=promotion_piece + "_promote", to_square=to_square, piece=promotion_piece) # move promotion piece to promotion square

    def _checkmate(self):
        # define checkmate sequence of waypoints here
        pass

    # add type checking logic (is of type Pose, or type list with 7 joint angles?)
    def _append_waypoint(self, angles):
        rospy.loginfo(f"Appending waypoint with angles: {angles}")

        if angles is None:
            rospy.logerr("Waypoint angles are None!")
            return

        if len(angles) != 7:
            rospy.logerr(f"Invalid joint angle length: {len(angles)} (expected 7)")
            return
        
        self._wpt.set_joint_angles(joint_angles = angles)
        self._traj.append_waypoint(self._wpt)

    def _send_single_waypoint(self, angles, pause=0.5):
        rospy.loginfo(f"Sending single waypoint: {angles}")

        if angles is None:
            rospy.logerr("Waypoint angles are None!")
            return

        if len(angles) != 7:
            rospy.logerr(f"Invalid joint angle length: {len(angles)} (expected 7)")
            return

        traj = MotionTrajectory()
        wpt = MotionWaypoint(options=self._wpt_opts, limb=self._limb)
        wpt.set_joint_angles(joint_angles=angles)
        traj.append_waypoint(wpt)

        result = traj.send_trajectory()

        if result is None:
            rospy.logerr("Trajectory execution returned None")
        elif result.result:
            rospy.loginfo("Waypoint executed successfully")
        else:
            rospy.logerr("Waypoint execution failed")

        rospy.sleep(pause)

def main():
    rospy.init_node("chess_waypoint_system")
    chess_waypoint_system = ChessWaypointSystem()
    rospy.spin()
    ser.close()

if __name__ == "__main__":
    main()