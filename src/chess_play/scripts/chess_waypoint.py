#!/usr/bin/env python3

import rospy
import intera_interface
import rospkg
import yaml
import os

from intera_motion_interface import (
    MotionTrajectory,
    MotionWaypoint,
    MotionWaypointOptions
)
from geometry_msgs.msg import Pose, Point, Quaternion

piece_heights = {
    'pawn': 0.05,
    'rook': 0.07,
    'knight': 0.06,
    'bishop': 0.06,
    'queen': 0.08,
    'king': 0.09
}

class ChessWaypointSystem:
    def __init__(self):
        rospy.loginfo("Initializing Sawyer chess waypoint system...")
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path("position_calibration")
        self._yaml_path = os.path.join(pkg_path, "config", "board_positions.yaml")
        self._board_positions = self._load_board_positions()

        self._limb = intera_interface.Limb()
        self._traj = MotionTrajectory()
        self._wpt_opts = MotionWaypointOptions()
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
        chess_notation = msg.notation
        move_uci = msg.uci
        from_square = msg.from_Square
        to_square = msg.to_square
        from_piece = msg.from_piece
        to_piece = msg.to_piece
        rospy.info(f"Executing move: {chess_notation}")
        if '#' in chess_notation:
            self._checkmate()
        elif 'x' in chess_notation: # capture
            self._discard(square=to_square, piece=to_piece) # remove captured piece
            self._move(from_square=from_square, to_square=to_square, from_piece=from_piece) # move piece to new square
        elif '=' in chess_notation:
            self._promote(from_square=from_square, to_square=to_square, from_piece=from_piece, promotion_piece=move_uci[-1])
        elif '-' in chess_notation:
            self._castle(notation=chess_notation)
        else:
            self._move(from_square=from_square, to_square=to_square, from_piece=from_piece)
        move = self._traj.send_trajectory()
        if move is None:
            rospy.loginfo(f"Failed to execute move: {chess_notation}")
        else:
            rospy.loginfo(f"Successfully executed move: {chess_notation}")


    def _move(self, from_square, to_square, piece):
        self._append_waypoint(self._board_positions[from_square]['joint_angles']) # go to original piece square
        self._pick(piece=piece) # pick up piece
        self._append_waypoint(self._board_positions['base']['joint_angles']) # move to base position
        self._append_waypoint(self._board_positions[to_square]['joint_angles']) # go to new piece square
        self._pick(piece=piece, release=True) # release piece
        self._append_waypoint(self._board_positions['base']['joint_angles']) # move to base position

    def _discard(self, square, piece):
        """
        Discards a piece by moving it to a designated discard area.

        :param square: the square from which the piece is being discarded (e.g., 'e4')
        :param piece: the type of chess piece being discarded (e.g., 'pawn', 'rook', 'knight', 'bishop', 'queen', 'king')
        """
        self._append_waypoint(self._board_positions[square]['joint_angles']) # go to piece square
        self._pick(piece=piece) # pick up piece
        self._append_waypoint(self._board_positions['discard']['joint_angles']) # go to discard position
        self._pick(piece=piece, release=True) # release piece

    def _pick(self, square, piece, release=False):
        """
        Lower or raise the end effector by a pre-determined amount based on the piece type and provides the option to grasp or release the piece.
        
        :param piece: the type of chess piece being manipulated (e.g., 'pawn', 'rook', 'knight', 'bishop', 'queen', 'king')
        :param release: whether to lower and grab the piece (False) or to lower and release the piece (True), defaults to False
        """
        square_point = self._board_positions[square]['position']
        pick_pose = Pose()
        pick_point = Point()
        pick_point.x = square_point[0]
        pick_point.y = square_point[1] - piece_heights[piece]
        pick_point.z = square_point[2]
        pick_pose.position = pick_point
        # pick_pose.orientation = Quaternion(0, 1, 0, 0)
        pick_pose.orientation = self._board_positions[square]['orientation']

        joint_solution = self._limb.ik_request(pick_pose)
        if joint_solution:
            self._append_waypoint(joint_solution)
        else:
            rospy.logerr("No IK solution found for pick pose of piece %s at square %s", piece, square)
        if release:
            # turn magnet off to release piece
        else:
            # turn magnet on to grasp piece
        pick_point.y = square_point[1] + piece_heights[piece]
        joint_solution = self._limb.ik_request(pick_pose)
        if joint_solution:
            self._append_waypoint(angles = joint_solution)
        else:
            rospy.logerr("No IK solution found for raised pose of piece %s at square %s", piece, square)
        self._append_waypoint(self._board_positions['base']['joint_angles']) # move to base position after pick/release

    def _promote(self, from_square, to_square, from_piece, promotion_piece):
        self._move(from_square=from_square, to_square=to_square, from_piece=from_piece, piece="pawn") # move pawn to promotion square
        self._discard(square=to_square, piece="pawn") # discard pawn
        self._move(from_square=promotion_piece + "_promote", to_square=to_square, piece=promotion_piece) # move promotion piece to promotion square
    
    def _castle(self, notation):
        if notation == "O-O":
            # kingside castle logic
        elif notation == "O-O-O":
            # queenside castle logic

    def _checkmate(self):

    # add type checking logic (is of type Pose, or type list with 7 joint angles?)
    def _append_waypoint(self, angles):
        self._wpt.set_joint_angles(joint_angles = angles)
        self._traj.append_waypoint(self._wpt)

def main():
    rospy.init_node("chess_waypoint_system")
    chess_waypoint_system = ChessWaypointSystem()
    rospy.spin()

if __name__ == "__main__":
    main()