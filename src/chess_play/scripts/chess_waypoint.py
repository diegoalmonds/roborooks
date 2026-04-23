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
    'R': 0.061,
    'N': 0.048,
    'B': 0.039,
    'Q': 0.0327,
    'K': 0.0270
}

JOINT_KEYS = ['right_j0', 'right_j1', 'right_j2', 'right_j3', 'right_j4', 'right_j5', 'right_j6']

PORT = "/dev/ttyACM0"
BAUD = 115200

# FIX: serial port is opened inside __init__ after ROS is initialised,
# so startup failures produce a proper rospy error instead of a bare crash.
ser = None

def magnet_on():
    cmd = "PIN 9 ON\n"
    ser.write(cmd.encode())
    rospy.loginfo("MAGNET ON")

def magnet_off():
    cmd = "PIN 9 OFF\n"
    ser.write(cmd.encode())
    rospy.loginfo("MAGNET OFF")

class ChessWaypointSystem:
    def __init__(self):
        rospy.loginfo("Initializing Sawyer chess waypoint system...")

        # FIX: open serial port here, after rospy.init_node, so errors are
        # captured by the ROS logging system and the node shuts down cleanly.
        global ser
        try:
            ser = serial.Serial(PORT, BAUD, timeout=1)
            rospy.loginfo(f"Serial port {PORT} opened at {BAUD} baud.")
        except serial.SerialException as e:
            rospy.logerr(f"Failed to open serial port {PORT}: {e}")
            raise

        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path("position_calibration")
        self._yaml_path = os.path.join(pkg_path, "config", "board_positions.yaml")
        self._board_positions = self._load_yaml()

        self._limb = intera_interface.Limb()
        # FIX: convert dict_values to a plain list immediately so downstream
        # code that expects an indexable sequence works consistently.
        self._current_angles = list(self._limb.joint_angles().values())
        rospy.loginfo(f"Current joint angles: {self._current_angles}")
        self._wpt_opts = MotionWaypointOptions(max_joint_speed_ratio=0.3)
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
        rospy.loginfo(str(msg))
        move_type = msg.move_type
        move_notation = msg.notation
        from_square = msg.from_square
        to_square = msg.to_square
        from_piece = msg.from_piece.upper()
        to_piece = msg.to_piece.upper()
        promotion = msg.promotion_piece
        checkmate = msg.is_checkmate

        # FIX: a checkmate flag-only message (empty move_type) triggers the
        # end sequence without trying to execute a move.
        if checkmate and not move_type:
            rospy.loginfo("Checkmate flag received — shutting down.")
            rospy.signal_shutdown("CHECKMATE - shutting down waypoint system.")
            return

        rospy.loginfo(f"Executing move: {move_notation}")
        if move_type == 'capture':
            self._discard(square=to_square, piece=to_piece)
            self._move(from_square=from_square, to_square=to_square, piece=from_piece)
        elif move_type == 'promotion':
            # to_piece is non-empty when the promotion is also a capture.
            self._promote(
                from_square=from_square,
                to_square=to_square,
                from_piece=from_piece,
                promotion_piece=promotion,
                capture=bool(to_piece),
                captured_piece=to_piece
            )
        else:  # castle or regular move
            self._move(from_square=from_square, to_square=to_square, piece=from_piece)

        if checkmate:
            rospy.loginfo("Checkmate detected.")
            rospy.signal_shutdown("CHECKMATE - shutting down waypoint system.")

    def _move(self, from_square, to_square, piece):
        rospy.loginfo(f"MOVE - from {from_square} to {to_square}")

        # Route through "away" before every square approach so the arm always
        # travels at a safe height and never cuts through pieces on the board.
        self._send_single_waypoint(square="away")
        self._send_single_waypoint(square=from_square)
        self._pick(piece=piece, square=from_square)

        self._send_single_waypoint(square="away")
        self._send_single_waypoint(square=to_square)
        self._pick(piece=piece, square=to_square, release=True)

        # Return to away after the full move
        self._send_single_waypoint(square="away")

    def _discard(self, square, piece):
        """
        Discards a piece by moving it to a designated discard area.

        :param square: the square from which the piece is being discarded (e.g., 'e4')
        :param piece: the type of chess piece being discarded (e.g., 'P', 'R', 'N', 'B', 'Q', 'K')
        """
        rospy.loginfo(f"DISCARD - piece {piece} at square {square}")

        self._send_single_waypoint(square="away")
        self._send_single_waypoint(square=square)
        self._pick(piece=piece, square=square)

        self._send_single_waypoint(square="away")
        self._send_single_waypoint(square="discard")
        self._pick(piece=piece, square="discard", release=True)
        self._send_single_waypoint(square="away")

    def _pick(self, square, piece, release=False):
        """
        Lower or raise the end effector by a pre-determined amount based on the
        piece type and provide the option to grasp or release the piece.

        :param square: the square at which the piece is located (e.g., 'e4')
        :param piece: the type of chess piece being manipulated (e.g., 'P', 'R', 'N', 'B', 'Q', 'K')
        :param release: if False, lower and grab; if True, lower and release. Defaults to False.
        """
        rospy.loginfo(f"PICK - {'releasing' if release else 'grabbing'} piece {piece} at square {square}")

        # create joint dictionary for IK seed
        ik_seed = dict(zip(JOINT_KEYS, self._current_angles))

        pos = self._board_positions[square]['position']
        pick_pose = Pose()

        # FIX: the original formula simplified to (0.36 - piece_heights[piece]),
        # making pos['z'] irrelevant and ignoring board-level variation across
        # squares. Correct formula: lower from the calibrated hover height by
        # the piece's physical height.
        BASELINE_HOVER_Z = 0.36
        pick_z = BASELINE_HOVER_Z - piece_heights[piece]
        pick_pose.position = Point(
            x=pos['x'],
            y=pos['y'],
            z=pick_z
        )

        ori = self._board_positions[square]['orientation']
        pick_pose.orientation = Quaternion(
            x=ori['x'],
            y=ori['y'],
            z=ori['z'],
            w=ori['w']
        )

        joint_solution = self._limb.ik_request(pick_pose, joint_seed=ik_seed)
        if joint_solution:
            angles = list(joint_solution.values())
            self._send_single_waypoint(angles=angles)
            self._current_angles = angles
        else:
            rospy.logerr("No IK solution found for pick pose of piece %s at square %s", piece, square)

        # toggle magnet based on pick or release
        if release:
            magnet_off()
        else:
            magnet_on()

        # Raise end effector back to hover height to avoid knocking over pieces
        pick_pose.position.z = pos['z']
        ik_seed = dict(zip(JOINT_KEYS, self._current_angles))
        joint_solution = self._limb.ik_request(pick_pose, joint_seed=ik_seed)

        if joint_solution:
            angles = list(joint_solution.values())
            self._send_single_waypoint(angles=angles)
            self._current_angles = angles
        else:
            rospy.logerr("No IK solution found for raised pose of piece %s at square %s", piece, square)

        # FIX: removed the trailing _send_single_waypoint("base") that was here.
        # _move() already calls _send_single_waypoint("away") after each _pick,
        # so this caused a redundant double-reset on every pick/place operation.

    def _promote(self, from_square, to_square, from_piece, promotion_piece, capture=False, captured_piece=""):
        # If the promotion is also a capture, discard the enemy piece on to_square first.
        if capture and captured_piece:
            self._discard(square=to_square, piece=captured_piece)

        # Move the pawn to the promotion square, then discard it.
        self._move(from_square=from_square, to_square=to_square, piece="P")
        self._discard(square=to_square, piece="P")

        # Place the promoted piece from its staging position.
        # YAML keys: <Piece><color>_promote, e.g. 'Qb_promote', 'Rw_promote'.
        # White promotes on rank 8, black on rank 1.
        color_suffix = 'w' if to_square[1] == '8' else 'b'
        staging_square = promotion_piece.upper() + color_suffix + "_promote"
        self._move(
            from_square=staging_square,
            to_square=to_square,
            piece=promotion_piece
        )

    def _checkmate(self, from_square=None, to_square=None, piece=None):
        # TODO: define a celebratory end-of-game motion sequence here.
        pass

    def _send_single_waypoint(self, square=None, angles=None, pause=0.5):
        rospy.loginfo(f"Sending single waypoint for position: {square if square else 'joint angles'}")

        if square:
            pos = self._board_positions[square]['position']
            ori = self._board_positions[square]['orientation']
            waypoint_pose = Pose()
            waypoint_pose.position = Point(
                x=pos['x'],
                y=pos['y'],
                z=pos['z']
            )
            waypoint_pose.orientation = Quaternion(
                x=ori['x'],
                y=ori['y'],
                z=ori['z'],
                w=ori['w']
            )

            ik_seed = dict(zip(JOINT_KEYS, self._current_angles))
            joint_solution = self._limb.ik_request(waypoint_pose, joint_seed=ik_seed)
            if joint_solution:
                angles = list(joint_solution.values())
            else:
                rospy.logerr("No IK solution found for pose at square %s", square)
                return

        traj = MotionTrajectory()
        wpt = MotionWaypoint(options=self._wpt_opts, limb=self._limb)
        wpt.set_joint_angles(joint_angles=angles)
        traj.append_waypoint(wpt)

        result = traj.send_trajectory()
        self._current_angles = angles

        if result is None:
            rospy.logerr("Trajectory execution returned None")
        elif result.result:
            rospy.loginfo("Waypoint executed successfully")
        else:
            rospy.logerr("Waypoint execution failed")

        rospy.sleep(pause)
        return angles

def main():
    rospy.init_node("chess_waypoint_system")
    chess_waypoint_system = ChessWaypointSystem()
    rospy.spin()
    # FIX: close serial port only if it was successfully opened.
    if ser is not None:
        ser.close()

if __name__ == "__main__":
    main()