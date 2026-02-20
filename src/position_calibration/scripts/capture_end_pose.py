#!/usr/bin/env python3

import rospy
import rospkg
import intera_interface
import threading
import yaml
import ast
import os

from intera_core_msgs.msg import IODeviceStatus, EndpointState

class EndPoseCapture:
    def __init__(self,
                 button_name="right_button_lower"):
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path("position_calibration")
        self.yaml_path = os.path.join(pkg_path, "config", "board_positions.yaml")
        self.data = {}
        self.mutex = threading.Lock()
        
        self.button_name = button_name
        self.current_state = 0
        self.previous_state = 0
        self.rise = False
        
        self._cuff = intera_interface.Cuff()
        self._limb = intera_interface.Limb()
        self._cuff_state_sub = rospy.Subscriber(
            "/io/robot/cuff/state", 
            IODeviceStatus, 
            self._check_click)
        self._end_pose_sub = rospy.Subscriber(
            "/robot/limb/right/endpoint_state",
            EndpointState,
            self._get_end_position)
        self.latest_position = None
        
    def _get_end_position(self, msg):
        with self.mutex:
            self.latest_position = msg.pose.position
        
    def _check_click(self, msg):
        with self.mutex:
            for signal in msg.signals:
                if signal.name == self.button_name:
                    self.current_state = ast.literal_eval(signal.data)[0]
            if self.current_state == 1 and self.previous_state == 0:
                self.rise = True
            self.previous_state = self.current_state        
        
    def prompt_tile(self):
        while not rospy.is_shutdown():
            tile = input("Enter tile name (e.g. 'a1') or 'q' to quit: ")
            if tile == 'q':
                return None
            elif tile:
                return tile
            
    def _load_yaml(self):
        with open(self.yaml_path, "r") as f:
            self.data = yaml.safe_load(f) or {}
            
    def _save_yaml(self):
        tmp_path = self.yaml_path + ".tmp"
        with open(tmp_path, "w") as f:
            yaml.safe_dump(self.data, f, default_flow_style=False, sort_keys=True)
        os.replace(tmp_path, self.yaml_path)
        
    def capture_tile_position(self, tile):
        rospy.loginfo("Beginning capture process..")
        rate = rospy.Rate(100)
        
        while not rospy.is_shutdown():
            with self.mutex:
                pressed = self.rise
                self.rise = False
                pose = self.latest_position
            if pressed:
                if pose is None:
                    rospy.logwarn("No pose data available.")
                    return
                self._load_yaml()
                self.data[tile] = {
                    "x": pose.x,
                    "y": pose.y,
                    "z": pose.z
                }
                rospy.loginfo(f"Captured {tile}: {self.data[tile]}")
                self._save_yaml()
                return
            
            rate.sleep()
                        
if __name__ == "__main__":
    rospy.init_node("end_pose_capture")
    capturer = EndPoseCapture(
            button_name="right_button_lower",
            yaml_path=""
    )
    rospy.loginfo("End pose capture node initialized.")
    
    try:
        while not rospy.is_shutdown():
            tile = capturer.prompt_tile()
            if tile is None:
                break
            capturer.capture_tile_position(tile)
    except KeyboardInterrupt:
        pass
    rospy.loginfo("Shutting down end pose capture node.")