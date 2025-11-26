#!/usr/bin/env python3
import time
import threading
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Pose

from pymoveit2 import MoveIt2
from pymoveit2.gripper_interface import GripperInterface


def make_pose(x, y, z, qx, qy, qz, qw) -> Pose:
    p = Pose()
    p.position.x, p.position.y, p.position.z = x, y, z
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = qx, qy, qz, qw
    return p


class PickAndPlaceNode(Node):
    def __init__(self):
        super().__init__("pick_and_place")

        # Parameters
        self.declare_parameter("task", "place_by_color")
        # Default order: Red, Green, Blue, Yellow
        self.declare_parameter("order", "red green blue yellow")

        # MoveIt Setup
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"],
            base_link_name="base_link",
            end_effector_name="end_effector_link",
            group_name="arm",
        )

        # Gripper Setup
        try:
            self.gripper = GripperInterface(
                node=self,
                gripper_joint_names=["right_finger_bottom_joint"],
                open_gripper_joint_positions=[0.45],
                closed_gripper_joint_positions=[0.01],
                gripper_group_name="gripper",
                gripper_command_action_name="/gen3_lite_2f_gripper_controller/gripper_cmd",
                ignore_new_calls_while_executing=False,
            )
        except Exception as e:
            self.get_logger().warn(f"GripperInterface initialization failed: {e}")
            self.gripper = None

        # ---------------- Coordinates Configuration ----------------
        
        # Heights
        # Table is at 0.30. Block is 0.04.
        # Center of block = 0.30 + 0.02 = 0.32
        self.block_z = 0.02
        
        self.approach_height = 0.35  # Safe height to fly over
        self.grasp_height    = 0.025  # Grip height (tune this if gripper misses)
        self.place_height_z  = 0.04  # Place slightly above board (0.305 + 0.02 + buffer)

        # self.top_down_orientation = (-0.7071, 0.7071, 0.0, 0.0)
        self.top_down_orientation = (1.0, 0.0, 0.0, 0.0)
        self.j_retract = [0.40, 0.02, 2.27, -1.57, -0.84, 1.97]
        self.touch_links = ["end_effector_link", "right_finger_bottom_link", "left_finger_bottom_link"]

        # 1. Pick Locations (The 24 cubes on the right)
        # Matches your .sh script logic:
        # COLOR_START_X="-0.20", SPACING="0.065"
        # BLOCK_START_Y="-0.25", SPACING="-0.065"
        
        self.pick_locations = {
            "red": [], "blue": [], "yellow": [], "green": [], "purple": [], "black": []
        }
        # Order must match your spawn script loop order (row 0 to 5)
        spawn_order = ["red", "blue", "yellow", "green", "purple", "black"]

        #--------24 boxes--------
        
        # x_start = -0.20
        # x_spacing = 0.06
        # y_start = -0.25
        # y_spacing = -0.07 # Moving negative Y

        # for i, color in enumerate(spawn_order):
        #     x = x_start + (i * x_spacing)
        #     for j in range(4):
        #         y = y_start + (j * y_spacing)
        #         self.pick_locations[color].append([x, y, self.block_z])

        # ---------6 boxes test--------
        x_start = -0.16
        x_spacing = 0.08
        fixed_y = -0.25
        
        for i, color in enumerate(spawn_order):
            # Calculate X: Start + (index * spacing)
            # i=0 (Red)    -> -0.16
            # i=1 (Blue)   -> -0.08
            # i=2 (Yellow) ->  0.00
            # ...
            x = x_start + (i * x_spacing)
            y = fixed_y
            
            # Append coordinate [x, y, z]
            self.pick_locations[color].append([x, y, self.block_z])
        #-------------------

        # 2. Place Locations (The 4 spots on the Gameboard)
        # Board at X=0.30, Y=0.0
        # We want 4 spots from Left(Y+) to Right(Y-)
        self.place_locations = [
            [0.30,  0.09, self.place_height_z], # Spot 1
            [0.30,  0.03, self.place_height_z], # Spot 2
            [0.30, -0.03, self.place_height_z], # Spot 3
            [0.30, -0.09, self.place_height_z], # Spot 4
        ]

    # ---------------------- Utils ----------------------
    def move_to_joints(self, joints):
        self.moveit2.move_to_configuration(joint_positions=joints)
        self.moveit2.wait_until_executed()

    def move_to_pose(self, pose: Pose, cartesian: bool = False) -> bool:
        traj = self.moveit2.plan(pose=pose, cartesian=cartesian, max_step=0.005)
        if traj is None:
            self.get_logger().error("Planning failed.")
            return False
        self.moveit2.execute(traj)
        self.moveit2.wait_until_executed()
        return True

    def open_gripper(self):
        if self.gripper: self.gripper.open()

    def close_gripper(self):
        if self.gripper: self.gripper.close()

    # -------------------- Task Logic --------------------
    
    def task_add_scene(self):
        # Add table collision object to avoid hitting it
        # self.moveit2.add_collision_box(
        #     id="table_top",
        #     size=(1.0, 1.5, 0.05), # Updated size from your .sh
        #     position=(0.0, 0.0, -0.05),
        #     quat_xyzw=(0, 0, 0, 1),
        #     frame_id="base_link",
        # )
        
        # # Add Gameboard collision (approximation)
        # self.moveit2.add_collision_box(
        #     id="gameboard",
        #     size=(0.35, 0.50, 0.02),
        #     position=(0.30, 0.0, 0.0),
        #     quat_xyzw=(0, 0, 0, 1),
        #     frame_id="base_link",
        # )
        self.get_logger().info("Planning scene added.")

    def task_place_by_color(self):
        # 1. Parse Order
        order_str = self.get_parameter("order").get_parameter_value().string_value
        # Split string into list
        requested_colors = [c.strip().lower() for c in order_str.split() if c.strip()]
        
        if not requested_colors:
            self.get_logger().warn("No colors specified in 'order' parameter!")
            return

        # We only have 4 spots on the board
        if len(requested_colors) > 4:
            self.get_logger().warn("Only placing the first 4 colors.")
            requested_colors = requested_colors[:4]

        # Track used blocks to avoid picking the same one
        # e.g. {'red': 0} means next red block is at index 0
        used_counts = {color: 0 for color in self.pick_locations}

        self.get_logger().info(f"Starting Task: Place {requested_colors}")
        
        # Start position
        self.move_to_joints(self.j_retract)
        self.open_gripper()
        time.sleep(0.5)

        # 2. Loop through request
        for i, color in enumerate(requested_colors):
            if color not in self.pick_locations:
                self.get_logger().error(f"Invalid color: {color}, skipping.")
                continue
            
            # Check availability
            idx = used_counts[color]
            if idx >= 4: # We only spawned 4 of each
                self.get_logger().warn(f"No more {color} blocks left!")
                continue
            
            # Identify coordinates
            pick_xyz = self.pick_locations[color][idx]
            place_xyz = self.place_locations[i]
            
            # Identify Name (for collision attachment)
            # .sh names are 1-based: mastermind_block_red_1
            block_name = f"mastermind_block_{color}_{idx + 1}"
            
            # Increment counter
            used_counts[color] += 1
            
            self.get_logger().info(f"Moving {block_name} to Spot #{i+1}")
            
            # --- ACTION ---
            self.run_pick_and_place(block_name, pick_xyz, place_xyz)
            
        # Finish
        self.move_to_joints(self.j_retract)
        self.get_logger().info("Task Done.")

    def run_pick_and_place(self, block_id, pick, place):
        # Poses
        pre_pick  = make_pose(pick[0], pick[1], self.approach_height, *self.top_down_orientation)
        grasp     = make_pose(pick[0], pick[1], self.grasp_height,    *self.top_down_orientation)

        pre_place = make_pose(place[0], place[1], self.approach_height, *self.top_down_orientation)
        drop      = make_pose(place[0], place[1], place[2],           *self.top_down_orientation)

        # Go Pick
        self.get_logger().info(f"Going to pre-pick...")
        self.move_to_pose(pre_pick)

        self.get_logger().info(f"Going down to grasp...")
        self.move_to_pose(grasp, cartesian=True)

        squat_pose = make_pose(pick[0], pick[1], self.grasp_height - 0.002, *self.top_down_orientation)
        self.move_to_pose(squat_pose, cartesian=True)

        self.close_gripper()
        time.sleep(0.5)
        
        # Attach object (simulated grasp)
        self.moveit2.attach_collision_object(block_id, "end_effector_link", self.touch_links)
        
        # Lift
        self.move_to_pose(pre_pick, cartesian=True)
        
        # Go Place
        self.move_to_pose(pre_place)
        self.move_to_pose(drop, cartesian=True)
        
        # Drop
        self.open_gripper()
        time.sleep(0.5)
        self.moveit2.detach_collision_object(block_id)
        
        # Retreat
        self.move_to_pose(pre_place, cartesian=True)


def main(args=None):
    rclpy.init(args=args)
    node = PickAndPlaceNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()

    # Add scene objects first to avoid collisions
    node.task_add_scene()
    
    task = node.get_parameter("task").get_parameter_value().string_value
    if task == "place_by_color":
        node.task_place_by_color()
    else:
        node.get_logger().info("Task parameter not set to 'place_by_color'. Doing nothing.")

    time.sleep(1)
    rclpy.shutdown()


if __name__ == "__main__":
    main()