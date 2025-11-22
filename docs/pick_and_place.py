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

        # choose task at runtime
        self.declare_parameter("task", "home")
        # for the new task: space-separated colors, e.g. "red green blue yellow"
        self.declare_parameter("order", "red blue yellow green")

        # --- robot / moveit setup (same as your working version) ---
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"],
            base_link_name="base_link",
            end_effector_name="end_effector_link",
            group_name="arm",
        )

        try:
            self.gripper = GripperInterface(
                node=self,
                gripper_joint_names=["right_finger_bottom_joint"],
                open_gripper_joint_positions=[0.80],
                closed_gripper_joint_positions=[0.01],
                gripper_group_name="gripper",
                gripper_command_action_name="/gen3_lite_2f_gripper_controller/gripper_cmd",
                ignore_new_calls_while_executing=False,
            )
        except Exception as e:
            self.get_logger().warn(f"GripperInterface initialization failed: {e}")
            self.gripper = None

        # ---------------- world layout (lab06 + green) ----------------
        table_z = -0.0001
        block_size = 0.04
        self.table_center_z = table_z - 0.05 / 2.0
        self.block_center_z = table_z + block_size / 2.0

        block_x = 0.45
        # exactly what your spawn script makes: red, blue, yellow, green
        # y = -0.08, 0.0, +0.08, +0.16
        self.blocks_xyz = [
            (block_x, -0.08, self.block_center_z),   # RED
            (block_x,  0.00, self.block_center_z),   # BLUE
            (block_x,  0.08, self.block_center_z),   # YELLOW
            (block_x,  0.16, self.block_center_z),   # GREEN
        ]

        # "whiteboard line" targets — already working in your setup
        self.drop_spots = [
            (0.20, 0.20),
            (0.20, 0.28),
            (0.20, 0.36),
            (0.20, 0.44),
        ]

        self.approach_height = 0.32
        self.grasp_height = 0.19
        self.top_down_orientation = (-0.7071, 0.7071, 0.0, 0.0)

        # keep the old lab06 helper poses
        pick_xyz = self.blocks_xyz[0]
        self.pre_grasp_pose = make_pose(pick_xyz[0], pick_xyz[1], self.approach_height, *self.top_down_orientation)
        self.grasp_pose     = make_pose(pick_xyz[0], pick_xyz[1], self.grasp_height,    *self.top_down_orientation)

        target_xyz = self.blocks_xyz[1]
        place_z = self.grasp_height + block_size
        self.pre_place_pose = make_pose(target_xyz[0], target_xyz[1], self.approach_height, *self.top_down_orientation)
        self.place_pose     = make_pose(target_xyz[0], target_xyz[1], place_z,               *self.top_down_orientation)

        self.j_home    = [0.0] * 6
        self.j_retract = [0.40, 0.02, 2.27, -1.57, -0.84, 1.97]
        self.touch_links = [
            "end_effector_link",
            "right_finger_bottom_link",
            "left_finger_bottom_link",
        ]

        self.block_size = block_size
        self.center_idx = 1
        self.release_up_dz = 0.01

        # color -> index in blocks_xyz
        # (must match the spawn order you just showed)
        self.color_to_index = {
            "red": 0,
            "blue": 1,
            "yellow": 2,
            "green": 3,
        }

    # ---------------------- utils ----------------------
    def move_to_joints(self, joints):
        self.moveit2.move_to_configuration(joint_positions=joints)
        self.moveit2.wait_until_executed()

    def move_to_pose(self, pose: Pose, cartesian: bool = False) -> bool:
        traj = self.moveit2.plan(
            pose=pose,
            cartesian=cartesian,
            max_step=0.005,
            cartesian_fraction_threshold=0.90 if cartesian else None,
        )
        if traj is None:
            self.get_logger().error("Planning failed.")
            return False
        self.moveit2.execute(traj)
        self.moveit2.wait_until_executed()
        return True

    def open_gripper(self):
        if self.gripper:
            self.gripper.open()

    def close_gripper(self):
        if self.gripper:
            self.gripper.close()

    def _micro_lift(self, pose: Pose):
        up = make_pose(
            pose.position.x,
            pose.position.y,
            pose.position.z + self.release_up_dz,
            *self.top_down_orientation
        )
        _ = self.move_to_pose(up, cartesian=True)

    # -------------------- planning scene --------------------
    def add_scene(self):
        self.get_logger().info("Adding objects to the planning scene.")
        self.moveit2.add_collision_box(
            id="table_top",
            size=(1.0, 1.0, 0.05),
            position=(0.0, 0.0, self.table_center_z),
            quat_xyzw=(0, 0, 0, 1),
            frame_id="base_link",
        )
        time.sleep(0.1)

        for i, (x, y, z) in enumerate(self.blocks_xyz, start=1):
            self.moveit2.add_collision_box(
                id=f"block_{i}",
                size=(0.04, 0.04, 0.04),
                position=(x, y, z),
                quat_xyzw=(0, 0, 0, 1),
                frame_id="base_link",
            )
            time.sleep(0.05)

        # your board is already added in a separate run earlier — we’re not touching it here
        self.get_logger().info("Planning scene has been set up.")

    # -------------------- old single-block task --------------------
    def task_pick_place_one(self):
        self.get_logger().info("Starting pick and place task for 'block_1'.")
        self.move_to_joints(self.j_retract)
        self.open_gripper()
        time.sleep(1.0)

        if not self.move_to_pose(self.pre_grasp_pose):
            return
        if not self.move_to_pose(self.grasp_pose, cartesian=True):
            return

        self.close_gripper()
        time.sleep(1.0)
        self.moveit2.attach_collision_object("block_1", "end_effector_link", self.touch_links)
        time.sleep(0.2)

        if not self.move_to_pose(self.pre_grasp_pose, cartesian=True):
            return

        if not self.move_to_pose(self.pre_place_pose):
            return
        if not self.move_to_pose(self.place_pose, cartesian=True):
            return

        self.open_gripper()
        time.sleep(1.0)
        self._micro_lift(self.place_pose)
        self.moveit2.detach_collision_object("block_1")
        time.sleep(0.2)

        _ = self.move_to_pose(self.pre_place_pose, cartesian=True)
        self.move_to_joints(self.j_retract)
        self.get_logger().info("Pick and place task completed successfully.")

    # -------------------- existing 4-block line task --------------------
    def task_place_four_in_line(self):
        self.get_logger().info("Starting 4-block line placement (on board).")
        self.move_to_joints(self.j_retract)
        self.open_gripper()
        time.sleep(0.5)

        for idx in range(4):
            block_id = f"block_{idx+1}"
            pick_x, pick_y, _ = self.blocks_xyz[idx]
            drop_x, drop_y = self.drop_spots[idx]

            self.get_logger().info(f"Handling {block_id}")

            pre_pick = make_pose(pick_x, pick_y, self.approach_height, *self.top_down_orientation)
            pick_pose = make_pose(pick_x, pick_y, self.grasp_height,    *self.top_down_orientation)
            pre_drop  = make_pose(drop_x, drop_y, self.approach_height, *self.top_down_orientation)
            drop_pose = make_pose(drop_x, drop_y, self.grasp_height,    *self.top_down_orientation)

            if not self.move_to_pose(pre_pick):
                self.get_logger().error(f"Could not go above {block_id}")
                return
            if not self.move_to_pose(pick_pose, cartesian=True):
                self.get_logger().error(f"Could not descend to {block_id}")
                return

            self.close_gripper()
            time.sleep(0.4)
            self.moveit2.attach_collision_object(block_id, "end_effector_link", self.touch_links)
            time.sleep(0.2)

            if not self.move_to_pose(pre_pick, cartesian=True):
                self.get_logger().error(f"Could not lift {block_id}")
                return

            if not self.move_to_pose(pre_drop):
                self.get_logger().error(f"Could not move above drop for {block_id}")
                return
            if not self.move_to_pose(drop_pose, cartesian=True):
                self.get_logger().error(f"Could not descend to drop for {block_id}")
                return

            self.open_gripper()
            time.sleep(0.4)
            self._micro_lift(drop_pose)
            self.moveit2.detach_collision_object(block_id)
            time.sleep(0.2)

            _ = self.move_to_pose(pre_drop, cartesian=True)

        self.move_to_joints(self.j_retract)
        self.get_logger().info("All 4 blocks placed in a vertical line.")

    # -------------------- NEW: place by color order --------------------
    def task_place_by_color(self):
        # string like "red green blue yellow"
        order_str = self.get_parameter("order").get_parameter_value().string_value
        tokens = [t.strip().lower() for t in order_str.split() if t.strip()]
        if not tokens:
            self.get_logger().warn("No colors given in 'order' param; nothing to do.")
            return

        self.get_logger().info(f"Placing by color order: {tokens}")
        self.move_to_joints(self.j_retract)
        self.open_gripper()
        time.sleep(0.4)

        drop_idx = 0
        for color in tokens:
            if drop_idx >= len(self.drop_spots):
                self.get_logger().warn("More colors than drop spots; stopping.")
                break

            if color not in self.color_to_index:
                self.get_logger().warn(f"Unknown color '{color}', skipping.")
                continue

            block_index = self.color_to_index[color]
            block_id = f"block_{block_index+1}"
            pick_x, pick_y, _ = self.blocks_xyz[block_index]
            drop_x, drop_y = self.drop_spots[drop_idx]

            self.get_logger().info(f"Placing color '{color}' -> {block_id} to spot {drop_idx+1}")

            pre_pick = make_pose(pick_x, pick_y, self.approach_height, *self.top_down_orientation)
            pick_pose = make_pose(pick_x, pick_y, self.grasp_height,    *self.top_down_orientation)
            pre_drop  = make_pose(drop_x, drop_y, self.approach_height, *self.top_down_orientation)
            drop_pose = make_pose(drop_x, drop_y, self.grasp_height,    *self.top_down_orientation)

            if not self.move_to_pose(pre_pick):
                self.get_logger().error(f"Could not go above {block_id}")
                return
            if not self.move_to_pose(pick_pose, cartesian=True):
                self.get_logger().error(f"Could not descend to {block_id}")
                return

            self.close_gripper()
            time.sleep(0.4)
            self.moveit2.attach_collision_object(block_id, "end_effector_link", self.touch_links)
            time.sleep(0.2)

            if not self.move_to_pose(pre_pick, cartesian=True):
                self.get_logger().error(f"Could not lift {block_id}")
                return

            if not self.move_to_pose(pre_drop):
                self.get_logger().error(f"Could not move above drop for {block_id}")
                return
            if not self.move_to_pose(drop_pose, cartesian=True):
                self.get_logger().error(f"Could not descend to drop for {block_id}")
                return

            self.open_gripper()
            time.sleep(0.4)
            self._micro_lift(drop_pose)
            self.moveit2.detach_collision_object(block_id)
            time.sleep(0.2)

            _ = self.move_to_pose(pre_drop, cartesian=True)

            drop_idx += 1

        self.move_to_joints(self.j_retract)
        self.get_logger().info("Color-ordered placement complete.")

    # -------------------- shortcuts --------------------
    def task_home(self): self.move_to_joints(self.j_home)
    def task_retract(self): self.move_to_joints(self.j_retract)
    def task_add_scene(self): self.add_scene()


def main(args=None):
    rclpy.init(args=args)
    node = PickAndPlaceNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()

    task = node.get_parameter("task").get_parameter_value().string_value
    node.get_logger().info(f"Executing task: '{task}'")

    try:
        if task == "home":
            node.task_home()
        elif task == "retract":
            node.task_retract()
        elif task == "add_scene":
            node.task_add_scene()
        elif task == "pick_place_one":
            node.task_pick_place_one()
        elif task == "place_four_in_line":
            node.task_place_four_in_line()
        elif task == "place_by_color":
            node.task_place_by_color()
        else:
            node.get_logger().warn(
                f"Unknown task '{task}'. Valid: home, retract, add_scene, pick_place_one, place_four_in_line, place_by_color"
            )
    finally:
        time.sleep(1)
        rclpy.shutdown()


if __name__ == "__main__":
    main()
