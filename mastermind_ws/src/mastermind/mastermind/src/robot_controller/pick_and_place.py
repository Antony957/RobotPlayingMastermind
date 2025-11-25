#!/usr/bin/env python3

import os
import subprocess
import threading
import time

import rclpy
from geometry_msgs.msg import Pose
from pymoveit2 import MoveIt2
from pymoveit2.gripper_interface import GripperInterface
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from mastermind_interfaces.msg import Code, Status

from ..game_state.game_state import COLOR_TO_NUM, GAME_STATUS, NUM_TO_COLOR


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
        self.declare_parameter("order", "red blue yellow green")

        # plate height offset (from your spawn)
        self.declare_parameter("plate_height_offset", 0.01)

        # wait times for gripper actions
        self.declare_parameter("close_wait_sec", 1.2)
        self.declare_parameter("open_wait_sec", 0.4)

        # extra release clearance above plate to avoid tilted block clipping through
        self.declare_parameter("place_clearance", 0.02)  # 2 cm

        # Gazebo world name (your spawn uses "empty")
        self.declare_parameter("world", "empty")

        # Gazebo block center height (from your spawn_blocks.sh)
        # TABLE_TOP_Z=0.30, BLOCK_SIZE=0.04, BLOCK_DROP=0.01 -> BLOCK_Z = 0.33
        self.declare_parameter("gz_block_center_z", 0.33)

        self.close_wait_sec = (
            self.get_parameter("close_wait_sec").get_parameter_value().double_value
        )
        self.open_wait_sec = (
            self.get_parameter("open_wait_sec").get_parameter_value().double_value
        )
        self.place_clearance = (
            self.get_parameter("place_clearance").get_parameter_value().double_value
        )
        self.world_name = self.get_parameter("world").get_parameter_value().string_value
        self.gz_block_center_z = (
            self.get_parameter("gz_block_center_z").get_parameter_value().double_value
        )

        # --- robot / moveit setup ---
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=[
                "joint_1",
                "joint_2",
                "joint_3",
                "joint_4",
                "joint_5",
                "joint_6",
            ],
            base_link_name="base_link",
            end_effector_name="end_effector_link",
            group_name="arm",
        )

        # ---------------- world layout ----------------
        table_z = -0.0001
        block_size = 0.04

        self.table_center_z = table_z - 0.05 / 2.0
        self.block_center_z = table_z + block_size / 2.0

        block_x = 0.45

        # block poses MUST match your spawn_blocks.sh
        self.blocks_xyz = [
            (block_x, -0.08, self.block_center_z),  # block_1: RED
            (block_x, 0.00, self.block_center_z),  # block_2: BLUE
            (block_x, 0.08, self.block_center_z),  # block_3: YELLOW
            (block_x, 0.16, self.block_center_z),  # block_4: GREEN
            (0.33, 0.16, self.block_center_z),  # block_5: PURPLE
            (0.33, 0.00, self.block_center_z),  # block_6: BLACK
        ]

        # UPDATED drop spots you already tuned
        self.drop_spots = [
            (0.20, 0.21),
            (0.20, 0.28),
            (0.20, 0.35),
            (0.20, 0.42),
        ]

        self.approach_height = 0.32
        self.grasp_height = 0.19

        # placing height lifted to match plate being higher/thicker
        self.plate_height_offset = (
            self.get_parameter("plate_height_offset").get_parameter_value().double_value
        )
        self.place_height = self.grasp_height + self.plate_height_offset

        self.top_down_orientation = (-0.7071, 0.7071, 0.0, 0.0)

        # helper poses (based on first block)
        pick_xyz = self.blocks_xyz[0]
        self.pre_grasp_pose = make_pose(
            pick_xyz[0], pick_xyz[1], self.approach_height, *self.top_down_orientation
        )
        self.grasp_pose = make_pose(
            pick_xyz[0], pick_xyz[1], self.grasp_height, *self.top_down_orientation
        )

        target_xyz = self.blocks_xyz[1]

        # place_z uses clearance
        place_z = self.place_height + self.place_clearance
        self.pre_place_pose = make_pose(
            target_xyz[0],
            target_xyz[1],
            self.approach_height,
            *self.top_down_orientation,
        )
        self.place_pose = make_pose(
            target_xyz[0], target_xyz[1], place_z, *self.top_down_orientation
        )

        self.j_home = [0.0] * 6
        self.j_retract = [0.33, 0.02, 2.27, -1.57, -0.84, 1.97]

        self.touch_links = [
            "end_effector_link",
            "right_finger_bottom_link",
            "left_finger_bottom_link",
        ]

        self.block_size = block_size
        self.release_up_dz = 0.03

        # ---------------- GRIPPER SETUP ----------------
        action_name = self._auto_find_gripper_action()
        self.get_logger().info(f"Using gripper action: {action_name}")

        try:
            self.gripper = GripperInterface(
                node=self,
                gripper_joint_names=["right_finger_bottom_joint"],
                open_gripper_joint_positions=[0.75],
                closed_gripper_joint_positions=[0.00],
                gripper_command_action_name=action_name,
                ignore_new_calls_while_executing=False,
            )
        except Exception as e:
            self.get_logger().warn(f"GripperInterface initialization failed: {e}")
            self.gripper = None

        # ---------------- RESPAWN SUPPORT ----------------
        # spawn_blocks.sh writes SDFs here
        self.sdf_dir = "/tmp/lab06_spawn"
        self.color_to_sdf = {
            "red": os.path.join(self.sdf_dir, "cube_red.sdf"),
            "blue": os.path.join(self.sdf_dir, "cube_blue.sdf"),
            "yellow": os.path.join(self.sdf_dir, "cube_yellow.sdf"),
            "green": os.path.join(self.sdf_dir, "cube_green.sdf"),
            "purple": os.path.join(self.sdf_dir, "cube_purple.sdf"),
            "black": os.path.join(self.sdf_dir, "cube_black.sdf"),
        }

        # Gazebo base model names from spawn_blocks.sh
        self.color_to_gz_base = {
            "red": "lab06_block_red",
            "blue": "lab06_block_blue",
            "yellow": "lab06_block_yellow",
            "green": "lab06_block_green",
            "purple": "lab06_block_purple",
            "black": "lab06_block_black",
        }

        # Original pickup xyz per color (use same x,y each time)
        self.color_spawn_xyz = {c: self.blocks_xyz[i] for c, i in COLOR_TO_NUM.items()}

        # Queue of available MoveIt collision IDs per color
        # Start with original block_1..block_6 IDs
        self.color_pick_queue = {
            "red": ["block_1"],
            "blue": ["block_2"],
            "yellow": ["block_3"],
            "green": ["block_4"],
            "purple": ["block_5"],
            "black": ["block_6"],
        }

        # Counter to make unique respawned names
        self.spawn_counter = {c: 1 for c in COLOR_TO_NUM.keys()}

        # detect gz long-flag support (like your spawn script)
        self.gz_use_long_flags = self._gz_has_long_flags()

        # Pub/sub
        self.game_status_pub = self.create_publisher(Status, "game_status", 10)
        self.submit_code_sub = self.create_subscription(
            Code, "submit_code", self.handle_code, 10
        )

    def handle_code(self, msg: Code):
        """
        Callback for submit_code topic subscriber.
        Task:
            - Parse Code message to make sure author isn't player_1
            - Turn code integers to code string, e.g., "red blue red blue"
            - Run task_place_by_color(colors_str) with color string
        """
        player_name = msg.player_name
        code = msg.code

        if player_name == "player_1":
            return

        self.get_logger().info(f"Code {code} received from computer_vision")

        # Turn numeric code to string and pass to pick and place method
        colors = [NUM_TO_COLOR[c] for c in code]
        colors_str = " ".join(colors)
        self.task_place_by_color(colors_str)

    def publish_game_status(self, status: int):
        """
        Publish given status from sender 'robot_arm'.
        """
        msg = Status()
        msg.sender = "robot_arm"
        msg.status = status
        self.game_status_pub.publish(msg)

    # ---------------------- GRIPPER AUTO-DETECT ----------------------
    def _auto_find_gripper_action(self) -> str:
        fallback = "/gen3_lite_2f_gripper_controller/gripper_cmd"
        try:
            acts = self.get_action_names_and_types()
            names = [a[0] for a in acts]
        except Exception:
            return fallback

        preferred = [
            "/gen3_lite_2f_gripper_controller/gripper_cmd",
            "/gripper_controller/gripper_cmd",
            "/robotiq_gripper_controller/gripper_cmd",
        ]
        for p in preferred:
            if p in names:
                return p
        for n in names:
            low = n.lower()
            if "gripper_cmd" in low or ("gripper" in low and "cmd" in low):
                return n
        return fallback

    # ---------------------- gz helpers ----------------------
    def _gz_has_long_flags(self) -> bool:
        try:
            out = subprocess.check_output(["gz", "service", "--help"], text=True)
            return "--reqtype" in out
        except Exception:
            return True

    def _gazebo_spawn(
        self, sdf_path: str, model_name: str, x: float, y: float, z: float
    ):
        payload = (
            f'sdf_filename: "{sdf_path}"\n'
            f'name: "{model_name}"\n'
            f"pose {{ position {{ x: {x} y: {y} z: {z} }} }}\n"
            f"allow_renaming: false\n"
        )

        if self.gz_use_long_flags:
            cmd = [
                "gz",
                "service",
                "-s",
                f"/world/{self.world_name}/create",
                "--reqtype",
                "gz.msgs.EntityFactory",
                "--reptype",
                "gz.msgs.Boolean",
                "--timeout",
                "3000",
                "--req",
                payload,
            ]
        else:
            cmd = [
                "gz",
                "service",
                "-s",
                f"/world/{self.world_name}/create",
                "-m",
                "gz.msgs.EntityFactory",
                "-r",
                "gz.msgs.Boolean",
                "-t",
                "3000",
                "-p",
                payload,
            ]

        try:
            subprocess.run(
                cmd, check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
            )
            self.get_logger().info(
                f"[respawn] Spawned {model_name} at ({x:.3f},{y:.3f},{z:.3f})"
            )
        except Exception as e:
            self.get_logger().warn(
                f"[respawn] Gazebo spawn failed for {model_name}: {e}"
            )

    def _respawn_color(self, color: str):
        if color not in self.color_to_sdf:
            return

        # use original x,y from pickup spot
        x, y, _ = self.color_spawn_xyz[color]

        # Gazebo needs the real world Z (~0.33) from your spawn script
        gz_z = self.gz_block_center_z

        sdf = self.color_to_sdf[color]
        base = self.color_to_gz_base[color]

        k = self.spawn_counter[color]
        self.spawn_counter[color] += 1

        gz_name = f"{base}_{k}"
        moveit_id = f"{color}_respawn_{k}"

        # 1) Spawn in Gazebo with unique name
        self._gazebo_spawn(sdf, gz_name, x, y, gz_z)

        # 2) Add collision box to MoveIt at your planning Z (keep consistent)
        self.moveit2.add_collision_box(
            id=moveit_id,
            size=(0.04, 0.04, 0.04),
            position=(x, y, self.block_center_z),
            quat_xyzw=(0, 0, 0, 1),
            frame_id="base_link",
        )
        time.sleep(0.05)

        # 3) Queue it for future picks
        self.color_pick_queue[color].append(moveit_id)
        self.get_logger().info(f"[respawn] Ready next {color}: {moveit_id}")

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
            ec = (
                self.moveit2._planning_result.error_code.val
                if hasattr(self.moveit2, "_planning_result")
                else None
            )
            self.get_logger().warn(f"Planning failed! Error code: {ec}.")
            self.get_logger().error("Planning failed.")
            return False
        self.moveit2.execute(traj)
        self.moveit2.wait_until_executed()
        return True

    def open_gripper(self) -> bool:
        if not self.gripper:
            return False
        try:
            self.gripper.open()
            return True
        except Exception as e:
            self.get_logger().error(f"Gripper open failed: {e}")
            return False

    def close_gripper(self) -> bool:
        if not self.gripper:
            return False
        try:
            self.gripper.close()
            return True
        except Exception as e:
            self.get_logger().error(f"Gripper close failed: {e}")
            return False

    def _micro_lift(self, pose: Pose):
        up = make_pose(
            pose.position.x,
            pose.position.y,
            pose.position.z + self.release_up_dz,
            *self.top_down_orientation,
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

        self.get_logger().info("Planning scene has been set up.")

    # -------------------- place by color order (supports duplicates + respawn) --------------------
    def task_place_by_color(self, order_str=None):
        if order_str is None:
            order_str = self.get_parameter("order").get_parameter_value().string_value

        tokens = [t.strip().lower() for t in order_str.split() if t.strip()]
        if not tokens:
            self.get_logger().warn("No colors given in 'order' param; nothing to do.")
            return

        self.get_logger().info(f"Placing by color order: {tokens}")
        self.move_to_joints(self.j_retract)
        self.open_gripper()
        time.sleep(self.open_wait_sec)

        drop_idx = 0
        for color in tokens:
            if drop_idx >= len(self.drop_spots):
                self.get_logger().warn("More colors than drop spots; stopping.")
                break

            if color not in self.color_pick_queue:
                self.get_logger().warn(f"Unknown color '{color}', skipping.")
                continue

            if len(self.color_pick_queue[color]) == 0:
                self.get_logger().warn(f"No available '{color}' blocks to pick.")
                continue

            # next available collision object for that color
            block_id = self.color_pick_queue[color].pop(0)

            pick_x, pick_y, _ = self.color_spawn_xyz[color]
            drop_x, drop_y = self.drop_spots[drop_idx]

            self.get_logger().info(
                f"Placing color '{color}' -> {block_id} to spot {drop_idx+1}"
            )

            pre_pick = make_pose(
                pick_x, pick_y, self.approach_height, *self.top_down_orientation
            )
            pick_pose = make_pose(
                pick_x, pick_y, self.grasp_height, *self.top_down_orientation
            )
            pre_drop = make_pose(
                drop_x, drop_y, self.approach_height, *self.top_down_orientation
            )

            drop_pose = make_pose(
                drop_x,
                drop_y,
                self.place_height + self.place_clearance,
                *self.top_down_orientation,
            )

            if not self.move_to_pose(pre_pick):
                self.get_logger().error(f"Could not go above {block_id}")
                return
            if not self.move_to_pose(pick_pose, cartesian=True):
                self.get_logger().error(f"Could not descend to {block_id}")
                return

            squat_pose = make_pose(
                pick_x, pick_y, self.grasp_height - 0.002, *self.top_down_orientation
            )
            _ = self.move_to_pose(squat_pose, cartesian=True)

            ok = self.close_gripper()
            time.sleep(0.1)
            if not ok:
                self.get_logger().warn("Close gripper failed, attaching anyway.")
            self.moveit2.attach_collision_object(
                block_id, "end_effector_link", self.touch_links
            )

            time.sleep(self.close_wait_sec)

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
            time.sleep(self.open_wait_sec)
            self._micro_lift(drop_pose)

            # Detach and REMOVE from MoveIt scene so the placed block stays on plate
            self.moveit2.detach_collision_object(block_id)
            self.moveit2.remove_collision_object(block_id)
            time.sleep(0.1)

            # Respawn same color on the Gazebo table + add to MoveIt queue
            self._respawn_color(color)

            _ = self.move_to_pose(pre_drop, cartesian=True)

            drop_idx += 1

        self.move_to_joints(self.j_retract)
        self.get_logger().info("Color-ordered placement complete.")

        # Set game status to 3 so camera can start
        self.publish_game_status(3)

    # -------------------- shortcuts --------------------
    def task_home(self):
        self.move_to_joints(self.j_home)

    def task_retract(self):
        self.move_to_joints(self.j_retract)

    def task_add_scene(self):
        self.add_scene()


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
        elif task == "place_by_color":
            node.task_place_by_color()
        else:
            node.get_logger().warn(
                f"Unknown task '{task}'. Valid: home, retract, add_scene, place_by_color"
            )
    finally:
        time.sleep(1)
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
