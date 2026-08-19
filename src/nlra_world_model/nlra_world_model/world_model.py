"""NL Robot Arm — world model.

Tracks ground-truth object poses from Gazebo's scene broadcaster topic
/world/scene/pose/info (bridged via ros_gz_bridge as tf2_msgs/TFMessage;
each transform's child_frame_id is the entity name) and
exposes them to the orchestrator through two services:

  /world_model/get_objects      (nlra_interfaces/srv/GetObjects)
  /world_model/get_object_pose  (nlra_interfaces/srv/GetObjectPose)
  /world_model/get_grasp_pose   (nlra_interfaces/srv/GetGraspPose)

This is the grounding layer: symbols (e.g. "red_cube") -> poses. In a later
phase a perception pipeline can feed the same store instead of ground truth;
the service API stays identical.

get_grasp_pose answers "where/how would the robot grasp this object?" and
returns a full 6-DOF pose computed from the object's complete orientation:
the gripper approaches the object's most upward-facing face (or, for a
cylinder, from above across its axis), with the finger closing axis aligned
to the object's geometry — not merely its horizontal yaw.
"""
import math
import threading

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from scipy.spatial.transform import Rotation

from geometry_msgs.msg import PoseStamped
from tf2_msgs.msg import TFMessage

from nlra_interfaces.msg import WorldObject
from nlra_interfaces.srv import GetGraspPose, GetObjectPose, GetObjects

# Static catalog of the tabletop scene (id -> kind, size, graspable).
# Poses come live from Gazebo; this is the semantic annotation.
CATALOG = {
    "red_cube": {"kind": "cube", "size": (0.05, 0.05, 0.05), "graspable": True},
    "green_cube": {"kind": "cube", "size": (0.04, 0.04, 0.04), "graspable": True},
    "yellow_cylinder": {"kind": "cylinder", "size": (0.04, 0.04, 0.06),
                        "graspable": True},
    "orange_block": {"kind": "block", "size": (0.03, 0.03, 0.06),
                     "graspable": True},
    "blue_box": {"kind": "tray", "size": (0.16, 0.16, 0.08), "graspable": False},
    "table": {"kind": "table", "size": (0.8, 0.8, 0.4), "graspable": False},
}
WORLD_FRAME = "world"
# Grasp point = midpoint of the two fingertip link origins (orchestrator FK).
# The gripper's VISUAL fingertips extend 57 mm below those origins.  The
# fingertip collision boxes match the full mesh bounds, including its 6 mm
# extension above the tip-link origin,
# and the tips sweep ~13.5 mm further down while closing. Aiming the grasp
# point at the object's center would therefore drive the fingertips into the
# table. Raise it above the object's bottom (its support plane for tabletop
# objects): 57 (fingertip length) + 13.5 (closing sweep) + ~10 mm of table
# clearance. This is gripper/table geometry only — it does not depend on the
# object's size, so the same raise applies to any future object.
GRASP_RAISE = 0.0805


class WorldModel(Node):
    def __init__(self):
        super().__init__("nlra_world_model")
        self._lock = threading.Lock()
        self._poses = {}  # id -> PoseStamped

        self.create_subscription(
            TFMessage, "/world/scene/pose/info", self._on_pose_info, 10)
        # Per-model PosePublisher topics (these carry proper entity names).
        # Static models publish on .../pose_static instead of .../pose.
        for obj_id in CATALOG:
            for suffix in ("pose", "pose_static"):
                self.create_subscription(
                    TFMessage, f"/model/{obj_id}/{suffix}", self._on_pose_info, 10)

        self.create_service(GetObjects, "world_model/get_objects", self._srv_get_objects)
        self.create_service(GetObjectPose, "world_model/get_object_pose", self._srv_get_pose)
        self.create_service(GetGraspPose, "world_model/get_grasp_pose",
                            self._srv_get_grasp_pose)
        self.get_logger().info(
            f"world model up; tracking: {', '.join(CATALOG)}")

    def _on_pose_info(self, msg: TFMessage):
        # Scene broadcaster publishes every entity's pose; child_frame_id is
        # the entity name. Keep only cataloged objects.
        for tr in msg.transforms:
            obj_id = tr.child_frame_id
            if obj_id not in CATALOG:
                continue
            ps = PoseStamped()
            ps.header.stamp = tr.header.stamp
            ps.header.frame_id = WORLD_FRAME
            ps.pose.position.x = tr.transform.translation.x
            ps.pose.position.y = tr.transform.translation.y
            ps.pose.position.z = tr.transform.translation.z
            ps.pose.orientation = tr.transform.rotation
            with self._lock:
                self._poses[obj_id] = ps

    def _make_object(self, obj_id) -> WorldObject:
        meta = CATALOG[obj_id]
        obj = WorldObject()
        obj.id = obj_id
        obj.kind = meta["kind"]
        obj.size = list(meta["size"])
        obj.graspable = meta["graspable"]
        with self._lock:
            if obj_id in self._poses:
                obj.pose = self._poses[obj_id]
        return obj

    def _srv_get_objects(self, req, res):
        for obj_id in CATALOG:
            if req.kind_filter and CATALOG[obj_id]["kind"] != req.kind_filter:
                continue
            with self._lock:
                have_pose = obj_id in self._poses
            if have_pose:
                res.objects.append(self._make_object(obj_id))
        return res

    def _srv_get_pose(self, req, res):
        with self._lock:
            found = req.id in CATALOG and req.id in self._poses
        res.found = found
        if found:
            res.object = self._make_object(req.id)
        return res

    def _compute_grasp(self, kind, size, p, R):
        """Compute a 6-DOF grasp from the object's full orientation.

        R is the world<-object rotation matrix (columns = local axes in
        world). Returns (grasp_pos, close, approach, note) where grasp_pos is
        the fingertip-midpoint (numpy 3-vector, world), close is the gripper
        +x (finger closing axis), approach is gripper +z (points toward the
        object), and note is a human-readable description.

        Box/cube/block: grasp perpendicular to the face whose outward normal
        points most upward; close along the remaining local axis with the
        largest horizontal component (so elongated objects are grasped across
        their short dimension, fingers on the long faces). Cylinder: approach
        from above; close across a diameter when standing, along the axis's
        horizontal projection when lying down.
        """
        z_world = np.array([0.0, 0.0, 1.0])
        if kind == "cylinder":
            axis = R[:, 2]                 # cylinder length direction (world)
            if abs(axis[2]) > 0.5:
                # standing: close across a horizontal diameter
                close = R[:, 0].copy()
                close[2] = 0.0
                note = "side of standing cylinder, closing across the diameter"
            else:
                # lying: close along the axis's horizontal projection
                close = axis.copy()
                close[2] = 0.0
                note = "top of lying cylinder, closing along its length"
            nc = math.hypot(close[0], close[1])
            close = close / nc if nc > 1e-6 else np.array([1.0, 0.0, 0.0])
            approach = -z_world
            # Vertical half-extent (generalized OBB bottom): the object rests
            # GRASP_RAISE below the grasp point along the approach.
            half = np.array(size) / 2.0
            vhe = float(np.sum(np.abs(R[2, :]) * half))
            grasp_pos = p.copy()
            grasp_pos[2] = p[2] - vhe + GRASP_RAISE
            return grasp_pos, close, approach, note

        # box / block / cube: pick the face most aligned with world up
        dots = np.abs(R[2, :])             # |z component| of each local axis
        i = int(np.argmax(dots))
        n = R[:, i].copy()
        if n[2] < 0:
            n = -n                         # outward normal of the top face
        approach = -n                      # gripper +z onto the top face
        others = [j for j in range(3) if j != i]
        j = max(others, key=lambda k: math.hypot(R[0, k], R[1, k]))
        close = R[:, j].copy()
        half = size[i] / 2.0
        grasp_pos = p + n * (GRASP_RAISE - half)
        note = (f"face along local axis {i} up, closing along local axis {j}")
        return grasp_pos, close, approach, note

    def _srv_get_grasp_pose(self, req, res):
        """Full 6-DOF grasp pose from the object's complete orientation.

        Orientation: gripper +z (approach) onto the object's most-up face
        (or from above for a cylinder); gripper +x (closing axis) aligned to
        the object's geometry. Position: GRASP_RAISE above the object's
        bottom so the closed fingertips clear the support (see GRASP_RAISE).
        """
        with self._lock:
            found = req.object_id in CATALOG and req.object_id in self._poses
        res.found = found
        if not found:
            res.message = (f"object '{req.object_id}' unknown to world model "
                           "or no pose yet")
            return res

        meta = CATALOG[req.object_id]
        if not meta["graspable"]:
            res.message = (f"object '{req.object_id}' (kind '{meta['kind']}') "
                           "is not graspable")
            return res

        ps = self._make_object(req.object_id).pose
        p = np.array([ps.pose.position.x, ps.pose.position.y,
                      ps.pose.position.z])
        q = ps.pose.orientation
        R = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()

        grasp_pos, close, approach, note = self._compute_grasp(
            meta["kind"], meta["size"], p, R)
        up = np.cross(approach, close)     # gripper +y, right-handed frame
        rot = Rotation.from_matrix(np.column_stack([close, up, approach]))
        q = rot.as_quat()                  # x y z w

        res.grasp_pose.header.frame_id = WORLD_FRAME
        res.grasp_pose.header.stamp = ps.header.stamp
        res.grasp_pose.pose.position.x = float(grasp_pos[0])
        res.grasp_pose.pose.position.y = float(grasp_pos[1])
        res.grasp_pose.pose.position.z = float(grasp_pos[2])
        res.grasp_pose.pose.orientation.x = q[0]
        res.grasp_pose.pose.orientation.y = q[1]
        res.grasp_pose.pose.orientation.z = q[2]
        res.grasp_pose.pose.orientation.w = q[3]
        res.message = (f"grasp pose for '{req.object_id}': {note}, "
                       "approach from above")
        return res


def main(args=None):
    rclpy.init(args=args)
    node = WorldModel()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
