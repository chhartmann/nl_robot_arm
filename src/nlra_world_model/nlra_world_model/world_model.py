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

get_grasp_pose answers "where/how would the robot grasp this object?" and by
default returns a pose with the gripper parallel to the object: top-down
approach with the finger closing axis aligned with the object's horizontal
axes (its yaw).
"""
import math
import threading

import numpy as np
import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation

from geometry_msgs.msg import PoseStamped
from tf2_msgs.msg import TFMessage

from nlra_interfaces.msg import WorldObject
from nlra_interfaces.srv import GetGraspPose, GetObjectPose, GetObjects

# Static catalog of the tabletop scene (id -> kind, size, graspable).
# Poses come live from Gazebo; this is the semantic annotation.
CATALOG = {
    "red_cube": {"kind": "cube", "size": (0.05, 0.05, 0.05), "graspable": True},
    "blue_box": {"kind": "tray", "size": (0.16, 0.16, 0.08), "graspable": False},
    "table": {"kind": "table", "size": (0.8, 0.8, 0.4), "graspable": False},
}
WORLD_FRAME = "world"
# Grasp point = midpoint of the two fingertip link origins (orchestrator FK).
# The fingertip collision boxes extend 33 mm below those origins, and the
# tips sweep ~13.5 mm further down while closing; aiming the grasp point at
# the object's center would therefore drive the fingertips into the table.
# Raise it above the object's bottom (its support plane for tabletop
# objects): 33 + 13.5 + ~9 mm of table clearance. This is gripper/table
# geometry only — it does not depend on the object's size, so the same raise
# applies to any future object.
GRASP_RAISE = 0.0555


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

    def _object_yaw(self, ps: PoseStamped) -> float:
        """Yaw of the object's local x-axis projected onto the ground plane.

        Robust to the object lying flat or flipped: the horizontal component
        of its local x-axis is what the gripper must be parallel to.
        """
        x = Rotation.from_quat([
            ps.pose.orientation.x, ps.pose.orientation.y,
            ps.pose.orientation.z, ps.pose.orientation.w]).as_matrix()[:, 0]
        if math.hypot(x[0], x[1]) < 1e-6:
            return 0.0   # local x is vertical -> any yaw is parallel
        return math.atan2(x[1], x[0])

    def _srv_get_grasp_pose(self, req, res):
        """Grasp pose with the gripper parallel to the object (default).

        Orientation: gripper +z (finger pointing direction) straight down,
        gripper +x (finger closing axis) parallel to the object's horizontal
        axes. Position: GRASP_RAISE above the object's bottom so the closed
        fingertips clear the table (see GRASP_RAISE).
        """
        with self._lock:
            found = req.object_id in CATALOG and req.object_id in self._poses
        res.found = found
        if not found:
            res.message = (f"object '{req.object_id}' unknown to world model "
                           "or no pose yet")
            return res

        ps = self._make_object(req.object_id).pose
        p = ps.pose.position
        size = CATALOG[req.object_id]["size"]
        z_bottom = p.z - size[2] / 2.0   # object rests on the tabletop
        yaw = self._object_yaw(ps)
        close = np.array([math.cos(yaw), math.sin(yaw), 0.0])
        approach = np.array([0.0, 0.0, -1.0])
        up = np.cross(approach, close)   # gripper +y for a right-handed frame
        rot = Rotation.from_matrix(
            np.column_stack([close, up, approach]))
        q = rot.as_quat()                # x y z w

        res.grasp_pose.header.frame_id = WORLD_FRAME
        res.grasp_pose.header.stamp = ps.header.stamp
        res.grasp_pose.pose.position.x = p.x
        res.grasp_pose.pose.position.y = p.y
        res.grasp_pose.pose.position.z = z_bottom + GRASP_RAISE
        res.grasp_pose.pose.orientation.x = q[0]
        res.grasp_pose.pose.orientation.y = q[1]
        res.grasp_pose.pose.orientation.z = q[2]
        res.grasp_pose.pose.orientation.w = q[3]
        res.message = (f"grasp pose for '{req.object_id}': gripper parallel to "
                       f"object (closing axis at {math.degrees(yaw):.1f} deg), "
                       "approach from above")
        return res


def main(args=None):
    rclpy.init(args=args)
    node = WorldModel()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
