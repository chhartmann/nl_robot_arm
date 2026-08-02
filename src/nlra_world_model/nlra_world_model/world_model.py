"""NL Robot Arm — world model.

Tracks ground-truth object poses from Gazebo's scene broadcaster topic
/world/scene/pose/info (bridged via ros_gz_bridge as tf2_msgs/TFMessage;
each transform's child_frame_id is the entity name) and
exposes them to the orchestrator through two services:

  /world_model/get_objects      (nlra_interfaces/srv/GetObjects)
  /world_model/get_object_pose  (nlra_interfaces/srv/GetObjectPose)

This is the grounding layer: symbols (e.g. "red_cube") -> poses. In a later
phase a perception pipeline can feed the same store instead of ground truth;
the service API stays identical.
"""
import threading

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from tf2_msgs.msg import TFMessage

from nlra_interfaces.msg import WorldObject
from nlra_interfaces.srv import GetObjectPose, GetObjects

# Static catalog of the tabletop scene (id -> kind, size, graspable).
# Poses come live from Gazebo; this is the semantic annotation.
CATALOG = {
    "red_cube": {"kind": "cube", "size": (0.05, 0.05, 0.05), "graspable": True},
    "blue_box": {"kind": "tray", "size": (0.16, 0.16, 0.08), "graspable": False},
    "table": {"kind": "table", "size": (0.8, 0.8, 0.4), "graspable": False},
}
WORLD_FRAME = "world"


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
