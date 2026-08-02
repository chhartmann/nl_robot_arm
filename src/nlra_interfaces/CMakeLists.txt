cmake_minimum_required(VERSION 3.10)
project(nlra_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/WorldObject.msg"
  "srv/GetObjects.srv"
  "srv/GetObjectPose.srv"
  "srv/NLCommand.srv"
  "action/MoveTo.action"
  "action/ExecuteTask.action"
  "action/MoveJoints.action"
  "action/Grasp.action"
  "action/Release.action"
  "action/Home.action"
  DEPENDENCIES geometry_msgs
)

ament_package()
