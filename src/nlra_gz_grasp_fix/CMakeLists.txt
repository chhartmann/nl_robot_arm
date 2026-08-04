cmake_minimum_required(VERSION 3.10)
project(nlra_gz_grasp_fix)

find_package(ament_cmake REQUIRED)
find_package(gz-sim8 REQUIRED)

add_library(nlra_gz_grasp_fix SHARED src/grasp_fix_transport.cc)
target_compile_features(nlra_gz_grasp_fix PUBLIC cxx_std_17)
target_link_libraries(nlra_gz_grasp_fix PRIVATE gz-sim8::gz-sim8)

install(TARGETS nlra_gz_grasp_fix
  LIBRARY DESTINATION lib
)

ament_package()
