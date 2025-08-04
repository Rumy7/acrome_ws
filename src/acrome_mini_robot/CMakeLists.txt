cmake_minimum_required(VERSION 3.5)
project(acrome_mini_robot)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch urdf meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
