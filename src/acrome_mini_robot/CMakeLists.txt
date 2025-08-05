cmake_minimum_required(VERSION 3.5)
project(acrome_mini_robot)

find_package(ament_cmake REQUIRED)

# Tüm gerekli dosyaları install klasörüne kopyala
install(
  DIRECTORY launch urdf meshes config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()