# Acrome Mini Robot – Simulation Setup & Usage Manual #

## Doküman Hakkında ##

Bu belge, ROS 2 Jazzy Jalisco (Ubuntu 24.04) işletim sistemi üzerinde Gazebo Harmonic simülatörü kullanılarak geliştirilen Acrome Mini Robot simülasyonunun kurulumunu, kullanımını ve SLAM (Eşzamanlı Konum Belirleme ve Harita Oluşturma) uygulamasını adım adım açıklamaktadır.

Belgenin Amacı: Bu çalışmayı ilk defa kuracak kullanıcıların, simülasyon ortamını hatasız bir şekilde çalıştırabilmesini, robotu hareket ettirebilmesini, temel sensör verilerini (LIDAR) gözlemleyebilmesini ve nihayetinde SLAM kullanarak bir ortamın haritasını çıkarabilmesini sağlamaktır.

## Sistem Gereksinimleri ##







## Kurulum ve Yapılandırma ##
### ROS 2 Jazzy Kurulumu ###
Kurulum için aşağıdaki bağlantıya tıklayınız : 
https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html
### Bağımlılıkların Yüklenmesi ###
```
sudo apt install python3-rosdep
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### Ek Paket Kurulumları ###
```
sudo apt install ros-jazzy-gazebo-ros-pkgs
sudo apt install ros-jazzy-slam-toolbox
sudo apt install ros-jazzy-teleop-twist-keyboard
sudo apt install ros-jazzy-robot-state-publisher ros-jazzy-joint-state-publisher
sudo apt install ros-jazzy-velocity-controllers ros-jazzy-joint-state-broadcaster
sudo apt install ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge ros-jazzy-gz-ros2-control
sudo apt install ros-jazzy-tf-transformations
```
### Proje Repository'sinden Klonlama ###
```
git clone https://github.com/Rumy7/acrome_ws.git
cd ~/acrome_ws
```
! Workspacedeki eski log build install dosyaları silinir
! launch xacro ve .py dosyalarındaki dosya yolları kontrol edilmeli ve düzenlenmeli

### Proje Derleme ###
Proje kodlarda her değişimde terminalde derlenmek zorundadır
```
colcon build --symlink-install
source install/setup.bash
```

## Robot Simülasyonu ve Kontrolü ##
Robotu gazebo ve rviz ortamlarında görüntülemek için terminalde : 
`ros2 launch acrome_mini_robot gazebo_launch.py` 
komutu çalıştırılır.
Slam adımına geçmek için aşağıdaki komut bloku çalıştırılır:
```
ros2 lifecycle set /slam_toolbox configure
ros2 lifecycle set /slam_toolbox activate
ros2 lifecycle get /slam_toolbox
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 lidar_link  acrome_mini_robot/base_link/lidar_sensor
```
Robotu kontrol edebilmek için 3. bir terminal açılır ve aşağıdaki komut çalıştırılır :
```
python3 src/acrome_mini_robot/launch/robot_controller.py
```
### Rvizde verilerin görselleştirilmesi ###
Görülmek istenilen tüm parametreler `add` kısmından sol panele eklenir bu şekilde araç hareket ettirilmeye başlandığında `odometry` oku yön değiştirir , `lazer` etrafı tarar ve `map` elde edilmeye başlanır.

## Sensör Verileri ve Monitoring ##
### Sensör Verilerini İzleme ###
LIDAR Veri Akışı :
```
ros2 topic echo /scan
```
Odometry Verileri :
```
ros2 topic echo /odom
```
### Sistem Monitoring ###
ROS Graph Görüntüleme :
```
rqt_graph
```
Aktif Topic'lerin Listelenmesi :
```
ros2 topic list
```
Aktif Node'ların Listelenmesi :
```
ros2 node list
```
## Hata Ayıklama ##
### Log ve Hata Mesajları ###
```
ros2 topic echo /rosout
```
### TF (Transform) Sistem Kontrolü ###
TF Tree Görüntüleme:
```
ros2 run tf2_tools view_frames
```
### Parametre Kontrolü ###
ros2 param list :
```
ros2 param get /node_name parameter_name
```

## Sistem Yönetimi ve Bakım ##
### Sistem Temizleme ###
Gazebo Proseslerini Durdurma:
```
pkill gzserver
pkill gzclient
```
### Çalışma Alanı Yönetimi ###
Derleme Sonrası Ortam Değişkenleri:
```
source ~/acrome_ws/install/setup.bash
```
Projeyi Yeniden Derleme :
```
cd ~/acrome_ws
colcon build --symlink-install
```
## Test Senaryoları ##
### Temel Fonksiyon Testleri ###
Robotun hareket komutları topic listte gözükmesine rağmen hareket etmiyorsa aşağıdaki test komutları gönderilebilir.
Doğrusal Hareket Testi
```
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}" & sleep 5; kill $!
```
Dönüş Hareket Testi
```
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}" & sleep 3; kill $!
```
## Sık Karşılaşılan Sorunlar ve Çözümler ##
### Derleme Hataları ###
Problem: colcon build hataları
Çözüm:
```
rosdep install --from-paths src --ignore-src -r -y
```
Gazebo Başlatma Problemleri
Problem: Gazebo açılmıyor
Çözüm:
```
pkill gzserver
pkill gzclient
ros2 launch acrome_bringup gazebo.launch.py
```

## Son Notlar ##
Her yeni terminal oturumunda ROS 2 ve çalışma alanı ortam değişkenleri yüklenmelidir
Gazebo simülasyonları sistem kaynaklarını yoğun kullanabilir
SLAM haritalama için robotun hareket etmesi gerekmektedir

Bu belge, Acrome robot projesinin ROS 2 Jazzy ve Gazebo entegrasyonu için hazırlanmıştır. Tüm komutlar test edilmiş olup, stabil çalışma ortamı için önerilmektedir.





## Programı çalıştırma ##
Normal Simülasyonu çalıştırmak için:

1. Terminal
```
colcon build
source install/setup.bash
source ~/.bashrc
ros2 launch acrome_mini_robot gazebo_launch.py slam_mode:=mapping
```

2. Terminal
```
ros2 lifecycle set /slam_toolbox configure
ros2 lifecycle set /slam_toolbox activate
ros2 lifecycle get /slam_toolbox
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 lidar_link acrome_mini_robot/base_link/lidar_sensor
```

3. Terminal
```
python3 src/acrome_mini_robot/launch/robot_controller.py
```

## rassbery pi için ##
run publisher from computer
```
python3 /home/halit/acrome_ws/src/acrome_mini_robot/launch/pc_forwarder.py
```
run receiver from rassbery
```
python3 Desktop/odom_receiver.py
ros2 topic echo /odom
ros2 topic echo /scan
```
## Docker build için ##
```
cd ~/acrome_ws
DOCKER_BUILDKIT=0 docker build -t my_image .
```

## Dockera girmek için ##
1. Terminal
```
cd ~/acrome_ws
docker run -it --rm \
  --net=host \
  -v /home/halit/acrome_ws:/home/halit/acrome_ws \
  --env DISPLAY=$DISPLAY \
  --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
  acrome_robot:latest /bin/bash
```
```
sudo apt install ros-jazzy-slam-toolbox
sudo apt install ros-jazzy-tf-transformations
```
```
. /opt/ros/jazzy/setup.sh
colcon build
source install/setup.bash
source ~/.bashrc
```
2. Terminal
```
cd ~/acrome_ws
docker ps
docker exec -it NAMES /bin/bash
```
```
. /opt/ros/jazzy/setup.sh
source install/setup.bash
source ~/.bashrc
```
```
ros2 launch acrome_mini_robot gazebo_launch.py slam_mode:=mapping
```
3. Terminal
```
cd ~/acrome_ws
docker ps
docker exec -it NAMES /bin/bash
```
```
. /opt/ros/jazzy/setup.sh
source install/setup.bash
source ~/.bashrc
```
```
ros2 lifecycle set /slam_toolbox configure
ros2 lifecycle set /slam_toolbox activate
ros2 lifecycle get /slam_toolbox
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 lidar_link acrome_mini_robot/base_link/lidar_sensor
```
4. Terminal 
```
cd ~/acrome_ws
docker ps
docker exec -it NAMES /bin/bash
```
```
. /opt/ros/jazzy/setup.sh
source install/setup.bash
source ~/.bashrc
```
```
python3 src/acrome_mini_robot/launch/robot_controller.py
```
