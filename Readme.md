# Acrome Mobil Robotun Gazeboya Entegresi #
```
cd
git clone https://github.com/Rumy7/acrome_mini_robot.git
```
#### Build edilmiş mobil robot dosyalarını klonluyoruz ####
```
cd
mv acrome_mini_robot acrome_ws
cd acrome_ws
sudo apt install python3-colcon-common-extensions
```
#### Alttaki 2 komut ile cmake in ön belleğini temizliyorsun yoksa ilk build olduğu konumu arar ve başka bir bilgisayarda olduğu için bulamaz ####
```
cd ~/acrome_ws
rm -rf build/ install/ log/
```
#### Tekrardan build ediyoruz ####
```
colcon build
echo 'export GZ_SIM_RESOURCE_PATH=$HOME/acrome_ws/src' >> ~/.bashrc
```
#### alttaki 2 satırı uygulamayı her başlattığımızda acrome_ws dosyasının içerisinde çalıştıracağız ####
```
source ~/.bashrc
source install/setup.bash
```
#### launch dosyasının urdf dosyasına erişebilmesi için ####
```
cd ~/acrome_ws/src/acrome_mini_robot/urdf
chmod +x acrome_mini_robot.urdf
```
#### acrome mobile robotunun spawn olduğu simülasyonu açmak için ####
``` 
ros2 launch acrome_mini_robot gazebo_launch.py
```
