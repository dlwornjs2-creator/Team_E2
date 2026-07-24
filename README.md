# Team_E2
Rokey Boot Camp 
협동 1 - 2 프로젝트 레포지토리

## 로봇 실행
ros2 launch m0609_rg2_bringup bringup.launch.py mode:=real host:=192.168.1.100 model:=m0609

## 빌드
colcon build --packages-select Team_E2 --symlink-install
source install/setup.bash

ros2 run Team_E2 main_node