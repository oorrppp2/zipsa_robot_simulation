# 2020/05/26
## 1. 테이블이 추가된 world파일
## 2. arm6_joint의 가용 각도 범위를 10(Degree)로 제한하여 singularity 문제를 해결.
## 3. gazebo world를 gazebo에서 라이다로 스캔하여 시뮬레이션용 맵을 새로 만들어 living_lab_robot_navigation/maps/living_lab_gazebo/ 폴더에 저장.
## 4. living_lab_robot_moveit_client/src/moveit_client_moveit_client_node_obstacle_avoidance.py 파일에 obastacle을 추가하고, 제거하는 기능을 구현함.
 $ rosrun living_lab_robot_moveit_client moveit_client_node_obstacle_avoidance.py
  rostopic pub -1 /add_obstacle std_msgs/String "data: '0'"
   ㄴ> map 기준으로 (2.8, -2.0, 0.45) 위치에 (1.05, 1.75, 0.95) 사이즈의 박스 추가. (5cm씩 margin)
  rostopic pub -1 /del_obstacle std_msgs/String "data: '0'"
   ㄴ> remove box.
