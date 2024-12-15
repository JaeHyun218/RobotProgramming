터틀봇 모델 생성

TURTLEBOT3_MODEL=waffle_pi

제작된 gazebo 맵 열기

ros2 launch turtlebot3_gazebo turtlebot3_ex_house.launch.py

navigation2 실행

ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=$HOME/map.yaml

slam 실행

ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True

맵 저장

ros2 run nav2_map_server map_saver_cli -f ~/map

자동 매핑

ros2 run turtlebot_run mapping

yolo 실행

ros2 run turtlebot_run yolo

지정 좌표로 이동

ros2 run turtlebot_run navigation
