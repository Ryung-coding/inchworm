<!DOCTYPE html>
<html lang="ko">
<head>
  <meta charset="UTF-8">
</head>
<body>
  <h1>🐛 Inchworm Robot</h1>
  <p>ROS 2 기반 Inchworm 로봇 제어 시스템입니다. 키보드 입력을 통해 Manipulating 모드와 Locomotion 모드를 전환하며 조작할 수 있습니다. Dynamixel 모터 제어와 시리얼 기반 Arduino 그리퍼 동작이 통합되어 있으며, C++ 기반 제어 노드와 Python 기반 키보드 노드가 함께 실행됩니다.</p>

  <p>Launch로 실행하는 경우:</p>
  <pre><code>ros2 launch inchworm robot_launch.py</code></pre>
  <p>position_control 노드가 먼저 실행되고, 1초 후에 별도 터미널에서 keyboard_teleop 노드가 자동 실행됩니다.</p>

  <p>직접 실행하는 경우:</p>
  <pre><code>ros2 run crawling_robot_controller position_control
ros2 run keyboard keyboard_teleop</code></pre>

  <h2>키보드 조작</h2>
  <p>Manipulating 모드에서는 End-effector의 위치를 조정하거나 그리퍼를 조작할 수 있습니다. <code>m</code> 키로 Locomotion 모드로 전환하면 경로를 따라 자동 이동하면서 그리퍼가 자동으로 열리고 닫힙니다. 다시 <code>m</code> 키를 누르면 Manipulating 모드로 복귀합니다.</p>
  <ul>
    <li>↑: End-effector Y+ 이동</li>
    <li>↓: End-effector Y- 이동</li>
    <li>→: End-effector X+ 이동</li>
    <li>←: End-effector X- 이동</li>
    <li><code>m</code>: 모드 전환 (Manipulating ⇄ Locomotion)</li>
    <li><code>,</code>: 간격 줄이기</li>
    <li><code>.</code>: 간격 늘리기</li>
    <li><code>i</code>: 그리퍼 잡기 (Grip ON)</li>
    <li><code>o</code>: 그리퍼 놓기 (Grip OFF)</li>
    <li><code>Ctrl + C</code>: 노드 종료</li>
  </ul>

  <h2>주의사항</h2>
  <p><code>crawling_robot_controller/include/crawling_robot_controller/define.h</code> 파일 내 시리얼 포트를 시스템 환경에 맞게 수정해야 합니다:</p>
  <pre><code>#define DEVICENAME "/dev/ttyUSB0"</code></pre>
  <p>사용 환경에 따라 <code>/dev/ttyACM0</code> 등으로 변경할 수 있습니다.</p>
  <p>keyboard 노드는 <code>stdin</code>을 사용하므로 launch에서 직접 실행 시 오류가 발생할 수 있습니다. 이를 방지하기 위해 <code>gnome-terminal</code>을 사용하여 별도 터미널에서 실행되도록 구성되어 있습니다. gnome-terminal이 없는 환경에서는 <code>ros2 run keyboard keyboard_teleop</code>을 수동으로 실행해야 합니다.</p>
</body>
</html>
