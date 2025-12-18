



### 1. Prerequisites
#### 1.1 OS Requirements
Ubuntu 20.04 (PC / Master machine)

SBC (Raspberry Pi 4):

Ubuntu 20.04 Server

ROS Noetic installed on Ubuntu 20.04 Server

1.2 Core Dependencies Installed on Both Machines
sudo apt update
sudo apt install -y git python3-rosdep python3-rosinstall \
                   python3-wstool build-essential ros-noetic-desktop-full
Initialize rosdep:

sudo rosdep init
rosdep update

✅ 2. Workspace Setup
On both the PC and Pi:

mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_make
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc


✅ 3. Clone All Packages
On the PC:

cd ~/catkin_ws/src
git clone <your_repo_containing> rover_description
git clone <your_repo_containing> rover_hardware
git clone <your_repo_containing> rover_arm_traj
git clone <your_repo_containing> toolbox_rover_moveit
If you don’t have repos yet, copy the generated folders into ~/catkin_ws/src.

On the Raspberry Pi:
Only clone the hardware-related packages:

cd ~/catkin_ws/src
git clone <your_repo_containing> rover_hardware
git clone <your_repo_containing> rover_arm_traj





















1. System architecture Hardware

PC (ROS master)

    Runs: roscore, RViz, Gazebo, MoveIt, navigation stack, teleop, OpenCV processing, logging.

Raspberry Pi CM4 on rover

    Runs: sensor/actuator interface nodes, robot_state_publisher, base control, nav stack “local” bits if desired.


Arduino Mega #1 (“drive_mega”)
    6× DRI0002 motor controllers via PCA9685 (PWM) + GPIO DIR pins.
    6× quadrature encoders (A/B).
    4× steering servos via PCA9685.
    TCA9548A I²C mux with:
    MPU6050 IMU
    AHT20 temp/humidity
    INA219 voltage/current
    Talks with CM4 via rosserial over USB.

Arduino Mega #2 (“arm_mega”)
    5× NEMA17 + A4988 (STEP/DIR/EN).
    Limit switches for each joint.
    rosserial over USB.
    Kinect RGB-D camera via USB on CM4/PC.
    YB-MVV21-V1 GPS via USB (NMEA serial) on CM4/PC.
    Joystick via USB on PC.





1. Structure workspace

    rover_description/
            urdf/
                rover.urdf.xacro
            rviz/
                rover.rviz


    rover_bringup/
            launch/
                hardware_bringup.launch
                rviz.launch


    rover_control/
            src/
                wheel_pid_node.py
                ackermann_kinematics_node.py
                odom_node.py
            cfg/
                WheelPID.cfg
            config/
                controllers.yaml


    rover_simulation/
            launch/
                gazebo_rover.launch
            worlds/
                empty.world


    rover_teleop/
            src/
                joy_ackermann_teleop.py
            launch/
                joy_teleop.launch




#### 2. Dependencies 

Install essential packages:

        sudo apt update
        
        sudo apt install -y ros-noetic-desktop-full ros-noetic-rosserial-arduino ros-noetic-rosserial-python \
          ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control ros-noetic-joy ros-noetic-teleop-twist-joy \
          ros-noetic-ackermann-msgs ros-noetic-nav-msgs ros-noetic-robot-state-publisher ros-noetic-joint-state-publisher \
          ros-noetic-controller-manager ros-noetic-gazebo-plugins ros-noetic-gazebo-ros-control \
          ros-noetic-tf2-ros ros-noetic-tf2-tools ros-noetic-rviz ros-noetic-moveit

Python dependencies for custom nodes (example):

        pip3 install numpy rospy tf transformations serial




#### 2. Catkin workspace & package layout

    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws/src
    catkin_init_workspace
    Suggested packages:
    
    catkin_ws/src
    ├── rover_bringup          # Launch files, master/slave setup, rosserial
    ├── rover_description      # URDF/Xacro, meshes
    ├── rover_control          # ros_control, Ackermann, PID, odom
    ├── rover_arduino_drive    # messages + config for drive_mega
    ├── rover_arduino_arm      # messages + config for arm_mega
    ├── rover_navigation       # robot_localization, move_base configs
    ├── rover_teleop           # joystick + teleop/ackermann nodes
    ├── rover_vision           # Kinect, OpenCV, nav perception
    ├── rover_gazebo           # Gazebo world, plugins, sim launch
    └── rover_arm_moveit       # MoveIt config for 5-DOF arm



    mkdir -p ~/rover_ws/src
    cd ~/rover_ws/src

    catkin_create_pkg rover_bringup rospy std_msgs sensor_msgs geometry_msgs nav_msgs tf tf2_ros ackermann_msgs joy
    catkin_create_pkg rover_description urdf xacro
    catkin_create_pkg rover_control rospy std_msgs sensor_msgs nav_msgs tf tf2_ros dynamic_reconfigure
    catkin_create_pkg rover_simulation gazebo_ros
    catkin_create_pkg rover_teleop rospy ackermann_msgs sensor_msgs joy

    catkin_make

    source ~/catkin_ws/devel/setup.bash



    TF tree from URDF:

        odom (from odom_node)

        base_link

        fl_link → fl_wheel

        ml_link → ml_wheel

        rl_link → rl_wheel

        fr_link → fr_wheel

        mr_link → mr_wheel

        rr_link → rr_wheel



#### 4. URDF (rover.urdf.xacro)

    — summary
    The URDF uses xacro macros. Key links:
    
    base_link -> chassis_link
    
    imu_link mounted to chassis
    
    wheel_* joints (6 wheels): wheel_fl, wheel_ml, wheel_rl, wheel_fr, wheel_mr, wheel_rr
    
    steer_fl_joint, steer_fr_joint, steer_rl_joint, steer_rr_joint (4 steering joints)
    
    caster middle wheels fixed rotation joints
    
    camera_link (kinect) attached near front
    
    arm_base_link, arm_joint_1..arm_joint_5
    
    Include transmission tags for ros_control (effort or velocity controllers) for wheels and position controllers for steering servos and arm joints.
    
    (Full xacro is included in the canvas document.)
    
    map
    
    odom
    
    base_link
    
    imu_link
    
    gps_link
    
    camera_link → camera_rgb_optical_frame, camera_depth_optical_frame
    
    Wheel links: front_left_wheel_link, mid_left_wheel_link, rear_left_wheel_link, etc.
    
    Steering joints: front_left_steer_link, etc.
    
    Arm base: arm_base_link → arm_shoulder_link → arm_elbow_link → arm_wrist_pitch_link → arm_wrist_roll_link → arm_gripper_link





#### 5. Arduino sketch (arduino_sketch.ino)
    
    PCA9685 at 0x40.

    TCA9548A at 0x70, with:
        Channel 0: MPU6050
        Channel 1: AHT20
        Channel 2: INA219
    
    
    Features implemented in sketch:
    
    I2C to PCA9685 to control motor PWM & steering servos
    
    Encoder reading via interrupts (attachInterrupt)
    
    MPU6050 read and DMP or raw accel/gyro fused into orientation (complementary filter)
    
    GPS read from serial port and NMEA parsing (GGA/VTG)
    
    Stepper control for arm (step/dir pins) and reading limit switches
    
    rosserial publisher topics: /imu/data_raw, /gps/fix, /encoder_counts, /odometry_raw, /arm/joint_states
    
    rosserial subscribers: /cmd_wheels (custom message: array of wheel velocities), /cmd_steer (angles), /arm/cmd (joint moves)
    
    The sketch includes safe-guards, watchdog, and a parameter for PCA9685 I2C address.
    
    (Full Arduino sketch included in canvas document.)




#### 6. Key ROS nodes (scripts)

    ackermann_to_wheels.py
    Subscribes to /ackermann_cmd (type ackermann_msgs/AckermannDriveStamped) or /cmd_vel.
    
    Converts steering angle to servo PWM (via PCA9685) and longitudinal velocity to wheel angular velocities.
    
    Publishes wheel commands to /cmd_wheel_velocities (custom Float64MultiArray) or forwards to rosserial topics consumed by Arduino.
    
    odometry_node.py
    Reads encoder counts (from /encoder_counts) and IMU yaw to compute odometry.
    
    Publishes nav_msgs/Odometry on /odom and broadcasts TF odom -> base_link.
    
    Optionally runs a velocity PID per wheel (or central velocity PID) and publishes control effort to /wheel_effort_cmds.
    
    pid_velocity_controller.py
    A controller node implementing simple PID for wheel velocities; tuned using params/controllers.yaml.
    
    arm_controller.py
    High-level node to convert joint-angle goals to stepper step counts and send step/dir sequences via rosserial topics.
    
    Performs homing using limit switches on startup.
    
    (Full python node code included in canvas.)

  TF tree from URDF:

        odom (from odom_node)

        base_link

        fl_link → fl_wheel

        ml_link → ml_wheel

        rl_link → rl_wheel

        fr_link → fr_wheel

        mr_link → mr_wheel

        rr_link → rr_wheel


#### 8.2 ROS control config - rover_control/config/controllers.yaml

    wheel velocity PIDs
    steering position controllers
    arm stepper microstepping & speed limits


        joint_state_controller:
        type: joint_state_controller/JointStateController
        publish_rate: 50

        wheel_controllers:
        fl_wheel_joint:
            type: velocity_controllers/JointVelocityController
            joint: fl_wheel_joint
            pid: {p: 10.0, i: 0.0, d: 0.0}
        ml_wheel_joint:
            type: velocity_controllers/JointVelocityController
            joint: ml_wheel_joint
            pid: {p: 10.0, i: 0.0, d: 0.0}
        rl_wheel_joint:
            type: velocity_controllers/JointVelocityController
            joint: rl_wheel_joint
            pid: {p: 10.0, i: 0.0, d: 0.0}
        fr_wheel_joint:
            type: velocity_controllers/JointVelocityController
            joint: fr_wheel_joint
            pid: {p: 10.0, i: 0.0, d: 0.0}
        mr_wheel_joint:
            type: velocity_controllers/JointVelocityController
            joint: mr_wheel_joint
            pid: {p: 10.0, i: 0.0, d: 0.0}
        rr_wheel_joint:
            type: velocity_controllers/JointVelocityController
            joint: rr_wheel_joint
            pid: {p: 10.0, i: 0.0, d: 0.0}

        steering_controllers:
        fl_steer_joint:
            type: position_controllers/JointPositionController
            joint: fl_steer_joint
            pid: {p: 5.0, i: 0.0, d: 0.0}
        fr_steer_joint:
            type: position_controllers/JointPositionController
            joint: fr_steer_joint
            pid: {p: 5.0, i: 0.0, d: 0.0}
        rl_steer_joint:
            type: position_controllers/JointPositionController
            joint: rl_steer_joint
            pid: {p: 5.0, i: 0.0, d: 0.0}
        rr_steer_joint:
            type: position_controllers/JointPositionController
            joint: rr_steer_joint
            pid: {p: 5.0, i: 0.0, d: 0.0}


#### 8.3 Gazebo launch - rover_simulation/launch/gazebo_rover.launch

                <launch>
                  <arg name="world" default="$(find rover_simulation)/worlds/empty.world"/>
                
                  <include file="$(find gazebo_ros)/launch/empty_world.launch">
                    <arg name="world_name" value="$(arg world)"/>
                    <arg name="paused" value="false"/>
                    <arg name="use_sim_time" value="true"/>
                  </include>
                
                  <param name="robot_description" command="$(find xacro)/xacro '$(find rover_description)/urdf/rover.urdf.xacro'"/>
                
                  <node pkg="gazebo_ros" type="spawn_model" name="spawn_rover" args="-urdf -model rover -param robot_description" />
                
                  <rosparam file="$(find rover_control)/config/controllers.yaml" command="load" />
                
                  <node name="controller_spawner" pkg="controller_manager" type="spawner"
                        args="joint_state_controller
                              fl_wheel_joint
                              ml_wheel_joint
                              rl_wheel_joint
                              fr_wheel_joint
                              mr_wheel_joint
                              rr_wheel_joint
                              fl_steer_joint
                              fr_steer_joint
                              rl_steer_joint
                              rr_steer_joint" />
                </launch>
                (You might instead spawn each controller name as specified in YAML; adjust spawner args accordingly.)





#### 9.0 Hardware bring-up launch - rover_bringup/launch/hardware_bringup.launch
        
        <launch>
          <!-- Robot description -->
          <param name="robot_description" command="$(find xacro)/xacro '$(find rover_description)/urdf/rover.urdf.xacro'" />
        
          <node pkg="robot_state_publisher" type="robot_state_publisher" name="robot_state_publisher" />
        
          <!-- rosserial Arduino -->
          <node pkg="rosserial_python" type="serial_node.py" name="rosserial_node" output="screen">
            <param name="port" value="/dev/ttyACM0" />
            <param name="baud" value="57600" />
          </node>
        
          <!-- Wheel PID -->
          <node pkg="rover_control" type="wheel_pid_node.py" name="wheel_pid_node" output="screen">
            <param name="ticks_per_rev" value="1024" />
            <param name="wheel_radius" value="0.1" />
          </node>
        
          <!-- Ackermann kinematics -->
          <node pkg="rover_control" type="ackermann_kinematics_node.py" name="ackermann_kinematics" output="screen">
            <param name="wheel_radius" value="0.1" />
            <param name="wheel_base" value="0.5" />
            <param name="track_width" value="0.4" />
          </node>
        
          <!-- Odometry -->
          <node pkg="rover_control" type="odom_node.py" name="odom_node" output="screen">
            <param name="ticks_per_rev" value="1024" />
            <param name="wheel_radius" value="0.1" />
            <param name="track_width" value="0.4" />
          </node>
        </launch>



#### 10. How it all fits together

      Topics & flow:
      
      Teleop:
      joy_node → /joy → joy_ackermann_teleop → /ackermann_cmd
      
      Kinematics:
      /ackermann_cmd → ackermann_kinematics →
      /wheel_speed_cmd (6) + /steering_cmd (4)
      
      Control:
      /wheel_speed_cmd → wheel_pid_node + /encoder_ticks → /wheel_pwm_cmd
      /wheel_pwm_cmd + /steering_cmd → Arduino (motor drivers + servos)
      
      Feedback & odom:
      Arduino → /encoder_ticks → odom_node → /odom + TF odom -> base_link
      
      Visualization:
      URDF + joint states (from hardware or Gazebo) → robot_state_publisher → TF tree
      RViz displays odom frame, TF tree, RobotModel.




8 — Launch files

simulation.launch — starts Gazebo with rover model, spawns robot state publisher, joint_state_publisher, controllers, openni2_launch (Kinect), and RViz.
bringup.launch — connects to physical Arduino via rosserial_python and starts IMU, GPS, odom nodes and controller managers.
teleop.launch — starts joy_node and maps to ackermann teleop node.
ackermann_bridge.launch — bridge from /ackermann_cmd to wheel+steer topics.



9 — Gazebo simulation notes

Use gazebo_ros_control plugin and ros_control transmissions to let controller_manager handle wheel velocity controllers and position controllers for steering and arm joints.
Use libgazebo_ros_openni_kinect.so plugin (or OpenNI2 equivalent) to simulate Kinect depth and color topics /camera/rgb/image_raw, /camera/depth_registered/image_raw, and /camera/depth/points.
Provide a plugin to simulate encoder ticks (joint state publisher with high-res values) or use the joint_state_controller.


10 — RViz & TF

RViz config shows:
    map -> odom -> base_link -> chassis_link -> camera_link -> arm_base_link -> arm_joint_*
    Wheel links and steering links displayed as joint states
    IMU and GPS markers (visualization_msgs/Marker)
    PointCloud2 from Kinect


11 — Wiring & pin mapping (summary)

Arduino (Uno/Mega suggestion: Mega recommended for many interrupts and serials)
PCA9685 SDA/SCL -> Arduino SDA/SCL (I2C)
PCA9685 V+ -> 12V (for servos), Vcc -> 5V (logic) — isolate power grounds
DRI0002 motor driver PWM inputs connected to PCA9685 channels (external motor power supply to DRI0002)
Encoder A/B -> Arduino digital interrupt pins (use hardware interrupts; Megas have many)
MPU6050 -> I2C
GPS TX -> Arduino RX1 (use Serial1 on Mega)
Stepper A4988 STEP & DIR -> Arduino digital pins (one pair per motor or use stepper driver board multiplex)
Limit switches -> digital input with pullups
Powering: use separate battery for motors (12V) and logic (5V regulated). Ensure common ground.



12 — Testing & calibration checklist

Flash Arduino sketch and verify rosserial connection: rosrun rosserial_python serial_node.py /dev/ttyACM0.
Verify IMU publishes to /imu/data_raw.
Verify encoders publish counts when wheels turned manually.
Test PCA9685: move a single servo and single motor PWM at low values.
Run ackermann_to_wheels.py in simulation while in Gazebo and verify steering conversion.
Tune wheel velocity PID: start low P, then increase I/D carefully.
Calibrate odometry wheel radius and encoder ticks per revolution in params/odometry.yaml.
Arm homing: test limit switches and stepper movements slowly.



13 — Next steps & optional enhancements

Add RTK-capable GPS / RTK library for centimeter-level positioning.
Integrate MoveIt for arm path planning and grasping.
Add sensor fusion with robot_localization for IMU+GPS+odometry.
Add autonomous behaviors using move_base, teb_local_planner or nav2 porting.





10. Bringup launches
Rover side (CM4)
rover_bringup/launch/rover_cm4.launch:

<launch>
  <!-- Set ROS_MASTER_URI in env when starting -->

  <include file="$(find rover_bringup)/launch/description.launch"/>

  <!-- rosserial for drive & arm -->
  <node pkg="rosserial_python" type="serial_node.py" name="drive_serial">
    <param name="port" value="/dev/ttyACM0"/>
    <param name="baud" value="115200"/>
  </node>

  <node pkg="rosserial_python" type="serial_node.py" name="arm_serial">
    <param name="port" value="/dev/ttyACM1"/>
    <param name="baud" value="115200"/>
  </node>

  <!-- Kinect, GPS, odom, ackermann node, etc. -->
  <include file="$(find rover_vision)/launch/kinect.launch"/>
  <include file="$(find rover_navigation)/launch/ekf.launch"/>
  <include file="$(find rover_navigation)/launch/move_base.launch"/>

  <node pkg="rover_control" type="ackermann_drive_node.py" name="ackermann_drive" output="screen"/>
  <!-- Odometry node here if running on CM4 -->
</launch>
PC/master side
rover_bringup/launch/master_pc.launch:

<launch>
  <!-- Assume roscore running here -->

  <include file="$(find rover_bringup)/launch/description.launch"/>
  <include file="$(find rover_teleop)/launch/joystick_teleop.launch"/>
  <include file="$(find rover_bringup)/launch/rviz.launch"/>

  <!-- Gazebo sim (optional) -->
  <!-- <include file="$(find rover_gazebo)/launch/rover_world.launch"/> -->

  <!-- MoveIt for arm -->
  <include file="$(find rover_arm_moveit)/launch/demo.launch"/>
</launch>



            1. Hardware Overview:
            Rover Structure: Six-wheeled rover with 12V, 83 RPM motors for each wheel and encoders.
            
            Steering: Four corner-mounted 40kg servos for Ackermann steering.
            
            Motor Controllers: DRI0002 motor drivers for controlling the wheels.
            
            Sensors:
            
            IMU: MPU6050 for orientation and acceleration data.
            
            Voltage & Current Sensor: INA219 to monitor power.
            
            GPS: YB-MVV21-V1 GPS module for localization.
            
            Camera: Kinect RGBD for computer vision and autonomous navigation.
            
            Robotic Arm: 5-axis with NEMA17 stepper motors and A4988 controllers, controlled by a second Arduino Mega.
            
            2. System Components & Wiring:
            Arduino Mega: Two boards will handle the motor and sensor data.
            
            One for controlling the motors, servos, sensors (INA219, MPU6050, encoders), and the motor drivers.
            
            Another for the robotic arm and additional sensors.
            
            Motor Controllers: The DRI0002 motor controllers will interface with the Arduino via PWM or digital control pins for speed and direction control.
            
            PCA9685: The servo control will be handled by the PCA9685, controlled via I2C to set the steering angles.
            
            TCA9548A Multiplexer: Connect multiple I2C devices (e.g., INA219, MPU6050) using the multiplexer to avoid address conflicts.
            
            Sensors: INA219 for voltage/current monitoring, MPU6050 for IMU data, and GPS for localization.
            
            3. Software Architecture:
            The software environment will be structured with ROS Noetic as the backbone, integrating control systems, sensors, and simulation.
            
            ROS Packages and Nodes:
            Motor Control: A node will handle the control of the motors via the DRI0002 motor controllers, utilizing the PWM signals or serial communication with the Arduino Mega.
            
            Use rosserial to communicate with the Arduino Mega to control motors and read encoders.
            
            Implement PID controllers in ROS to control the speed and position of each wheel.
            
            Servo Control: Use the pca9685 ROS package to control the 40kg servos for Ackermann steering. The steering angle will be determined based on the rover's velocity and the control commands sent from the master PC or joystick.
            
            Encoder Feedback: Read encoder signals from each wheel using ROS topics. This can be done by configuring an interrupt-driven Arduino node or using a ROS package that interfaces with encoders.
            
            Encoder data will be used for odometry calculations.
            
            IMU Integration: Use the mpu6050 ROS package to interface with the MPU6050 for obtaining orientation, acceleration, and gyroscope data.
            
            This can also be fused with odometry data to improve the rover's localization.
            
            GPS Integration: Integrate the GPS module via a serial connection, using the nmea_navsat_driver package to handle GPS data.
            
            Robotic Arm Control: Implement control for the 5-axis robotic arm using stepper motors with the A4988 drivers.
            
            Use the ros_control framework to handle the arm’s motion and calibration.
            
            Add a separate node for programming and controlling the arm via the Arduino Mega.
            
            Simulation: Integrate the rover in Gazebo for simulation.
            
            Use the gazebo_ros_pkgs to simulate the rover and the robot arm.
            
            Create a model in URDF for the rover and robotic arm, and use gazebo_ros_control to simulate the arm’s motions.
            
            Implement a TF tree to publish the transformations between the rover base and robotic arm.
            
            Teleoperation: Use a joystick for manual control through ROS. The teleop_twist_joy package can be configured for controlling the rover’s movement.
            
            This will send Twist messages (linear and angular velocities) to the rover for forward/backward and turning movements.
            
            Nodes Breakdown:
            Master Node (PC):
            
            Control system that sends commands (velocity, steering angle, arm control).
            
            Visualization in RViz.
            
            Autonomous navigation and SLAM (Simultaneous Localization and Mapping) using sensor data.
            
            Use OpenCV for computer vision tasks with the Kinect camera.
            
            Rover Node (SBC or Arduino Mega):
            
            Motor control (DRI0002 + encoders).
            
            Servo control (PCA9685 for Ackermann steering).
            
            Sensor data collection (INA219, MPU6050).
            
            Odometry calculation and publishing.
            
            GPS data processing.
            
            4. PID Control for Motors:
            Implement PID control in ROS for each wheel’s motor to maintain desired speed and position. Use encoder feedback for closed-loop control. Tuning of PID parameters will be necessary to get stable control over the rover.
            
            5. Odometry and Navigation:
            Use wheel encoder data for odometry to track the rover's position and orientation.
            
            Combine the odometry with IMU data for better localization, using sensor fusion algorithms like robot_localization to integrate data from encoders, IMU, and GPS.
            
            Implement autonomous navigation using the move_base package for path planning and obstacle avoidance, utilizing the Kinect camera for environment perception.
            
            6. Autonomous Navigation:
            Create an autonomous navigation system that includes:
            
            Obstacle avoidance: Use data from the Kinect camera to detect obstacles and plan paths around them.
            
            Localization: Fuse GPS and odometry data for accurate localization in the environment.
            
            Path planning: Implement a local planner using the nav_core and dwa_local_planner to navigate through environments.
            
            7. Programming the Robotic Arm:
            Create a separate set of nodes to control the robotic arm, providing functions to move each joint of the arm.
            
            Use inverse kinematics (IK) for planning the arm’s motions and controlling it in 3D space.
            
            8. Gazebo Simulation:
            Create a detailed model for the rover and its arm in URDF format.
            
            Use gazebo_ros packages to integrate the robot model into Gazebo for testing.
            
            Simulate sensor data (e.g., IMU, encoders, camera, GPS) and test algorithms in the virtual environment.
            
            9. Visualization in RViz:
            Set up RViz for visualizing the robot's state, including:
            
            Robot Model: Visualize the rover and robotic arm.
            
            Odometry: Display the rover's trajectory in RViz.
            
            Sensor Data: Show IMU, GPS, camera feed, and laser scans.
            
            10. Joystick Teleoperation:
            Set up the teleop_twist_joy package for controlling the rover manually via a joystick, sending velocity commands for both driving and steering.
            
            11. Communication Between SBC and PC:
            Set up communication between the master PC and the SBC (Single Board Computer) using ROS topics and services. The master PC can control the rover remotely through ROS messages.
            
            This approach combines hardware integration with ROS-based control and simulation, providing both manual and autonomous operation for the rover.



9) Bringup / Launch files

rover_bringup/launch/rover_on_pi.launch

Runs on CM4:

<launch>
  <param name="/use_sim_time" value="false"/>

  <!-- rosserial to Arduino1 and Arduino2 -->
  <node pkg="rosserial_python" type="serial_node.py" name="serial_drive"
        args="/dev/ttyACM0" output="screen">
    <param name="baud" value="57600"/>
  </node>

  <node pkg="rosserial_python" type="serial_node.py" name="serial_arm"
        args="/dev/ttyACM1" output="screen">
    <param name="baud" value="57600"/>
  </node>

  <node pkg="rover_bridge" type="rover_bridge_node.py" name="rover_bridge" output="screen">
    <param name="wheel_radius" value="0.10"/>
    <param name="track" value="0.55"/>
  </node>
</launch>
rover_bringup/launch/rover_on_pc.launch
Runs on PC:

<launch>
  <node pkg="robot_state_publisher" type="robot_state_publisher" name="rsp"/>

  <node pkg="rover_control" type="rover_control_node" name="rover_control" output="screen">
    <param name="wheel_radius" value="0.10"/>
    <param name="track" value="0.55"/>
    <param name="wheelbase" value="0.80"/>
    <param name="max_steer" value="0.6"/>
  </node>

  <!-- Teleop -->
  <node pkg="teleop_twist_keyboard" type="teleop_twist_keyboard.py"
        name="teleop" output="screen" />

</launch>

10) How to run
On PC

source ~/rover_ws/devel/setup.bash
export ROS_MASTER_URI=http://<PC_IP>:11311
export ROS_IP=<PC_IP>
roslaunch rover_bringup rover_on_pc.launch
On CM4

source ~/rover_ws/devel/setup.bash
export ROS_MASTER_URI=http://<PC_IP>:11311
export ROS_IP=<PI_IP>
roslaunch rover_bringup rover_on_pi.launch
You should now be able to:

teleop from PC

see /wheel_cmd, /steer_cmd

see encoder-based /odom on PC

visualize TF in RViz



2) rover_msgs (custom messages)
rover_msgs/msg/WheelCmd.msg
float32[] wheel_rpm   # length 6
rover_msgs/msg/SteerCmd.msg
float32[] steer_deg   # length 4 (FL, FR, RL, RR)
rover_msgs/msg/ArmCmd.msg
float32[] joint_deg   # length 5
float32   speed_scale # 0..1
Edit rover_msgs/CMakeLists.txt:

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)

add_message_files(
  FILES
  WheelCmd.msg
  SteerCmd.msg
  ArmCmd.msg
)

generate_messages(
  DEPENDENCIES std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)
rover_msgs/package.xml: add message_generation, message_runtime.



rover_base/CMakeLists.txt additions:

find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs rover_msgs)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(rover_controller src/rover_controller.cpp)
add_dependencies(rover_controller ${catkin_EXPORTED_TARGETS})
target_link_libraries(rover_controller ${catkin_LIBRARIES})


5) rover_bringup launch on Pi
rover_bringup/launch/pi_bringup.launch:

<launch>
  <rosparam file="$(find rover_base)/config/base.yaml" command="load"/>

  <!-- rosserial to Drive Mega -->
  <node pkg="rosserial_python" type="serial_node.py" name="drive_mega"
        output="screen">
    <param name="port" value="/dev/ttyACM0"/>
    <param name="baud" value="57600"/>
  </node>

  <!-- rosserial to Arm Mega -->
  <node pkg="rosserial_python" type="serial_node.py" name="arm_mega"
        output="screen">
    <param name="port" value="/dev/ttyACM1"/>
    <param name="baud" value="57600"/>
  </node>

  <node pkg="rover_base" type="rover_controller" name="rover_controller" output="screen"/>
  <node pkg="rover_base" type="odom_node.py" name="rover_odom" output="screen"/>
</launch>

6) PC bringup (teleop, rviz, nav, MoveIt)
rover_bringup/launch/pc_master.launch:

<launch>
  <env name="ROS_MASTER_URI" value="http://<PC_IP>:11311"/>
  <env name="ROS_IP" value="<PC_IP>"/>

  <node pkg="teleop_twist_keyboard" type="teleop_twist_keyboard.py"
        name="teleop" output="screen"/>

  <node pkg="rviz" type="rviz" name="rviz" output="screen" />

  <!-- add SLAM/nav stacks later -->
</launch>

