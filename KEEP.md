











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









#### 8.2 ROS control config - rover_control/config/controllers.yaml

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




