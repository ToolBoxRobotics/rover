




















#### 8.2 ROS control config - rover_control/config/controllers.yaml



#### 8.3 Gazebo launch - rover_simulation/launch/gazebo_rover.launch



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




