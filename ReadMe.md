# RNE robot cooperation
## Development environment
- drone framework PX4
- ROS package
    - mavros
    - tams_apriltags
    - apriltag_ros
    - usb_cam
    - mavros_controller
    - jackel

## Setup simulation
1. install jackel simulation in gazebo.
For ros noetic
```shell
  sudo apt-get install ros-noetic-jackal-simulator ros-noetic-jackal-desktop ros-noetic-jackal-navigation
```     
If ros-noetic-jackal-desktop cannot be located, then
```shell
  wget https://packages.clearpathrobotics.com/public.key -O - | sudo apt-key add -
sudo sh -c 'echo "deb https://packages.clearpathrobotics.com/stable/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/clearpath-latest.list'
sudo apt-get update
```

Launch a jackel in simulation
```shell
  roslaunch jackal_gazebo jackal_world.launch
```
without gazebo gui
  1.
  ```shell
    roslaunch jackal_gazebo jackal_world.launch gui:=false
  ```
  2. view in rviz
  ```shell
    roslaunch jackal_viz view_robot.launch
  ```  
2. install and configure a drone with PX4 firmware


## Simulation test
1. launch simulation: 
  - empty word in gazebo
  - jackal robot + apriltag
  - drone with framework being px4
  - fake vicon
  - onboard node + mavros
  
```shell
  roslaunch rne_gazebo RNE_simu_setup.launch
```
2. launch drone controller + takeoff setpoint
  - mavros_controller
  - drone home position
  - drone takeoff height
```shell
  roslaunch rne_control rne_mission_control.launch
```
2. launch apriltag and visual servoing
```shell
  roslaunch rne_vs rne_vs.launch
```
3. launch switch node
```
  
```

rosrun plotjuggler plotjuggler

## takeoff test
1. swarn drone with camera and arpiltag into simulation
```html
      <include file="$(find rne_gazebo)/launch/RNE_simu.launch"></include>
```

2. run drone controller to receive commands from ```/reference/setpoint``` and send control input bodyrate to ```/mavros/setpoint_raw/attitude```
```html
  <!-- drone controller -->  
  <arg name="mav_name" default="iris"/>
  <arg name="command_input" default="2" />
  <arg name="gazebo_simulation" default="true" />

  <node pkg="geometric_controller" type="geometric_controller_node" name="geometric_controller" output="screen">
        <param name="mav_name" type="string" value="$(arg mav_name)" />
            <remap from="command/bodyrate_command" to="/mavros/setpoint_raw/attitude"/>
            <param name="ctrl_mode" value="$(arg command_input)" />
            <param name="enable_sim" value="$(arg gazebo_simulation)" />
            <param name="max_acc" value="10.0" />
            <!--Params specific for Iris copter-->
            <param name="attctrl_constant" value="0.3"/>
            <param name="normalizedthrust_constant" value="0.06"/>
            <param name="normalizedthrust_offset" value="0.1"/>
            <param name="Kp_x" value="10.0" />
            <param name="Kp_y" value="10.0" />
            <param name="Kp_z" value="20.0" />
            <param name="Kv_x" value="5.0"/>
            <param name="Kv_y" value="5.0"/>
            <param name="Kv_z" value="10.0"/>            
    </node>
```

3. undertake arpiltag detection
```html
  <include file="$(find apriltag_ros)/launch/rne_continuous_detection.launch">
  </include> 
```

4. run takeoff node to send one single setpoint to ```/reference/setpoint```
```html
      <node pkg="geometric_controller" type="geometric_controller_takeoff_node" name="geometric_takeoff" output="screen">
  </node>
```


