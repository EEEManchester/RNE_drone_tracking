# RNE robot cooperation
## Development environment
- drone framework PX4
- ROS package
    - mavros
    - tams_apriltags
    - apriltag_ros
    - usb_cam

## Simulation test
1. launch simulation: word+robot+fake qualisys+mavros
```shell
  roslaunch rne_gazebo RNE_simu_px4.launch
```
2. launch drone controller + takeoff setpoint
```shell
  roslaunch rne_control rne_drone_control.launch
```
2. launch apriltag and visual servoing
```shell
  roslaunch rne_vs rne_vs.launch
```
3. launch switch node
```
  roslaunch rne_control rne_mission_control.launch
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


