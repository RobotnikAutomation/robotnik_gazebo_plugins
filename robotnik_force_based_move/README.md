Moves robot using a force-based approach, while also removing slippage by stopping the robot after the command watchdogsout. Based on GazeboRosForceBasedMove from Team HECTOR.

Sample configuration:

```
<gazebo>
   <plugin name="ros_force_based_move" filename="librobotnik_force_based_move.so">
       <commandTopic>robotnik_base_control/cmd_vel</commandTopic>
       <odometryTopic>robotnik_base_control/odom</odometryTopic>
       <odometryFrame>${prefix}odom</odometryFrame>
       <yaw_velocity_p_gain>10000.0</yaw_velocity_p_gain>
       <x_velocity_p_gain>10000.0</x_velocity_p_gain>
       <y_velocity_p_gain>10000.0</y_velocity_p_gain>
       <robotBaseFrame>${prefix}base_footprint</robotBaseFrame>
       <commandWatchdog>1</commandWatchdog>
       <odometryRate>50.0</odometryRate>
       <publishOdometryTf>${publish_tf}</publishOdometryTf>
   </plugin>
</gazebo>
```

