Publishes a ground truth odometry. Can be used as odometry for robot or to compare with another odometry

Sample configuration:

```
<gazebo>
  <plugin name="ground_truth_controller" filename="librobotnik_gazebo_ground_truth_odom.so">
    <alwaysOn>true</alwaysOn>
    <updateRate>50.0</updateRate>
    <bodyName>base_link</bodyName> <!-- robot body to track, in gazebo -->
    <frameName>base_link_gt</frameName> <!-- frame name of robot body to be published in ros -->
    <worldBodyName>world</worldBodyName> <!-- fixed body where body is tracked, in gazebo -->
    <worldFrameName>odom</worldFrameName> <!-- frame name of fixed body to be published in ros -->
    <topicName>odom_gt</topicName>
    <gaussianNoise>0.01</gaussianNoise>
    <xyzOffsets>0 0 0</xyzOffsets>
    <rpyOffsets>0 0 0</rpyOffsets>
    <broadcastTF>True</broadcastTF>
  </plugin>
</gazebo>
```
