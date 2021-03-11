# Set it up!
- It's difficult to make it works with an URDF. URDF has some limitations about the number of visuals that it can declare for a single link and sometimes is not clear the name that the visual has, for this reason I hardly recommend a native sdf to make this plugin works.

You can found an example at rbares_common (led stripes).

```
      <plugin name='robotnik_gazebo_set_visual' filename='librobotnik_gazebo_set_visual.so'>
        <modelName>robot_light_link</modelName>
        <visual>
          <visualName>robot_visual</visualName>
        </visual>
        <visual>
          <visualName>robot_visual_2</visualName>
        </visual>
        <visual>
          <visualName>robot_visual_3</visualName>
        </visual>
      </plugin>
```

The parameters are the name of the link(modelName) and the name of the visuals(visual/visualName). The visuals are specified like blocks inside the 'visual' tag to make the improvment of the plugin easier.

The plugins defines a ROS service (~/set_visual) that transforms the required gazebo_msgs/SetLightProperties msg into a gazebo::msgs::visual msg and then it publishes the order through gazebo's '~/visual' topic.

# Improve me!
TODO: 
- Update this Readme...
