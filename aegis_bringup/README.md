# aegis_bringup

This package provides the configuration and launch files required to enable all basic functionalities of the Aegis robot station.

## Launch files

- `bringup.launch.py`: Responsible for running the entire ROS 2 stack for the robot station.

## Run the project

> [!CAUTION]
> Never startup the project on the real hardware unattended!

1. Ensure that the `ros2_driver` program on the robot is loaded, and the robot is set to the `Remote` mode (the top-right corner on the teach pendant).

2. Run the main launch file:
```bash
ros2 launch aegis_bringup bringup.launch.py
```

3. Apply the hotfix publisher for the gripper state:
```bash
ros2 topic pub /joint_states sensor_msgs/msg/JointState \
"header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: ''
name: ["hande_right_finger_joint", "hande_left_finger_joint"]
position: [0.025, 0.025]
velocity: [0.0, 0.0]
effort: [0.0, 0.0]"
```

4. Start the client program on the robot via dashboard service:
```bash
ros2 service call /dashboard_client/play std_srvs/srv/Trigger {}
```

## Launch files diagram

```plantuml
@startuml Launch Files

package aegis_bringup {
    class bringup << (L,#FF7700) LaunchFile >> {
        Arg namespace=""
        Arg mock_hardware="false"
    }
}

package aegis_control {
    class start_drivers << (L,#FF7700) LaunchFile >> {
        Arg namespace
        Arg mock_hardware
    }
    class robot_description << (L,#FF7700) LaunchFile >> {
        Arg namespace
        Arg mock_hardware
        Node robot_state_publisher()
    }
    class ur_driver << (L,#FF7700) LaunchFile >> {
        Arg namespace
        Arg mock_hardware
        Node control_node()
        Node ur_control_node()
        Node dashboard_client_node()
        Node tool_communication_node()
        Node controller_stopper_node()
        Node urscript_interface()
        Node controller_spawners()
    }

}

package aegis_description {
    class aegis_display << (L,#FF7700) LaunchFile >> {
        Launch urdf_launch/display.launch()
        Node static_transform_publisher()
    }
}

package aegis_moveit_config {
    class move_group << (L,#FF7700) LaunchFile >> {
        Arg namespace
        Arg fake_hardware
        Arg launch_rviz
        Node move_group_node()
        Node rviz_node()
        Node tf_robot_base_node()
        Node tf_odom_node()
        Node robot_state_publisher_node()
        Node scene_objects_manager_node()
    }
}


aegis_bringup.bringup --> aegis_control.start_drivers
aegis_bringup.bringup --> aegis_moveit_config.move_group
aegis_control.start_drivers --> aegis_control.robot_description
aegis_control.start_drivers --> aegis_control.ur_driver

skinparam classAttributeIconSize 0
@enduml
```
