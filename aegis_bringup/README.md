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

TODO - embed dynamic PlantUML generation:
```markdown
![alternative text](http://www.plantuml.com/plantuml/proxy?cache=no&src=https://raw.github.com/plantuml/plantuml-server/master/src/main/webapp/resource/test2diagrams.txt
```

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

    package config {
        class ur_calibration << (Y,#ffffc9) YAML >> {}
    }

    package urdf {
        class aegis_control_urdf << (U,#bcffc8) urdf.xacro >> {}
    }

}

package aegis_description {
    class aegis_display << (L,#FF7700) LaunchFile >> {
        Launch urdf_launch/display.launch()
        Node static_transform_publisher()
    }

    package config {
        class controllers << (Y,#ffffc9) YAML >> {}
        class joint_limits << (Y,#ffffc9) YAML >> {}
        package ur5e {
            class update_rate << (Y,#ffffc9) YAML >> {}
        }
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
        Node scene_objects_manager_node()
    }
    package config {
        class aegis << (S,#c298e9) SRDF >> {}
        package move_group {
            class ompl_planning << (Y,#ffffc9) YAML >> {}
            class kinematics << (Y,#ffffc9) YAML >> {}
        }
        class moveit << (R,#ffffff) rviz >> {}
    }
}


aegis_bringup.bringup *-- aegis_control.start_drivers
aegis_bringup.bringup *-- aegis_moveit_config.move_group

aegis_control.start_drivers *-- aegis_control.robot_description
aegis_control.start_drivers *-- aegis_control.ur_driver
aegis_control.robot_description *-- aegis_control.urdf.aegis_control_urdf
aegis_control.ur_driver *-- aegis_description.config.ur5e
aegis_control.ur_driver *-- aegis_description.config.controllers

aegis_moveit_config.move_group *-- aegis_moveit_config.config
aegis_moveit_config.move_group *-- aegis_description.config.controllers
aegis_moveit_config.move_group *-- aegis_description.config.joint_limits
aegis_moveit_config.move_group *-- aegis_control.config.ur_calibration

skinparam classAttributeIconSize 0
hide << YAML >> members
hide << urdf.xacro >> members
hide << SRDF >> members
hide << rviz >> members
@enduml
```
