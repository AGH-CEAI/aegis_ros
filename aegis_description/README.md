# aegis_description

This package contains a description of the Aegis robot station, which consists of the following modules:
- UR5e series
- SCHUNK FT Sensor AXIA 80
- Robotiq Hand-E Gripper

## Preview

```bash
ros2 launch aegis_description display.launch.py
```

![aegis_preview](./docs/aegis_preview.png)


## URDF files diagram

```plantuml
@startuml URDF diagram

package aegis_control {
    class aegis_control_urdf << (U,#bcffc8) urdf.xacro >> {
        arg mock_hardware="false"
    }
    class aegis << (R,#98b6e9) urdf.ros2_control >> {}
    class initial_positions << (Y,#ffffc9) YAML >> {}
}

package aegis_description {
    class aegis << (X,cyan) xacro >> {}
    class aegis_urdf << (U,#bcffc8) urdf.xacro >> {}
    class cylinders_inertial << (X,cyan) xacro >> {}

    package modules {
        class adapter_from_sensor << (X,cyan) xacro >> {}
        class adapter_to_sensor << (X,cyan) xacro >> {}
        class ft_schunk_axia80 << (X,cyan) xacro >> {}
        class ur_definition << (X,cyan) xacro >> {}
    }
}

package aegis_moveit_config {
    class aegis << (S,#c298e9) SRDF >> {}
}

package robotiq_hande_gripper {
    class robotiq_hande_gripper << (X,cyan) xacro >> {}
}

package net_ft_description {
    class net_ft_sensor << (X,cyan) xacro >> {}
}

package ur_description {
    class ur_macro << (X,cyan) xacro >> {}
    class joint_limits << (Y,#ffffc9) YAML >> {}
    class physical_parameters << (Y,#ffffc9) YAML >> {}
    class visual_parameters << (Y,#ffffc9) YAML >> {}
}

package ur_robot_driver {
    class rtde_output_recipe << (T,#ffffc9) TXT >> {}
    class rtde_input_recipe << (T,#ffffc9) TXT >> {}
}


aegis_control.aegis_control_urdf *-- aegis_control.aegis
aegis_control.aegis_control_urdf *-- aegis_description.aegis
aegis_control.aegis_control_urdf o-- aegis_control.initial_positions

aegis_description.aegis_urdf *-- aegis_description.aegis
aegis_description.aegis *-- aegis_description.modules
aegis_description.aegis *-- robotiq_hande_gripper.robotiq_hande_gripper

aegis_description.modules.adapter_from_sensor *-- aegis_description.cylinders_inertial
aegis_description.modules.adapter_to_sensor *-- aegis_description.cylinders_inertial
aegis_description.modules.ft_schunk_axia80 *-- net_ft_description.net_ft_sensor
aegis_description.modules.ur_definition *-- ur_description
aegis_description.modules.ur_definition *-- ur_robot_driver
aegis_description.modules.ur_definition o-- aegis_control.initial_positions

hide members
show << urdf.xacro >> fields
@enduml
```
