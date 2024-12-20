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

## Configuration files

```
aegis_description/
├── config
│   ├── ur5e
│   │   ├── calibration.yaml
│   │   ├── joint_limits.yaml
│   │   ├── physical_parameters.yaml
│   │   ├── update_rate.yaml
│   │   └── visual_parameters.yaml
│   ├── controllers.yaml
│   └── initial_positions.yaml
```

Legend:
* 📜 - URDFs
* 🚀 - Launch Files

| File                                                                           | Used in | Description                                                                                             |
| ------------------------------------------------------------------------------ | ------- | ------------------------------------------------------------------------------------------------------- |
| [config/ur5e/calibration.yaml](./config/ur5e/calibration.yaml)                 | 📜🚀    | The calibration parameters extracted from the UR5e robot.                                               |
| [config/ur5e/joint_limits.yaml](./config/ur5e/joint_limits.yaml)               | 📜🚀    | Limits for each of the joints, used both by the `ur_robot_driver` and `aegis_moveit_config`.            |
| [config/ur5e/physical_parameters.yaml](./config/ur5e/physical_parameters.yaml) | 📜      | Reference of the masses and interias taken from the official `ur_robot_driver`.                         |
| [config/ur5e/update_rate.yaml](./config/ur5e/update_rate.yaml)                 | 🚀      | Configuration for the `controller_manager` Node taken from the `ur_robot_driver`.                       |
| [config/ur5e/visual_parameters.yaml](./config/ur5e/visual_parameters.yaml)     | 📜      | Paths to the UR5e meshes in the `ur_description` package.                                               |
| [config/controllers.yaml](./config/controllers.yaml)                           | 🚀      | Config for the `ros2_control` controllers, mix of `ur_robot_driver` and MoveIt2 Setup Assistant output. |
| [config/initial_positions.yaml](./config/initial_positions.yaml)               | 📜      | Default joints positions.                                                                               |



## URDF files diagram

TODO - embed dynamic PlantUML generation:
```markdown
![alternative text](http://www.plantuml.com/plantuml/proxy?cache=no&src=https://raw.github.com/plantuml/plantuml-server/master/src/main/webapp/resource/test2diagrams.txt
```

```plantuml
@startuml URDF diagram

package aegis_description {
    class aegis_urdf << (U,#bcffc8) urdf.xacro >> {
        arg mock_hardware="false"
    }
    class aegis << (X,cyan) xacro >> {}
    class aegis_control << (R,#98b6e9) control.xacro >> {}
    class cylinders_inertial << (X,cyan) xacro >> {}

    package modules {
        class ur_definition << (X,cyan) xacro >> {}
        class adapter_to_sensor << (X,cyan) xacro >> {}
        class ft_schunk_axia80 << (X,cyan) xacro >> {}
        class adapter_from_sensor << (X,cyan) xacro >> {}
    }

    package config {
        class initial_positions << (Y,#ffffc9) YAML >> {}
        package ur5e {
            class joint_limits << (Y,#ffffc9) YAML >> {}
            class physical_parameters << (Y,#ffffc9) YAML >> {}
            class visual_parameters << (Y,#ffffc9) YAML >> {}
            class calibration << (Y,#ffffc9) YAML >> {}
        }
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
}

package ur_robot_driver {
    class rtde_output_recipe << (T,#ffffc9) TXT >> {}
    class rtde_input_recipe << (T,#ffffc9) TXT >> {}
}


aegis_description.aegis_urdf *-left- aegis_description.aegis_control
aegis_description.aegis_urdf *-right- aegis_description.aegis
aegis_description.aegis_urdf *-up- aegis_description.config.initial_positions
aegis_description.aegis *-- aegis_description.modules
aegis_description.aegis *-- robotiq_hande_gripper.robotiq_hande_gripper

aegis_description.modules.adapter_from_sensor *-- aegis_description.cylinders_inertial
aegis_description.modules.adapter_to_sensor *-- aegis_description.cylinders_inertial
aegis_description.modules.ft_schunk_axia80 *-down- net_ft_description.net_ft_sensor
aegis_description.modules.ur_definition *-down- ur_description.ur_macro
aegis_description.modules.ur_definition *-right- ur_robot_driver
aegis_description.modules.ur_definition *-down- aegis_description.config

hide members
show << urdf.xacro >> fields
@enduml
```
