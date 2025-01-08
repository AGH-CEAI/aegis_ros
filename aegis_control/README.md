# aegis_control

This package contains the [ros2_control](https://control.ros.org/) launch files for the Aegis robot station, which incorporate configuration files from the `aegis_description` package.

## Launch files

```
aegis_control/
├── launch
│   ├── start_drivers.launch.py
│   └── ur_driver.launch.py
```

| File                                                        | Description                                                                                                                           |
| ----------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------- |
| [start_drivers.launch.py](./launch/start_drivers.launch.py) | The main launch file to run the entire Aegis' `ros2_control` stack.                                                                          |
| [ur_driver.launch.py](./launch/ur_driver.launch.py)         | Launches nodes from the [ur_robot_driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver) to control the UR5e robot. |
