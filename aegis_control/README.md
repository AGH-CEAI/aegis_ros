# aegis_control

This package contains the [ros2_control](https://control.ros.org/) launch files for the Aegis robot station.

## Launch files

```
aegis_control/
├── config
│    ├── cameras
│    │    └── cameras.yaml
│    └── controllers
│         ├── net_ft_broadcaster.yaml
│         ├── update_rate.yaml
│         └── ur_drivers.yaml
│
├── launch
│   ├── depthai_cameras_driver.launch.py
│   ├── ft_sensor_driver.launch.py
│   ├── start_drivers.launch.py
│   └── ur_driver.launch.py
```

| File/Directory                                                                | Description                                                                                                                                                   |
| ------------------------------------------------------------------------------| --------------------------------------------------------------------------------------------------------------------------------------------------------------|
| `config/cameras/*.yaml`                                                       | Parameters for Luxonis DepthAI cameras nodes, which are composed in the `depthai_cameras_driver.launch.py` file.                                              |
| `config/controllers/*.yaml`                                                   | All `controller_manager` node's parameters, which are composed in the `start_drivers.launch.py` file.                                                         |
| [depthai_cameras_driver.launch.py](./launch/depthai_cameras_driver.launch.py) | Launches DepthAI nodes from the [depthai_ros_driver](https://github.com/luxonis/depthai-ros/tree/humble/depthai_ros_driver) to acquire data from the cameras. |
| [ft_sensor_driver.launch.py](./launch/ft_sensor_driver.launch.py)             | Launches the [ros2_net_ft_driver](https://github.com/AGH-CEAI/ros2_net_ft_driver) to control the Schunk FT AXIA80 sensor.                                     |
| [start_drivers.launch.py](./launch/start_drivers.launch.py)                   | The main launch file to run the entire Aegis' `ros2_control` stack.                                                                                           |
| [ur_driver.launch.py](./launch/ur_driver.launch.py)                           | Launches nodes from the [ur_robot_driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver) to control the UR5e robot.                         |


## Development notes
* ROS 2 Humble ships with the older structure of the [Universal_Robots_ROS2_Driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/tree/humble) Due to the complexity of our project (where we need to merge several other controllers into a single `control_manager` ode configuration), the `ur_driver.launch.py` file is based on version `2.5.1` of UR's `ur_robot_driver` [launch file](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/blob/humble/ur_robot_driver/launch/ur_control.launch.py). The main difference is the removal of all unused parameters, which are set by default by the driver itself.
* Consequently, any distribution change (for instance, an upgrade to ROS 2 Jazzy) will result in catastrophic failure due to the incompatibility of parameters in the newer version of the driver (currently `3.0.1`). On the other hand, the newer driver simplifies many of our headaches regarding different configuration parameters.
