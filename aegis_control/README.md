# aegis_control

This package contains the ROS 2 control configuration files of the Aegis robot station, which extends the `aegis_description` package.

## Configuration files

```
aegis_control/
├── config
│   ├── ros2_controllers.yaml
│   └── ur_calibration.yaml
```

* `ros2_controllers.yaml` - Config for the all ros2_control controllers.
* `ur_calibration.yaml` - The extracted configuration file from the robot ([info](https://docs.universal-robots.com/Universal_Robots_ROS_Documentation/doc/ur_robot_driver/ur_calibration/doc/usage.html)).
