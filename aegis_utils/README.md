# aegis_utils

This package provides utility functions and tools for the Aegis robot station.

---

## Calibration data collection
This tool captures images from a specified camera as the robot moves through a sequence of predefined joint configurations, saving both the images and the corresponding TCP poses for calibration purposes.

### Supported camera names:
- `scene`
- `tool_front_right`
- `tool_front_left`
- `tool_right`
- `tool_left`

### Example usage:
To gather calibration data, specify the camera using the `-c` argument. For example:
```bash
ros2 run aegis_utils calib_data_collect -c tool_front_right
```

By default, the collected images and TCP poses are saved to:

`~/ceai_ws/calibration_data/`

You can optionally specify a custom path and folder name using the `-p` argument. For example:
```bash
ros2 run aegis_utils calib_data_collect -c tool_front_right -p ~/Documents/calib_data
```

> [!IMPORTANT]
>
> Before calibrating the front tool camera (Luxonis OAK-D SR), you may need to turn off the laser dot projector to avoid interference.
>
> Edit the configuration file:
>
> `aegis_control/config/cameras/depthai_cameras.yaml`
>
> Set the following parameter for `/cam_tool/ros__parameters/camera`:
>
> ```yaml
> i_laser_dot_brightness: 0
> ```
