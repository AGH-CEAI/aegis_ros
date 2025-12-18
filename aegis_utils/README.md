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

## Calibration of camera intrinsics
This tool computes the intrinsic parameters of a camera using previously collected calibration images.

It outputs the resulting camera matrix and distortion coefficients and saves it to the calibration data folder.

### Example usage
It accepts the same arguments and uses the same default data path as the data collection tool described above. For example:
```bash
ros2 run aegis_utils calib_intrinsics -c tool_front_right
```

## Calibration of camera extrinsics
This tool computes the extrinsic parameters of a camera using previously collected calibration images, TCP poses and the computed camera intrinsics.

For tool-mounted cameras, it outputs the resulting transformation matrix from the robot TCP frame to the camera frame.
In the case of a scene camera, it outputs the transformation matrix from the robot base frame to the camera frame.
The resulting transformation matrix is saved to the calibration data folder.

### Example usage
It accepts the same arguments and uses the same default data path as the data collection tool described above. For example:
```bash
ros2 run aegis_utils calib_extrinsics -c tool_front_right
```

## Measure scene camera transformation error
This tool verifies the calibration of the robot base to camera transformation.
It compares the position of a marker, computed from the camera image and transformed into the robot’s base frame, with the position obtained from the robot’s TCP.

The tool outputs the **mean error** and **standard deviation** for each measurement.

### Example usage
To start and keep freedrive active, you’ll have to frequently publish a `True` msg on the indicated topic.
Before starting the program in a new terminal, run the following command to enable the freedrive controller:
```bash
ros2 topic pub --rate 2 /freedrive_mode_controller/enable_freedrive_mode std_msgs/msg/Bool "{data: true}"
```

To run the program use:
```bash
ros2 run aegis_utils measure_camera_error
```

By default, the collected images and errors are saved to:

`~/ceai_ws/error_data/`

The camera setup data are located by default in:

`~/ceai_ws/src/aegis_ros/aegis_utils/config/`

You can optionally specify:
- a custom path and folder name for collecting the results using `-r`
- a path and folder for getting the camera data (camera_matrix and camera distortion) using the `-d`argument
- a tool offset from the robot flange (`tool0` frame) using the `-t` argument

For example:
```bash
ros2 run aegis_utils calib_data_collect -r ~/Documents/error_data -d ~/Documents/camera_data -t 0.00077 0.00053 0.26455
```

## Calibration results
For now, the JSON files containing intrinsic and extrinsic calibration results for each of the cameras are located in:

`aegis_utils/aegis_utils/config/`
