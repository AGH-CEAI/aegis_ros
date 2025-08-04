# aegis_utils

This package provides utility functions and tools for the Aegis robot station.

---

## Calibration Data Collection
This tool captures images from a specified camera as the robot moves through a sequence of predefined joint configurations, saving both the images and the corresponding TCP poses for calibration purposes.

### Supported camera names:
- `scene`
- `tool_front_right`
- `tool_front_left`
- `tool_right`
- `tool_left`

### Example usage:
```bash
ros2 run aegis_utils calibrate -c tool_front_right
```
