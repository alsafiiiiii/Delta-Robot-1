# G-Code Test Files

This directory contains G-code files for testing the Delta Robot's motion and pick-and-place capabilities.

## Files

* `pick_and_place.gcode`: Main test sequence for PnP operation.
* `square_test.gcode`: Moves the effector in a square pattern.
* `circle_test.gcode`: Interpolated circle test.
* `star_test.gcode`: Star pattern test.
* `triangle_test.gcode`: Triangle pattern test.

## Usage

Run these files using the interpreter:

```bash
ros2 run velocity_pub delta_gcode_interpreter.py src/velocity_pub/gcode/pick_and_place.gcode
```
