# Drone Gimbal

The iris model (`iris_demo`) spawned has access to a one axis pitch only gimbal which can be controlled by giving an absolute angle (in radians). The source code for this gimbal is in the `ProjectStarling` repository.

## Using the gimbal

The gimbal is controlled using a single Publisher/Subscriber pair

- `/<Vehicle_id>/gimbal_tilt_cmd` (std_msgs/msg/Float32): Subscriber takes a topic `{"data: 0.5"}` and moves the gimbal to 0.5 radians down from the horizontal. There exists an onboard P controller to do this.
- `/<vehicle_id>/gimbal_tilt_status` (std_msgs/msg/Float32): Publisher which publishes the radian angle of the gimbal