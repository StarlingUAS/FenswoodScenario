# Payload pick up and drop off

Added for the project 2022-2023

A new scenario has been created where the user needs to drop off a sensor package at the caldera of the vehicle, and then after sampling for a minute, pick it back up again.

This repository contains a simplified version of these mechanics which model its usage through ros topics, but not the physics of actually picking up and depositing an object, as well as ignoring the effect of a payload on the dynamics of the flight.

Therefore the mechanics can be summarised as follows.

1. The vehicle spawns above a sensor payload and initially attempts to attach to it when the vehicle is within 1m above the payload. The payload is rigidly attached.
2. The vehicle should fly as normal with the payload to the caldera
3. On deposit, the user sends a `{"data: false"}` topic to `/attach_vehicle/control` in order to detach the payload.
4. On pick up, the vehicle flies within 1m above the payload and sends a `{"data: true"}` topic to `/attach_vehicle/control` in order to attach the payload.

Note that this mechanic is only available if using the `fenswood_with_payload.world`. This world file also contains some of the parameters for this rosnode.

## Attaching Ros Node

The ros/gazebo node which governs the connection is in `simulation/attach_vehicles_plugin`. It is a bit of reused code from a UGV/UAV project which allowed one to carry the other.

In short, in gazebo, it is exceedingly difficult to model proper collision. Any attempt to use the physics engine to "pick up" and object will end up with extreme numerical errors and effects such as rubber banding and the like. Therefore we use a shortcut where we physically join the two models together rigidly in a dynamic fashion.

This plugin essentially monitors a `drone_model` and a `payload_model`. It creates one Publisher and one Subscriber of the following

- `attach_vehicle/control` (std_msgs/Bool): Subscriber to attempt to attach (true) or detach (false) the payload
- `attach_vehicle/status` (std_msgs/Bool): Publisher to show the status of the attaching. True for attached, False for detached.


When an attach command is requested, it goes into the simulation and finds the drone and payload models. A model in gazebo is made up of static links (contains geometry, collision, etc) and joints which join the links together. It gets the root link (a.k.a the base_link) of each of these models and checks the distances to each other. Currently it checks that the drone is within `allowable_offset_height` height above the payload, and within `allowable_offset_horizontal` in the X,Y plane. If the models are within the tolerated distance, a joint is dynamically created which rigidly joins the vehicle to the payload.

When a detach command is requested, this dynamic joint is removed.

> *NOTE* At simulation startup, the vehicle and payload use the same attach mechanic to initially attach to each other. Sometimes this can fail (e.g. if the payload falls through the world for some reason). The distance between the vehicle and the payload is logged. If this fails, restart the simulation. If it continually happens, your computer may be struggling with the load.

## Implementing support in your controllers

As mentioned above, this mechanic only uses simply rostopics. Therefore in your controller you will only need to add a new Publisher/Subscriber pair. Then when you want the attach/detach to happen, you will need to publish a `std_msgs/msg/Bool` to the Publisher. The standard ROS2 docs should cover this use case in both Python and CPP.
