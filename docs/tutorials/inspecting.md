
# Inspecting and Debugging Starling with Docker and ROS

Now we have the full example Fenswood Scenario running, we can start to inspect what's going on a bit more! This will be more useful when developing your own controller, but this will give you an idea of whether your systems are working or not.

## Inspecting Docker

As you will hopefully be aware now, there's a fair amount going on under the hood! Because of all of these pieces which need to fit together, occasionally things fail, and therefore you will need to be able to inspect whats going on.

As most of Starling relys on using **Docker** we can use some of the docker commands to inspect what is going on!

It is useful to know what docker containers exist locally on your system and whether they are the most recent release. The following command will list your local docker containers images and when they were last released
```console
myuser@my-machine:~$ docker images
uobflightlabstarling/starling-sim-iris-ap                                     latest    02ba82372286   6 weeks ago   5.07GB
uobflightlabstarling/starling-sim-iris                                        <none>    826dab23768b   6 weeks ago   5.84GB
uobflightlabstarling/starling-sim-ardupilot-gazebo                            latest    df39b5d0181e   6 weeks ago   5.05GB
uobflightlabstarling/starling-sim-ardupilot-copter                            <none>    6a10c0c95714   6 weeks ago   2.35GB
uobflightlabstarling/starling-mavros                                          <none>    3a5142abfc0c   6 weeks ago   2.04GB
uobflightlabstarling/rosbridge-suite                                          latest    97a05ece1aa3   6 weeks ago   887MB
uobflightlabstarling/starling-ui-example                                      latest    83e9a37ddbdf   6 weeks ago   1.33GB
```

To see what containers are running, you can use the following. When running Fenswood you should see 5 containers running.
```console
myuser@my-machine:~$ docker ps
CONTAINER ID   IMAGE                                                       COMMAND                  CREATED       STATUS         PORTS     NAMES
dfea0bbf9b69   uobflightlabstarling/starling-ui-example                    "/ros_entrypoint.sh …"   3 hours ago   Up 9 minutes             fenswoodscenario_ui-example_1
1b72f667a557   uobflightlabstarling/starling-mavros:latest                 "/ros_entrypoint.sh …"   3 hours ago   Up 9 minutes             fenswoodscenario_mavros_1
9d55eb4de60e   uobflightlabstarling/rosbridge-suite:latest                 "/ros_entrypoint.sh …"   3 hours ago   Up 9 minutes             fenswoodscenario_rosbridge-suite_1
5205bcf5495f   example_python_controller_controller                        "/ros_entrypoint.sh …"   3 weeks ago   Up 8 minutes             example_python_controller_controller_1
9fa66d47e21e   uobflightlabstarling/starling-sim-ardupilot-copter:latest   "/home/root/entrypoi…"   4 weeks ago   Up 9 minutes             fenswoodscenario_sitl_1
e5e47cc36ec1   uobflightlabstarling/starling-sim-iris-ap:latest            "/ros_entrypoint.sh …"   5 weeks ago   Up 9 minutes             fenswoodscenario_simhost_1
```

When running docker ps, it shows some useful information. You can use this information to inspect the inside of a container. Now what do we mean by the 'inside of a container'. Essentially the container allows us to run a pre-set set of instructions inside a blank mini version of linux. When we are debugging, it is sometimes really useful to have a look at what is going on inside this mini version of linux!

The `exec` command allows us to go inside one of these containers to make changes directly. Note that when we are inside, we are no longer in your own version of the desktop, and that changes made are persistent inside the container! When inside the container, you can run some of the ROS2 comamnds in the next section.
```
docker exec -it <container id> bash
```

e.g.
```console
myuser@my-machine:~$ docker exec -it 1b72f667a557 bash
root@my-machine:/ros_ws# 
```

If a particular docker container is not working properly, you can also kill a container:
```
docker kill <container id>
```

## Inspecting ROS2

As mentioned before, everything in starling is running ROS2. Therefore all of the ROS2 nodes, topics and services can be inspected and observed. We can do this inspection using the following few commands.

First ensure that you have `docker exec` into any of the containers. For example using the container id of the container labelled `starling-mavros`.

Once you are inside, you first need to run the following to enable ROS2 commands. (++tab++ autocompleting is available)
```
root@my-machine:~$ source /opt/ros/foxy/setup.bash
```

The first thing you can do is list all of the ROS2 nodes in the network `node` command:

```
root@my-machine:~$ ros2 node list
WARNING: Be aware that are nodes in the graph that share an exact name, this can have unintended side effects.
/gazebo
/rosapi
/rosapi
/rosbridge_websocket
/vehicle_1/camera_controller
/vehicle_1/example_controller
/vehicle_1/gimbal_small_2d
/vehicle_1/ros_bridge
```

This will show a list of all the nodes that ROS2 can find. You should see all of the nodes from the simulator and the example controller.

Then, we can inspect the list of available topics using the `topic` command.

```console
root@my-machine:~$ ros2 topic list
/client_count
/clock
/connected_clients
/emergency_stop
/mission_start
/parameter_events
/performance_metrics
/rosout
/vehicle_1/camera/camera_info
/vehicle_1/camera/image_raw
/vehicle_1/camera/image_raw/compressed
/vehicle_1/camera/image_raw/compressedDepth
/vehicle_1/camera/image_raw/theora
/vehicle_1/gimbal_tilt_cmd
/vehicle_1/gimbal_tilt_status
/vehicle_1/mavlink/from
/vehicle_1/mavlink/to
/vehicle_1/mavros/battery
/vehicle_1/mavros/distance_sensor/hrlv_ez4_sonar
/vehicle_1/mavros/distance_sensor/lidarlite_laser
/vehicle_1/mavros/distance_sensor/rangefinder
/vehicle_1/mavros/distance_sensor/temperature
/vehicle_1/mavros/global_position/global
/vehicle_1/mavros/global_position/velocity
/vehicle_1/mavros/image/camera_image
/vehicle_1/mavros/local_position/pose
/vehicle_1/mavros/manual_control/control
/vehicle_1/mavros/manual_control/send
/vehicle_1/mavros/mission/reached
/vehicle_1/mavros/mission/waypoints
/vehicle_1/mavros/px4flow/ground_distance
/vehicle_1/mavros/px4flow/raw/optical_flow_rad
/vehicle_1/mavros/safety_area
/vehicle_1/mavros/setpoint_accel/accel
/vehicle_1/mavros/setpoint_attitude/attitude
/vehicle_1/mavros/setpoint_attitude/cmd_vel
/vehicle_1/mavros/setpoint_attitude/thrust
/vehicle_1/mavros/setpoint_position/global
/vehicle_1/mavros/setpoint_position/global_to_local
/vehicle_1/mavros/setpoint_position/local
/vehicle_1/mavros/setpoint_raw/attitude
/vehicle_1/mavros/setpoint_raw/global
/vehicle_1/mavros/setpoint_raw/local
/vehicle_1/mavros/setpoint_velocity/cmd_vel_unstamped
/vehicle_1/mavros/state
/vehicle_1/mavros/vision_pose/pose
/vehicle_1/mavros/vision_pose/pose_cov
/vehicle_1/mavros/vision_speed/speed_twist
/vehicle_1/mavros/vision_speed/speed_vector
```

If there is a particular topic you want to inspect, you can use the `echo` command of `topic`, for example if we wanted to inspect the topic `/vehicle_1/mavros/state` we can run:

```
root@my-machine:~$ ros2 topic echo /vehicle_1/mavros/state
header:
  stamp:
    sec: 1639423734
    nanosec: 393874329
  frame_id: ''
connected: true
armed: false
guided: true
manual_input: true
mode: GUIDED
system_status: 3
---
header:
  stamp:
    sec: 1639423735
    nanosec: 435107761
  frame_id: ''
connected: true
armed: false
guided: true
manual_input: true
mode: GUIDED
system_status: 3
---
^C
```

> **Note:** Press ++ctrl+c++ to stop the echo stream

This should (assuming the connection to all of the other elements is working) start printing out the current state of the drone. Including whether it is armed or not, and which mode it is currently in.

Finally, you can also inspect the services on the network using the `service` command like the following:

```
root@my-machine:~$ ros2 service list
...
/unpause_physics
/vehicle_1/camera_controller/describe_parameters
/vehicle_1/camera_controller/get_parameter_types
/vehicle_1/camera_controller/get_parameters
/vehicle_1/camera_controller/list_parameters
/vehicle_1/camera_controller/set_parameters
/vehicle_1/camera_controller/set_parameters_atomically
/vehicle_1/example_controller/describe_parameters
/vehicle_1/example_controller/get_parameter_types
/vehicle_1/example_controller/get_parameters
/vehicle_1/example_controller/list_parameters
/vehicle_1/example_controller/set_parameters
/vehicle_1/example_controller/set_parameters_atomically
/vehicle_1/gimbal_small_2d/describe_parameters
/vehicle_1/gimbal_small_2d/get_parameter_types
/vehicle_1/gimbal_small_2d/get_parameters
/vehicle_1/gimbal_small_2d/list_parameters
/vehicle_1/gimbal_small_2d/set_parameters
/vehicle_1/gimbal_small_2d/set_parameters_atomically
/vehicle_1/mavros/cmd/arming
/vehicle_1/mavros/cmd/command
/vehicle_1/mavros/cmd/command_int
/vehicle_1/mavros/cmd/land
/vehicle_1/mavros/cmd/set_home
/vehicle_1/mavros/cmd/takeoff
/vehicle_1/mavros/cmd/trigger_control
/vehicle_1/mavros/mission/clear
/vehicle_1/mavros/mission/pull
/vehicle_1/mavros/mission/push
/vehicle_1/mavros/mission/set_current
/vehicle_1/mavros/set_mode
/vehicle_1/mavros/set_stream_rate
/vehicle_1/ros_bridge/describe_parameters
/vehicle_1/ros_bridge/get_parameter_types
/vehicle_1/ros_bridge/get_parameters
/vehicle_1/ros_bridge/list_parameters
/vehicle_1/ros_bridge/set_parameters
/vehicle_1/ros_bridge/set_parameters_atomically
/vehicle_1/set_camera_info
```