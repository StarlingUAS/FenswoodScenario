# Fenswood FARM

This repository contains a small set of files that set up an ArduCopter vehicle in a representation of the Fenswood
FARM. It leverages a number of images from Project Starling which holds the underlying framework.

## Usage

First, ensure all the required images are downloaded and up-to-date:

```sh
docker-compose -f docker-compose.linux.yml pull
# or on windows:
docker-compose -f docker-compose.windows.yml pull
```

Then launch the scenario with:

```sh
docker-compose -f docker-compose.linux.uml up
# or on windows:
docker-compose -f docker-compose.windows.yml up
```

The Gazebo web interface is then available on [localhost:8080](http://localhost:8080).

Any MAVLink compatible GCS can be connected to UDP 14550. Many GCS will do this automatically.

## Windows and Linux

Each example file has a *linux* and *windows* variant. 

- The *linux* variant allows you to run bare-metal application such as rviz2 or your own controllers natively. You do not need to wrap your own controllers in a docker container. Any exposed ports are automatically exposed to `localhost`. 
- The *windows* variant runs inside a docker-compose network named `fenswoodscenario_default`. This network is segregated from your local network traffic *except* for the exposed ports in the docker-compose file which are now accessible from `localhost`. Any other ROS2 nodes will need to be wrapped in a docker container for running and run with `--network fenswoodscenario_default`. 

## Developing your own ROS2 controller

An example offboard ROS2 controller can then be conncted to SITL by running the following in a separate terminal:

```
# Download the latest container
docker pull uobflightlabstarling/example_controller_python 

docker run -it --rm --network fenswoodscenario_default uobflightlabstarling/example_controller_python
```

See [the docs](https://docs.starlinguas.dev/guide/single-drone-local-machine/#2-running-example-ros2-offboard-controller-node) for further details

