# Fenswood Scenario Repository Layout

This document briefly explains the files and folders within this repository.

## Overview

At a high level, this repository generates the `starling-sim-iris-ap-fenswood` simulation docker container which will start up a gazebo simulation of Fenswood Farm with an Iris quadcopter with an attached payload and a dynamically generated volcano caldera.

It contains the following:

- Contains the world file which describes the fenswood farm environment (`fenswood.world`)
- Contains the script (`spawn_targets.py`) which dynamically generates the volcano caldera at runtime.
- Contains the gazebo ros2 plugin which allows for the attachment and detachment of a payload
- Contains the Dockerfile which compiles all of these into the `starling-sim-iris-ap` docker container
- Contains a docker-compose file which will automatically build the simulator without needing to build the container yourself manually
- Contains the documentation for `https://starlinguas.github.io/FenswoodScenario`
- Contains the `.github` to enable automated builds on github actions.
- Contains the example controller in `example_controller_python_ap`.

## World file

In the `fenswood/worlds` directory, there exist the `.world` sdf files which specify the layout and objects in the world. This specifyies all of the objects, as well as the GPS coordinates of the origin of the world.

To enable attach/detach, there is a second world which includes the `attach_vehicles` plugin `fenswood_with_payload.world`. This is enabled by default.

The models used are all in the `fenswood/models` folder.

The models which are not included may be in the upstream simulation containers such as `starling-sim-iris-ap` (e.g. the iris model (`iris_demo`))

The `fenswood/setup.bash` file copies all of the models in.

## Launching and target spawning

The `fenswood/iris.launch.xml` configuration determins what is being launched.

This calls the target spawning script is in `fenswood/spawn_targets.py` with an example target configuration in `fenswood/target`.

## Attaching/detaching payload

The `simulation/attach_vehicles_plugin` contains the Rosnode source for the global gazebo plugin which manages the attaching and detaching of sensor payloads.

The payload itself is defined in `fenswood/models/sensor_payload/model.sdf` and edts should be made there to change it.

The `fenswood/iris.launch.xml` launch file launches both the drone and the payload and attempts to automatically join them on statup.

## Building

There exists a `Makefile` which should provide automated commands for building and testing the `Dockerfile` simulator for this repository.

## Example controller

The example controller is in the `example_controller_python_ap` folder. This is a Python rosnode put into a local Docker container. The `docker-compose.example_drone_controller.yaml` docker-compose file is used to build and run this controller against the simulator run by `docker-compose.yml`
