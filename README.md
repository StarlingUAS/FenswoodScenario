# Fenswood FARM

This repository contains a small set of files that set up an ArduCopter vehicle in a representation of the Fenswood
FARM. It leverages a number of images from Project Starling which holds the underlying framework.

## Docs

A slightly prettier version of the docs along with a [tutorial](https://starlinguas.github.io/FenswoodScenario/tutorials/fenswood_scenario/) can be found at [starlinguas.github.io/FenswoodScenario](https://starlinguas.github.io/FenswoodScenario/)

## Usage

First, ensure all the required images are downloaded and up-to-date:

```sh
docker-compose -f docker-compose.linux.yml pull
# or on windows:
docker-compose -f docker-compose.windows.yml pull
```

Then launch the scenario with:

```sh
docker-compose -f docker-compose.linux.yml up
# or on windows:
docker-compose -f docker-compose.windows.yml up
```

The Gazebo web interface is then available on [localhost:8080](http://localhost:8080).

On Linux, any MAVLink compatible GCS can be connected to UDP 14550. Many GCS will do this automatically.

On Windows, any MAVLink compatible GCS can be connected to TCP 5761.

### Troubleshooting

The simulator needs to download some model files when it is first run so it will likely fail to spawn the vehicle.
Leave it running for a few minutes then use `Ctrl+C` to exit and run it again. This should allow the model to spawn.
The first time the model attempts to spawn, the simulator will need to download files for it too so it may be slow to
start. Subsequent runs should be much faster.

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

## Target Spawning

For the purpose of the project, a volcano has to be modelled with dynamic areas representing danger around the base.
Target spawning is achieved via the `spawn_target.py` script. It spawns a specified number or yellow target rectangles and red hotspot circles around the 'annulus' of a volcano.

1. The Yellow Target Rectangles represent areas of dangerous terrain where landing would be unsafe (pyroclastic flows etc)
2. The Red Hotspot Circles represent areas of no-flight which would be dangerous (geysers, gas vents etc)

### Spawning Algorithm

There has to be an element of randomness in the spawning. Since the annulus is modelled as a circle, the placement of targets is based on the radial angle theta, on the edge of the annulus. In addition the hotspots should be clustered around each yellow target either on or near them. Here is the process:

1. For each primary target, an angle theta is selected uniformly. For each additional target, random thetas are drawn which satisfy a minimal angle between existing targets (first valid theta chosen).
2. The hotspot locations are then drawn from a normal distribution with a given variance around the targets angle and radial distances to force clustering. Hotspot locations are drawn until valid hotspots which are sufficiently far from each other are found.
3. These locations are then dynamically generated into an sdf XML representing the volcano.

### Spawning Environment Variable Options

The primary options are given by the following:

Name                  | Default Value                | Description
----------------------|------------------------------|------------
SPAWN_TARGET_NUM_TARGETS |  2             | Number of Yellow Targets being spawned
SPAWN_TARGET_NUM_HOTSPOTS |  5            | Number of Red Targets being spawned
SPAWN_TARGET_HOTSPOT_RADIUS |     3           | Radius of Red circular hotspots
SPAWN_TARGET_TARGET_LENGTH |      20           | Length of yellow target
SPAWN_TARGET_TARGET_WIDTH |       5       | Width of yellow target
SPAWN_TARGET_RANDOM_SEED |       None        | Random seed for random generation, used for deterministic arrangements.
SPAWN_TARGET_FILE_PATH |         ''        | File Path to fixed spawn, see example file i 'fenswood/target/default_target.json'

Change Size and Location of Volcano Annulus:

Name                  | Default Value                | Description
----------------------|------------------------------|------------
SPAWN_TARGET_ANNULUS_RADIUS |   40    | Radius of Volcano
SPAWN_TARGET_ANNULUS_LOC_X |  -195  |   Location Relative to Spawn/Takeoff Location
SPAWN_TARGET_ANNULUS_LOC_Y |   -163 |   Location Relative to Spawn/Takeoff Location
SPAWN_TARGET_ANNULUS_LOC_Z |   0.1  |   Location Relative to Spawn/Takeoff Location

Change Random Spawn Parameters
Name                  | Default Value                | Description
----------------------|------------------------------|------------
SPAWN_TARGET_GEN_TARGET_LOC_ANGLE_MIN | -pi/2    | Target location generationg minimum angle
SPAWN_TARGET_GEN_TARGET_LOC_ANGLE_MAX |  pi/2   | Target location generation maximum angle
SPAWN_TARGET_GEN_HOTSPOT_ANGLE_VARIANCE |  0.3  | Hotspot sampling normal variance for angle variation relative to target centers
SPAWN_TARGET_GEN_HOTSPOT_RADIUS_VARIANCE | 0.5 | Hotspot sampling normal variance for radial variation relative to target centers
SPAWN_TARGET_GEN_BETWEEN_TARGET_LOC_ANGLE_MIN | pi/3   | Minimal angle targets
SPAWN_TARGET_GEN_BETWEEN_TARGET_LOC_ANGLE_MAX |     4 | Maximal distance between hotspots

Note that if minimal angle and maximal distance are set incorrectly, it may cause an indefinite sampling as valid samples cannot be generated.

### Setting variables

The variables can be set in the usual manner either in the docker run comamnd or in the docker-compose file.