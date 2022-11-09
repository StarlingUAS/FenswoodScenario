# Target Spawning

For the purpose of the project, a volcano has to be modelled with dynamic areas representing danger around the base.
Target spawning is achieved via the `spawn_target.py` script. It spawns a specified number or yellow target rectangles and red hotspot circles around the 'annulus' of a volcano.

1. The Yellow Target Rectangles represent areas of dangerous terrain where landing would be unsafe (pyroclastic flows etc)
2. The Red Hotspot Circles represent areas of no-flight which would be dangerous (geysers, gas vents etc)

## Spawning Algorithm

There has to be an element of randomness in the spawning. Since the annulus is modelled as a circle, the placement of targets is based on the radial angle theta, on the edge of the annulus. In addition the hotspots should be clustered around each yellow target either on or near them. Here is the process:

1. For each primary target, an angle theta is selected uniformly. For each additional target, random thetas are drawn which satisfy a minimal angle between existing targets (first valid theta chosen).
2. The hotspot locations are then drawn from a normal distribution with a given variance around the targets angle and radial distances to force clustering. Hotspot locations are drawn until valid hotspots which are sufficiently far from each other are found.
3. These locations are then dynamically generated into an sdf XML representing the volcano.

## Spawning Environment Variable Options

The primary options are given by the following:

Name                  | Default Value                | Description
----------------------|------------------------------|------------
SPAWN_TARGET_NUM_TARGETS |  2             | Number of Yellow Targets being spawned
SPAWN_TARGET_NUM_HOTSPOTS |  5            | Number of Red Targets being spawned
SPAWN_TARGET_HOTSPOT_RADIUS |     3           | Radius of Red circular hotspots
SPAWN_TARGET_TARGET_LENGTH |      20           | Length of yellow target
SPAWN_TARGET_TARGET_WIDTH |       5       | Width of yellow target
SPAWN_TARGET_RANDOM_SEED |       None        | Random seed for random generation, used for deterministic arrangements.
SPAWN_TARGET_FILE_PATH |         ""        | File Path to fixed spawn, see example file i 'fenswood/target/default_target.json'

Change Size and Location of Volcano Annulus (Calculated from GPS positions):

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

## Setting variables

The variables can be set in the usual manner either in the docker run comamnd or in the docker-compose file. For example, you can create a new docker-compose file that is the same as the current one with the following modification to the `simhost` container:

```yaml
  simhost:
    image: uobflightlabstarling/starling-sim-iris-ap:${STARLING_RELEASE:-latest}
    environment:
      - AP_SITL_HOST=sitl
      - SPAWN_TARGET_RANDOM_SEED=22 # <- This specifies a random seed
      - SPAWN_TARGET_GEN_BETWEEN_TARGET_LOC_ANGLE_MAX=8
      - SPAWN_TARGET_GEN_HOTSPOT_ANGLE_VARIANCE=0.5
    volumes:
      - ./fenswood:/ros.env.d/fenswood
    command: ["ros2", "launch", "/ros.env.d/fenswood/iris.launch.xml"]
    ports:
      - "8080:8080"
```

With the Makefile commands, these can be run like so

```bash
make run ENV="-e SPAWN_TARGET_RANDOM_SEED=22"
```