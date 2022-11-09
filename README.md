# Fenswood FARM

This repository contains a small set of files that set up an ArduCopter vehicle in a representation of the Fenswood
FARM. It leverages a number of images from Project Starling which holds the underlying framework.

## Docs

A slightly prettier version of the docs along with a [tutorial](https://starlinguas.github.io/FenswoodScenario/tutorials/fenswood_scenario/) can be found at [starlinguas.github.io/FenswoodScenario](https://starlinguas.github.io/FenswoodScenario/)

These docs also contain more information on target spawning and the attatch/detach mechanics

## Usage

First, ensure all the required images are downloaded and up-to-date:

```sh
docker-compose pull
```

Then launch the scenario with:

```sh
docker-compose up
```

> *Note:* If you wish to develop natively, or connect to ROS systems running locally on your system, you can run the
> system in host mode by using `docker-compose.host.yml` file instead:
> ```
> docker-compose -f docker-compose.host.yml up
> ```

The Gazebo web interface is then available on [localhost:8080](http://localhost:8080).
The Example Dashboard interface is then available on [localhost:3000](http://localhost:3000).

On Linux, any MAVLink compatible GCS can be connected to UDP 14550. Many GCS will do this automatically.

On Windows, any MAVLink compatible GCS can be connected to TCP 5761.

### Troubleshooting

The simulator needs to download some model files when it is first run so it will likely fail to spawn the vehicle.
Leave it running for a few minutes then use `Ctrl+C` to exit and run it again. This should allow the model to spawn.
The first time the model attempts to spawn, the simulator will need to download files for it too so it may be slow to
start. Subsequent runs should be much faster.

### Connecting to other things

When docker-compose is run, it will create its own private network to connect between the containers specified in the file.
The default `docker-compose.yml` file variant creates network named `fenswoodscenario_default` (named after the parent folder + `_default`). This network is segregated from your local network traffic *except* for the exposed ports specified in the docker-compose file which are now accessible from `localhost`. Any other ROS2 nodes will need to be wrapped in a docker container for running and run with `--network fenswoodscenario_default`.

> *Note:* If using the host docker-compose file then you do not need to specify any networks or exposed ports.

## Developing your own ROS2 controller

An example offboard ROS2 controller can then be conncted to SITL by running the following in a separate terminal:

```
# Download the latest container
docker pull uobflightlabstarling/example_controller_python

docker run -it --rm --network fenswoodscenario_default uobflightlabstarling/example_controller_python
```

See [the docs](https://docs.starlinguas.dev/guide/single-drone-local-machine/#2-running-example-ros2-offboard-controller-node) for further details
