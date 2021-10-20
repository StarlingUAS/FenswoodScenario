# Fenswood FARM

This repository contains a small set of files that set up an ArduCopter vehicle in a representation of the Fenswood
FARM. It leverages a number of images from Project Starling which holds the underlying framework.

## Usage

First, ensure all the required images are downloaded and up-to-date:

```sh
docker-compose pull
```

Then launch the scenario with:

```sh
docker-compose up
```

The Gazebo web interface is then available on [localhost:8080](http://localhost:8080).

Any MAVLink compatible GCS can be connected to UDP 14550. Many GCS will do this automatically.