# Building Fenswood Scenario and the Simulator Stack from Source

This tutorial takes the reader through how to build the simulator stack and the necessary requirements for the Fenswood Project scenario from source. 

**THIS IS RECOMMENDED FOR THOSE RUNNING ON ARM64 ARCHITECTURES** 

This includes the new Mac models running M1 chipsets. 

[TOC]

## The System

This repository builds upon the ProjectStarling set of containers and therefore requires the following containers to be built

1. Ardupilot core gazebo simulator (`uobflightlabstarling/starling-sim-iris-ap`)
2. Ardupilot SITL (`uobflightlabstarling/starling-sim-ardupilot-copter`)
3. Mavros (`uobflightlabstarling/starling-mavros`)
4. Rosbridge Suite (`uobflightlabstarling/rosbridge-suite`)
5. Controller Base (`uobflightlabstarling/starling-controller-base`)
6. FenswoodScenario simulator (`uobflightlabstarling/starling-sim-iris-ap`) which is built upon (1)

> **Warning** These containers are all fairly extensive, so some may take quite a while to build. 

> **Note:** All of these by default will be built with the tag `latest`, i.e. `uobflightlabstarling/starling-mavros:latest`. This may or may not align with the docker-compose files so please check if errors occur!  

## Building The System

### Building the core functionality

The core functionality is all in the Project Starling Repository: [https://github.com/StarlingUAS/ProjectStarling](https://github.com/StarlingUAS/ProjectStarling). 

In your workspace, first clone and navigate into the repository:

```bash
git clone --depth=1 https://github.com/StarlingUAS/ProjectStarling.git
cd ProjectStarling
```

Then lets first build the simulation containers (1 and 2). These can be built using a Makefile in the simulator directory.

```bash
cd simulator
make iris-ap
```

> A `Makefile` is an easy to use format which describes simple commands which can be run using the `make` comamnd. Here we use it to simplify running the commands to build our system. 

Once the simulator has been built, we can then build the system components (3, 4, and 5). These can be built using a Makefile in the system directory

```bash
cd system
make mavros
make controller-base
make rosbridge-suite
```

### Building the Fenswood Scenario

The Fenswood Scenario container in this repository is built on top of the Ardupilot core gazebo simulator (`uobflightlabstarling/starling-sim-iris-ap`) container we built earlier. Therefore we must make sure we have built it before we continue. 

Now you can navigate into the FenswoodScenario repository. If you have not yet cloned the repository, please clone it in your local workspace (outside of ProjectStarling).

```bash
git clone --depth=1 https://github.com/StarlingUAS/FenswoodScenario.git
cd FenswoodScenario
make
```

The make command above will automatically find the arduipilot core simulator we built previously and include the new parts which make it into the FenswoodScenario simulator. 

The simulator should now be ready to use during this tutorial, and during the fenswood volcano templates tutorials. 