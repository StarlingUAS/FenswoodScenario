
# Starling and Docker

So now we have installed this thing called **Docker** - but what is it, and why does it matter to you as a user of Starling. **For now, we recommend you treat `docker` as a special program that runs programs which we have written for you to use.** For the purposes of this session we will not yet go into the details - we'll leave that for a future session, but we can at least give you a flavour of what Docker has allowed us to do.


## What is the purpose of Starling

The purpose of Starling, for the use of this course, is to allow you to quickly and easily install and run a UAV simulation within a simulated environment, so that you can test your developed controllers against a semi-realistic scenario.

Therefore Starling is a set of pre-built programs/executables, some of which are pre-configured for the following:

*   Running a Physics Simulation with Visualisation
*   Running the Drone autopilot control software locally (a.k.a Software In The Loop or SITL)
*   Running the interface between Mavlink and other protocols such as the Robot Operating System (ROS)
*   And many others...

These pre-built programs, which we refer to as **containers**, are all available in the [StarlingUAS repository on github](https://github.com/StarlingUAS).

Together these **containers** form a modular ecosystem of drone systems which can be composed together to develop software for real drones. Any controllers developed via the simulator can be directly ported to run on a real drone.

## What 'magic' is docker doing

Perhaps it is easiest to explain what we (and you) would have to do if docker did not exist! Each of the elements described above uses its own programs. For example the physics simulator is a program called [gazebo](http://gazebosim.org/). On each of your machines you would have to manually install gazebo and configure it to work. Then you would have to do the same for SITL, Mavlink, ROS and so on, and then make sure all of them work together - configuration files, scripts etc etc. And still this is without considering how to write software which will run on a drone on a different computer altogther (we don't want the 'but sir, it worked on my machine' syndrome!).

With docker, we have essentially **packaged** up the software with all of it's dependencies and configurations to provide only the specific service we want it to provide. We have already figured out how to install and configure, say, gazebo, and how to get it running with everything else, put those instructions together and released it to you as a **container**.

Almost as importantly, because the installation and configuration is baked into the container, the system itself runs in a **consistent and replicatable environment**, hopefully allowing reproducable results each time. This can be different from running 'bare metal' on your local machine where you may have accidentally changed a configuration without knowing - leading to things going wrong.

All of this means that it's less work for you as the user, as you don't need to go through the difficulties of set-up and installation of all the many individual components. And it's better for us as we can get down to the interesting stuff quicker making life better for everyone!

We want to leave it like this for now, but if you want more detail you can see [this tutorial](https://starlinguas.github.io/starling_controller_templates/docker/) here. I would recommend you come back after you have finished section this tutorial.