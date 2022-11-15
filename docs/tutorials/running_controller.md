
# Running the Example UAV Controller

The next step in getting the Fenswood Scenario up and running is to actually get the UAV to do something. The previous steps simply started the simulator up, but now we have to instruct and control the UAV. For now we have provided an example controller to all students in the same `FenswoodScenario` repository under `example_controller_python_ap`.

This section will take you through step by step as to how to download and run the example, a bit of information about what the example contains. This information is also available in the example repository readme.

## Running the Example Controller

First, open up a terminal and start up the Fenswood Scenario Simulator if you haven't already (Refer to [](#running-the-example-scenario)). Double check it is open by going to Gazebo Web at [localhost:8080](http://localhost:8080)

Then, **open up a second terminal** and navigate to the FenswoodScenario repository folder. The example controller can be started by running the following:

```console
myuser@my-machine:~/Documents/FenswoodScenario$ docker-compose -f docker-compose.example_drone_controller.yaml up
```

With this command, the controller will start off and attempt to find a drone on the network.

> If a drone has not been found, try restarting the both the Fenswood Scenario and the controller.

Once a drone has been found, it will attempt to connect to the ardupilot. Once connected the path the vehicle will take is initialised and it is ready to fly if the user sends a mission go.

> Be patient, sometimes the ardupilot SITL can be quite slow to respond. Leave it for up to 2 or 3 minutes before trying again.

Now to send a mission go, you can use the provided simple UI that is started with the FenswoodScenario. Navigate to [localhost:3000](http://localhost:3000) in the browser, and a UI will start up:

![Starling UI](imgs/ui.png)

If it says `connected to rosbridge` in green, then you are ready to send a mission start command. Press the Green GO Button.

If you go to the terminal it should start sending waypoints. In the UI the camera image should start to change as the vehicle takes off, and you should see the vehicle takeoff in gazebo web as well.

Then you should hopefully see its attempt to follow a preset trajectory towards the target location.

This full process can be seen in the following gif (The commands are outdated, but you should see something very similar):
![Fenswood Example Controller demo a](imgs/starling-fenswood-demo-2a.gif)

The landing:
![Fenswood Example Controller demo a](imgs/starling-fenswood-demo-2b.gif)

## What is the Example Controller

So what exactly is the example controller doing?

Lets first start with what it is communicating with. On a real drone, the thing that controls the lower level operations of the drone is the **Autopilot** or **Flight controller**. The autopilot contains software which allows for different flight modes and translates hight level commands to motor voltages to fly the drone. As you might have come across, the primary with of controlling the autopilot is through sending Mavlink messages.

Now the autopilot software itself can be swapped out and changed. For this scenario, we use the [**Ardupilot Arducopter**](https://ardupilot.org/copter/docs/introduction.html) firmware for the flight controller. It is important to know the specific type of software as different flight controller firmware requires the use of different flight mode and instructions.

The Fenswood Scenario simulator utilises the Ardupilot Software In The Loop (SITL) simulator. This program is identical to the firmware that would be running onboard the flight controller. Then, within the example controller repository, there are two example controllers - one for Ardupilot (suffixed with `ap`), and one for the PX4 firmware (suffixed with `px4`). You can use the former to communicate with the Ardupilot SITL.

The example controller talks in ROS2 to the SITL via the Mavros translation node mentioned earlier. It sends commands for things like 'takeoff', 'go there' and 'land' via topics which the Mavros node advertises. The Mavros node takes subscribes to these topics and re-publishes them to the Flight Controller using Mavlink in a way Ardupilot understands.

More details will be given in the next tutorial about how this controller works under the hood, and how to develop your own.