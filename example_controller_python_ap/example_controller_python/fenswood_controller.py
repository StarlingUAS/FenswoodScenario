import math
import copy
import os

import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Empty
from geometry_msgs.msg import PoseStamped
import mavros_msgs.msg, mavros_msgs.srv


class RateLimiter():
    def __init__(self,min_period,clock):
        self.min_period = rclpy.duration.Duration(seconds=min_period)
        self.clock = clock
        self.time_of_last = None
    
    def call(self,func):
        if self.time_of_last is None:
            self.time_of_last = self.clock.now()
            time_since_last = self.min_period
        else:
            time_since_last = self.clock.now() - self.time_of_last
        
        if time_since_last > self.min_period:
            func()
            self.time_of_last = self.clock.now()

class DemoController(Node):

    def __init__(self):
        super().__init__('demo_controller')
        self.logger = self.get_logger()
        
        self.logger.info(f'Controller namespace is {self.get_namespace()}')

        self.start_mission_subscriber = self.create_subscription(
            Empty,
            '/mission_start',
            self.mission_start_callback,
            1)

        self.emergency_stop_subscriber = self.create_subscription(
            Empty,
            '/emergency_stop',
            self.emergency_stop_callback,
            1)

        self.position_subscriber = self.create_subscription(
            PoseStamped,
            f'mavros/local_position/pose',
            self.position_callback,
            1)
        
        self.state_subscriber = self.create_subscription(
            mavros_msgs.msg.State,
            f'mavros/state',
            self.state_callback,
            1)

        self.setpoint_publisher = self.create_publisher(
            PoseStamped,
            f'mavros/setpoint_position/local',
            1)

        self.offboard_rate_limiter = RateLimiter(1,self.get_clock())
        self.offboard_client = self.create_client(
            mavros_msgs.srv.SetMode,
            f'mavros/set_mode')

        self.stream_rate_limiter = RateLimiter(1,self.get_clock())
        self.stream_rate_client = self.create_client(
            mavros_msgs.srv.StreamRate,
            f'mavros/set_stream_rate')   

        self.arm_rate_limiter = RateLimiter(1,self.get_clock())
        self.arming_client = self.create_client(
            mavros_msgs.srv.CommandBool,
            f'mavros/cmd/arming')

        self.command_rate_limiter = RateLimiter(1, self.get_clock())
        self.command_client = self.create_client(
            mavros_msgs.srv.CommandLong,
            f'mavros/cmd/command')
        
        self.takeoff_client = self.create_client(
            mavros_msgs.srv.CommandTOL,
            f'mavros/cmd/takeoff')

        self.landing_client = self.create_client(
            mavros_msgs.srv.CommandTOL,
            f'mavros/cmd/land')
        
        self.takeoff_offset = 0
        self.land_offset = None
        self.flight_height = 5.0

        self.controller_command = 'Init'

        self.state = 'Init'

        timer_period = 0.02
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.vehicle_position = None
        self.vehicle_state = mavros_msgs.msg.State()


        self.land_start_time = None
        self.land_duration_before_estop = rclpy.duration.Duration(seconds=60.0)

        # List of points (time, x, y, yaw)
        self.trajectory_points = [
            (0, 10, 10, 0.0),
            (30, 100, 40, 0.0),
            (60, 300, 100, math.pi/2.0),
            (90, 300, -100, math.pi/2.0),
            (120, 200, -160, math.pi/2.0),
            (150, 150, -180, math.pi/2.0),
        ]

        self.angle = 0
        self.trajectory_idx = 0
        self.initial_position = None
        self.previous_position = None
        self.flight_start_time = None
        self.flight_previous_time = None
        self.flight_duration = rclpy.duration.Duration(seconds=self.trajectory_points[-1][0])

    
    def mission_start_callback(self, msg):
        self.controller_command = "Run"
        self.logger.info('\033[92m' + 'MISSION START RECEIVED: ' + str(msg) + '\033[0m')

    def emergency_stop_callback(self, msg):
        self.controller_command = "eStop"
        self.logger.warn('\033[91m' + 'EMERGENCY: ESTOP PRESS RECEIVED' + '\033[0m')

    def position_callback(self,msg):
        self.vehicle_position = msg

    def state_callback(self,msg):
        self.vehicle_state = msg

    def timer_callback(self):
        
        if self.state == 'DEAD':
            self.logger.info('Drone in Dead State, Switch Off or Restart Controller')
            self.timer.cancel()
            self.destroy_node()

        if self.controller_command == 'eStop':
            # Currently will just land and disarm
            self.eSTOP()
            self.state = 'DEAD'

        if self.state == 'Init':
            if self.initial_position==None and self.vehicle_position == None: # Request vehicle position from GCS
                commandCall = mavros_msgs.srv.StreamRate.Request()
                commandCall.stream_id = 6 # Request position streaming
                commandCall.message_rate = 20 # Request at 20hz
                commandCall.on_off = True
                self.stream_rate_limiter.call(lambda: self.stream_rate_client.call_async(commandCall))
                self.logger.info("Sent Set Message Interval Request (Ardupilot)")

            if self.initial_position == None and self.vehicle_position != None:
                self.initial_position = copy.deepcopy(self.vehicle_position)
                self.logger.info("Got initial position: ({},{},{})".format(
                    self.initial_position.pose.position.x,
                    self.initial_position.pose.position.y,
                    self.initial_position.pose.position.z
                    ))
                self.logger.info("Initialisation Confirmed, Going to ModeSwitch")
                self.state = 'ModeSwitch'

        if self.state == 'ModeSwitch':
            if self.vehicle_state.mode != "GUIDED":
                modeSetCall = mavros_msgs.srv.SetMode.Request()
                modeSetCall.custom_mode = "GUIDED"
                self.offboard_rate_limiter.call(lambda: self.offboard_client.call_async(modeSetCall))
            else:
                self.logger.info("Mode Switch Confirmed, Waiting for Mission Start, Before Arming")
                self.state = 'Arming'       
        
        if self.state == 'Disarming':
            if self.vehicle_state.armed == True:
                armingCall = mavros_msgs.srv.CommandBool.Request()
                armingCall.value = False
                self.arm_rate_limiter.call(lambda: self.arming_client.call_async(armingCall))
            else:
                self.logger.info("Disarm Completed")
                self.state = 'DEAD'

        setpoint_msg = None
        if self.initial_position != None:
            setpoint_msg = PoseStamped()
            setpoint_msg.header.stamp = self.get_clock().now().to_msg()
            setpoint_msg.pose = copy.deepcopy(self.initial_position.pose)

        if self.state == 'Land':
            # Send the Land command to ArduPilot
            landCommand = mavros_msgs.srv.CommandTOL.Request()
            self.command_rate_limiter.call(lambda: self.landing_client.call_async(landCommand))
                
            if self.vehicle_position.pose.position.z < 1.0 :
                self.state = 'Disarming'
                self.logger.info('Landing Confirmed, Going to Disarming')

        if self.controller_command == 'Run':
            if self.state == 'Arming':
                if self.vehicle_state.armed != True:
                    armingCall = mavros_msgs.srv.CommandBool.Request()
                    armingCall.value = True
                    self.arm_rate_limiter.call(lambda: self.arming_client.call_async(armingCall))
                else:
                    self.state = 'Takeoff'
                    self.logger.info("Arm Completed, Going to Takeoff")
                    
            if self.state == 'Takeoff':
                # Send the TakeOff command to ArduPilot
                takeoffCommand = mavros_msgs.srv.CommandTOL.Request()
                takeoffCommand.min_pitch = 0.0
                takeoffCommand.yaw = 0.0
                takeoffCommand.latitude = 0.0
                takeoffCommand.longitude = 0.0
                takeoffCommand.altitude = float(self.flight_height)
                self.command_rate_limiter.call(lambda: self.takeoff_client.call_async(takeoffCommand))
                
                setpoint_msg = None
                
                self.logger.info(f"Takeoff position: {self.vehicle_position.pose.position.z}:{self.flight_height * 0.95}")

                if self.vehicle_position.pose.position.z > self.flight_height * 0.95:
                    self.logger.info("Takeoff Complete, Going to Flight")
                    self.state = 'Flight'

            if self.state == 'Flight':
                current_time = self.get_clock().now()
                duration_since_start = (current_time - self.flight_start_time) if self.flight_start_time is not None else None
                if self.flight_start_time is None:
                    self.flight_start_time = current_time
                    self.flight_previous_time = current_time
                    self.previous_position = (0, self.vehicle_position.pose.position.x, self.vehicle_position.pose.position.y, 0)
                elif duration_since_start > self.flight_duration:
                    self.logger.info("Flight Completed, Going for Landing")
                    self.state = 'Land'                    
                elif duration_since_start > rclpy.duration.Duration(seconds=self.trajectory_points[self.trajectory_idx][0]):
                    # Move onto next point
                    self.previous_position = self.trajectory_points[self.trajectory_idx]
                    self.trajectory_idx += 1
                    self.logger.info(f"Moving to next point {self.trajectory_idx}, prev: {self.previous_position}")
                else:
                    # Linear Interpolation of setpoints
                    time_diff =  duration_since_start.nanoseconds - rclpy.duration.Duration(seconds=self.previous_position[0]).nanoseconds
                    traj_point = self.trajectory_points[self.trajectory_idx] # (time, x, y, yaw)
                    full_duration = rclpy.duration.Duration(seconds=traj_point[0] - self.previous_position[0]).nanoseconds
                    percentage_past = time_diff/full_duration
                    diffs = [pp + ((np - pp) * percentage_past) for np, pp in zip(traj_point, self.previous_position)]
                    self.logger.info(f"Setpoint: {diffs[1:]}")

                    setpoint_msg.pose.position.x = diffs[1]
                    setpoint_msg.pose.position.y = diffs[2]
                    setpoint_msg.pose.position.z = self.flight_height
                    if abs(diffs[3]) > 1e-5:
                        quat3 = math.sin(diffs[3]/2.0) * diffs[2] # Translate Yaw to quaternion
                        quat0 = math.cos(diffs[3]/2.0)
                        norm = math.sqrt(quat3*quat3 + quat0*quat0)
                        setpoint_msg.pose.orientation.x = quat0 / norm
                        setpoint_msg.pose.orientation.z = quat3 / norm 

        if setpoint_msg != None:
            self.setpoint_publisher.publish(setpoint_msg)

        
    def eSTOP(self):
        # eStop achieved by sending ARM command (400)
        # with param2 (Force) set to 21196
        commandCall = mavros_msgs.srv.CommandLong.Request()
        commandCall.broadcast = False
        commandCall.command = 400
        commandCall.param1 = 0.0
        commandCall.param2 = 21196.0
        commandCall.param3 = 0.0
        commandCall.param4 = 0.0
        commandCall.param5 = 0.0
        commandCall.param6 = 0.0
        commandCall.param7 = 0.0
        self.command_rate_limiter.call(lambda: self.command_client.call_async(commandCall))
        self.logger.warn('\033[91m' + 'EMERGENCY: KILL SIGNAL SENT' + '\033[0m')


def main(args=None):
    rclpy.init(args=args)

    demo_controller = DemoController()
    demo_controller.get_logger().info('STARTING DEMO CONTROLLER NOW!')

    try:
        rclpy.spin(demo_controller)
    except rclpy.handle.InvalidHandle as e:
        # Raised by node destroying itself.
        demo_controller.get_logger().info('demo_controller exited')
    else:
        demo_controller.destroy_node()

    rclpy.shutdown()

if __name__ == "__main__":
    main()
