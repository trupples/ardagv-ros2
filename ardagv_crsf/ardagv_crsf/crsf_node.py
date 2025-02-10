import rclpy
from rclpy.node import Node
from crsf_parser import CRSFParser
from crsf_parser.payloads import PacketsTypes
from crsf_parser.handling import crsf_build_frame
from std_msgs.msg import String, Int16MultiArray, Float64MultiArray 
from canopen_interfaces.srv import CORead # SDO read service
from geometry_msgs.msg import TwistStamped
from std_srvs.srv import Trigger
from serial import Serial

class CRSFNode(Node):
    def __init__(self):
        super().__init__('crsf')
        self.cmd_publisher = self.create_publisher(TwistStamped, 'muxd_vel', 1) # Multiplexed velocity
        self.forward_publisher = self.create_publisher(Float64MultiArray, 'muxd_fwd', 1) # Multiplexed forward kinematics

        self.channel_publisher = self.create_publisher(Int16MultiArray, 'channels', 1)
        self.auto_cmd_subscription = self.create_subscription(TwistStamped, 'cmd_vel_auto', self.auto_cmd_callback, 5) # Automatic control velocity

        # Service client for reading supply voltage
        self.cli = self.create_client(CORead, '/ardagv_motor_left/sdo_read')

        # Service clients to stop / start the motors
        motors = ['/ardagv_motor_left', '/ardagv_motor_right']
        self.stop_services = [
            self.create_client(Trigger, motor + "/halt") for motor in motors
        ]
        self.start_services = [
            self.create_client(Trigger, motor + "/" + action) for motor in motors for action in ["init", "velocity_mode"]
        ]

        # Wait for services to become available
        self.get_logger().info(f"Waiting for motor nodes {motors} to become available:")
        for cli in [self.cli] + self.stop_services + self.start_services:
            self.get_logger().info(f"Waiting for service {cli.srv_name}")
            while not cli.wait_for_service(timeout_sec=5.0):
                self.get_logger().info(f"Waiting for service {cli.srv_name}")

        self.fast_timer = self.create_timer(0.1, self.fast_timer_callback)
        self.battery_timer = self.create_timer(1, self.battery_timer_callback)

        self.get_logger().info("All services up!")
        self.serial = Serial("/dev/ttymxc3", 420000, timeout=0.01)

        self.channels = []
        self.channels_decoded = []

        # Queued up bytes from serial
        self.incoming = bytearray()
        crsf_callback = lambda frame,status: self.crsf_callback(frame, status)
        self.crsf_parser = CRSFParser(crsf_callback)

        self.state = 'init' # init -[kill switch on]-> kill -[kill switch off]-> run -[kill switch on]-> kill

    def battery_timer_callback(self):
        # Called every second to request the batery voltage via SDO
        req = CORead.Request()
        req.index = 0x2122 # SUPPLY_VOLTAGE
        req.subindex = 0
       
        self.get_logger().debug('Requesting battery')
        future = self.cli.call_async(req)
        future.add_done_callback(lambda result: self.battery_value_callback(result))

    def battery_value_callback(self, future):
        # Called when we get an SDO response for SUPPLY_VOLTAGE
        res = future.result()
        if not future.done() or not res.success:
            self.get_logger().error("Couldn't get battery voltage")
            return

        self.get_logger().debug('Got battery')

        v = res.data # Happily enough, the TMC reports in 100mV units, and CRSF expects 100mV units
        rem = ((v*0.1) / 3 - 3) / (4.2 - 3) * 100 # Rough percentage if 3V = 0% and 4.2V = 100%

        battery_frame = crsf_build_frame(PacketsTypes.BATTERY_SENSOR, {"voltage": v, "current": 1, "capacity": 100, "remaining": int(rem)})

        self.serial.write(battery_frame)

    def crsf_callback(self, frame, status):
        # Called as part of fast_timer_callback every processed CRSF frame
        # print(f'{frame=} {status=}')
        if frame.header.type == PacketsTypes.RC_CHANNELS_PACKED:
            self.channels = list(frame.payload.channels)[::-1][:10]
            self.channels_decoded = [int(1500.5 + 5/8*(x - 992)) for x in self.channels]

    def fast_timer_callback(self):
        # Called quickly to process incoming CRSF
        self.incoming.extend(self.serial.read(10000))
        # self.get_logger().info(f"{self.incoming=}")
        self.crsf_parser.parse_stream(self.incoming) # May call crsf_callback
        
        if not self.channels:
            return

        channel_msg = Int16MultiArray()
        channel_msg.data = self.channels_decoded
        self.channel_publisher.publish(channel_msg)

        # Parse channels
        # rx, ry, lx, ly = x, y values of right / left stick
        rx, ry, ly, lx, sa, sb, sc, sd, se, sf = [(x - 1500) / 5 for x in self.channels_decoded] # Map 1000..2000 to -100..100
        # print(f"{rx=}\t{ry=}\t{lx=}\t{ly=}\t{sa=}\t{sb=}\t{sc=}\t{sd=}\t{se=}\t{sf=}")

        if self.state == 'init':
            if sa > 0:
                self.get_logger().info("Safety: Activate kill switch, so that deactivating it is intentional")
                return
            else:
                self.enter_kill()

        if self.state == 'kill':
            if sa > 0:
                self.enter_run()
            else:
                return

        # Running
        if sa < 0:
            self.enter_kill()
            return

        if sd > 0:
            if self.state != 'run_auto':
                self.get_logger().info("MODE: RUNNING AUTO")
            self.state = 'run_auto'
        else:
            if self.state != 'run_manual':
                self.get_logger().info("MODE: RUNNING MANUAL")
            self.state = 'run_manual'

            max_vel = 0.5 # m/s
            max_rot = 1 # rad/s

            twist = TwistStamped()
            twist.twist.linear.x = ry / 100 * max_vel
            twist.twist.angular.z = -rx / 100 * max_rot 
            self.get_logger().info(f"MANUAL:\t{twist.twist.linear.x:.01f} m/s\t{twist.twist.angular.z:.01f} rad/s")
            self.cmd_publisher.publish(twist)
            self.do_kinematics(twist.twist.linear.x, twist.twist.angular.z)

    def auto_cmd_callback(self, msg):
        if self.state == 'run_auto':
            self.get_loger().info(f"AUTO: {msg.twist.linear.x:.01f} m/s\t{msg.twist.angular.z:.01f} rad/s")
            self.cmd_publisher.publish(msg)
            self.do_kinematics(msg.twist.linear.x, msg.twist.angular.z)

    def enter_kill(self):
        self.state = 'kill'
        self.get_logger().info("MODE: KILLSWITCH")

        twist = TwistStamped() # Defaults to all zeros
        self.cmd_publisher.publish(twist)

        self.do_kinematics(0, 0)

        #for cli in self.stop_services:
        #    cli.call_async(Trigger.Request()) # Stop asynchronously to ensure fast response

    def enter_run(self):
        self.state = 'run' # Will be shortly changed to run_manual or run_auto
        self.get_logger().info("MODE: RUNNING")

        #for cli in self.start_services:
        #    self.get_logger().info(f"Calling {cli.srv_name}")
        #    cli.call_async(Trigger.Request()) # Start synchhronously to make sure order is ok

    def do_kinematics(self, linear, angular):
        # Substitute for diff drive node because it acts up
        w = 0.27 # m
        r = 0.05 # m

        vl, vr = linear - angular * w / 2, linear + angular * w / 2
        wl, wr = vl / r, vr / r

        forward = Float64MultiArray()
        forward.data = [wl, wr]
        self.forward_publisher.publish(forward)
        self.get_logger().debug("Kinematics: {wl=:.01} {wr=:.01}")

def main(args=None):
    rclpy.init(args=args)

    crsf_node = CRSFNode()

    rclpy.spin(crsf_node)

    crsf_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

