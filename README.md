# Arduino AGV platform

Connect to the Portenta X8 using hostname `portenta-arduino`, username `analog`, password `analog`.

All relevant ros2 nodes are built inside the [shooteu/full-agv-setup](https://hub.docker.com/r/shooteu/full-agv-setup) docker image. The following flags (except for `--name`) are important:

```
docker run --privileged --network=host --cap-add=NET_ADMIN -itd --name=ros shooteu/full-agv-setup:2025-02-14
```

Inside the docker container, source `/home/runner/ros2_ws/install/setup.sh`.

Flags justified:
- `--privileged` used to allow full access to hardware devices. ToF, IMU, UART would otherwise require a laundry list of `--device` flags and careful permissions setup across dozens of device files.
- `--network=host` required for exposing the CAN interface. Can't `docker run -p` CAN. This sometimes breaks ros when communicating between two containers, so we run everything from the same container with `ros exec`.
- `--cap-add=NET_ADMIN` required for the Lely CANopen library (used as the foundation of the ros2 canopen stack) to initialize the CAN interface without `sudo`. Without this, you must first run the ros2_control stack as sudo (which will initialize the interface but for some reason won't accept any commands), then re-run as a normal user (which will be able to open the interface and accept commands).

# ROS2 nodes

Unless otherwise specified, these commands must be run from inside a ros2 docker container, with the ros2 workspace loaded.

## IMU

```
ros2 launch ardagv just_imu.launch.py
```

If running from a non `--privileged` container (more specifically, if `/sys/bus/iio` is not available to the docker container), replace "local:" with "ip:localhost" inside `ardagv/launch/just_imu.launch.py` and make sure the `iiod` service runs outside the container. systemd unit file coming soon :)

Publishes to `/imu`.

## ToF

```
ros2 launch ardagv just_tof.launch.py
```

Make sure `~/Workspace/media_config_16D_16AB_8C.sh` is run beforehand outside the container. systemd unit file coming soon :)

Publishes to `/cam1/depth_image` (pixel value = millimeters from camera), `/cam1/ab_image` (pixel value = IR image intensity).

## Motor control

Uses ros2_control framework. Currently tested with a [differential drive controller](https://control.ros.org/master/doc/ros2_controllers/diff_drive_controller/doc/userdoc.html). May also run with [forward velocity controller](https://control.ros.org/master/doc/ros2_controllers/forward_command_controller/doc/userdoc.html) by setting the `controller:=forward_velocity_controller` launch configuration.

```
ros2 launch ardagv just_motors.launch.py
```

Make sure the CAN interface is up and running.

Known issues:
- ros2_canopen fails to reliably initialize. Re-launch just_motors, just_crsf until both motors turn when commanded.

Publishes to `/ardagvmotor_robot_description`, and many others. Gets commands via `/diff_drive_controller/cmd_vel` or `/forward_velocity_controller/command` depending on which contrller was selected (diff drive default).

### Alternative motor control without ROS

Here's a basic python example to get you started with spinning the motors without *any* ROS2 usage, just using the `python-canopen` package. Can be run outside of a docker container, if you so wish:

```python
import canopen
from canopen.profiles.p402 import BaseNode402

node = BaseNode402(0x16, 'ardagvmotor.eds')
network = canopen.Network()
network.connect(channel='can0', interface='socketcan', bitrate=500000)
network.add_node(node)

network.sync.start(0.1)

node.nmt.state = 'RESET'
node.nmt.wait_for_bootup(5)
node.load_configuration()
node.setup_402_state_machine()
node.nmt.state = 'OPERATIONAL' # Start CANopen node

def set_speed(v): # v = -1 ... 1
    v = int(v * 4000000) # Convert to internal speed units, +-4e6
    node.rpdo[4]['Controlword'].raw = 0b1111
    node.rpdo[4]['Target velocity'].raw = v
    node.rpdo[4].transmit()

# Power on the motors
node.state = 'OPERATION ENABLED'

set_speed(0.5)
time.sleep(1)
set_speed(-0.5)
time.sleep(1)

# Power off the motors
node.state = 'SWITCH ON DISABLED'
```

## CRSF remote control

```
ros2 launch ardagv just_crsf.launch.py
```

Acts like a multiplexer between RC joystick and `/cmd_vel_auto` topic (navigation stack should write to this).
On RC controller:
- SA = killswitch
- SD = mux select: up = manual, robot controller by right RC stick. down = auto, robot controlled by `/cmd_vel_auto` topic.

Expects ExpressLRS UART on UART3 (/dev/ttymxc3), 420000baud.

# Portenta linux bringup

In case you fry your portenta and need to start over.

## Flash portenta

https://swdownloads.analog.com/cse/arduino_AGV/readme.txt

## Change boot environment

Should only be required once, after flashing, after which it will persist.

1. Connect to UART2, 115200 baud
2. Power on or reboot the portenta.
3. At boot-time ("Hit any key to stop autoboot"), press enter to prevent auto-boot and enter the u-boot console
4. Running `pri` should print out the current environment. There should be a line like:
```
overlays=ov_som_lbee5kl1dx ov_som_x8h7 ov_som_gpu_vpus ov_carrier_breakout_gpio ov_carrier_breakout_sdc ov_carrier_breakout_usbfs ov_carrier_mid_aditof_camera_mipi ov_carrier_mid_adi_imu_spi1
```

Also check that there is a `carrier_custom=1` line.

5. Append the following two overlays: `ov_carrier_breakout_uart1`, `ov_carrier_breakout_uart3`, by running the next command:
```
setenv overlays ov_som_lbee5kl1dx ov_som_x8h7 ov_som_gpu_vpus ov_carrier_breakout_gpio ov_carrier_breakout_sdc ov_carrier_breakout_usbfs ov_carrier_mid_aditof_camera_mipi ov_carrier_mid_adi_imu_spi1 ov_carrier_breakout_uart1 ov_carrier_breakout_uart3
```
(that is: `setenv overlays <list from before> <new overlays>`, no equals sign, only spaces)

6. Run `saveenv` and then `boot`.

## Change hostname

Via UART or SSH (default hostname `portenta-adi`), log in with username `analog`, password `analog`.

Edit `/etc/hostname`, `/etc/hosts` to replace `portenta-adi` with a hostname of your choice. Finally, run `sudo hostname your_new_hostname` and then reboot.

## Change MAC address

All devices flashed in this way get the same ethernet MAC address `7e:7c:b2:88:a3:bb`. If running multiple devices on the same network, this may pose issues with connecting to the right one, DHCP assignment, etc.

How to fix? TODO, the straightforward solution did not work

## Set date and time

If the portenta was not connected to Ethernet at bootup, you may connect it afterwards and set the date and time via HTTP headers:
```
sudo htpdate -s google.com
```

## Install libiio (required for IMU)

```
sudo apt-get install build-essential libxml2 libzstd-dev libxml2-dev bison flex libcdk5-dev cmake
sudo apt-get install libaio-dev libusb-1.0-0-dev libserialport-dev libxml2-dev libavahi-client-dev doxygen graphviz
git clone https://github.com/analogdevicesinc/libiio -b libiio-v0
cd libiio
mkdir build
cd build
cmake .. -DWITH_SYSTEMD=on
make
sudo make install
```

Check that the IMU IIO device works by running `iio_info` and looking for an `adis16470` device. Check that the raw values in its channels make sense (they should have some noise, and, if the board is upright, `accel_z` should be around the order of 50e6):

```
$ iio_info

[...snip...]

IIO context has 3 devices:
        iio:device0: adis16470 (buffer capable)
                8 channels found:

[...snip...]
                        accel_z:  (input, index: 5, format: be:S32/32>>0)
                        3 channel-specific attributes found:
                                attr  0: calibbias value: 0
                                attr  1: raw value: 52408266
                                attr  2: scale value: 0.000000187

[...snip...]
```

Enable and start the iiod service:
```
sudo systemctl enable iiod
sudo systemctl start iiod
```

Now, the libiio context is also exposed over the network, so it will later be accessible from within a Docker container. You may double check by running `iio_info -u ip:localhost`, expecting the same output as before.

## Grow root partition

```
sudo parted /dev/mmcblk2
```

Run `p`. You should get the following partition table:
```
Number  Start   End     Size    Type     File system  Flags
 1      4194kB  161MB   157MB   primary  fat32        lba
 2      161MB   8575MB  8414MB  primary  ext4
```

Resize partition 2 to take up all the remaining space:
```
resizepart 2 100%
```

Exit parted with `q`.

Run `sudo resize2fs /dev/mmcblk2p2`. This should just work.

You may check that the root partition is now larger (15GB) using `df -h`.

## Set up Docker

[Install docker as per the official Debian instructions.](https://docs.docker.com/engine/install/debian/)

Temporary fix for 28.0 iptables issue:
```
sudo apt-get remove docker-ce docker-ce-rootless-extras
sudo apt-get install docker-ce=5:27.5.1-1~debian.12~bookworm docker-ce-rootless-extras=5:27.5.1-1~debian.12~bookworm
```

With this done, you may proceed to the steps at the beginning of the file.
