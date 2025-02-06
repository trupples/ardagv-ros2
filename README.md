# Arduino AGV platform

Connect to the Portenta X8 using hostname `portenta-arduino`, username `analog`, password `analog`.

All relevant ros2 nodes are built inside the [shooteu/ros2-built-all](https://hub.docker.com/r/shooteu/ros2-built-all) docker image. Should be run with `--privileged` and `--network=host`:

```
docker run -it --privileged --network=host shooteu/ros2-built-all
```

Inside the docker container, source `ros2_ws/install/setup.sh`.

# ROS2 nodes

Unless otherwise specified, these commands must be run from inside a ros2 docker container, with the ros2 workspace loaded.

## IMU

```
ros2 run imu_ros2 imu_ros2_node --ros-args -p iio_context_string:="ip:localhost"
```

If that doesn't work, run `sudo systemctl restart iiod` outside of the container.

## Motor control

Uses ros2_control framework. Currently tested with a [forward velocity controller](https://control.ros.org/master/doc/ros2_controllers/forward_command_controller/doc/userdoc.html), and a [differential drive controller](https://control.ros.org/master/doc/ros2_controllers/diff_drive_controller/doc/userdoc.html).

### ros2_control differential drive

Differential drive inverse kinematics.

```
ros2 launch ardagvmotor diffdrive.launch.py
```

CAREFUL, running this should make your robot move:
```
ros2 topic pub /diff_drive_controller/cmd_vel geometry_msgs/msg/TwistStamped '{"header":{"stamp":{"sec":0,"nanosec":0},"frame_id":"base_link"},"twist":{
        "linear":{"x":1.0,"y":0.0,"z":0.0},
        "angular":{"x":0.0,"y":0.0,"z":0.0}
}}'
```

### ros2_control forward velocity control

No inverse kinematics, just set the velocity and go. Values of -4000 ... 4000 roughly correspond to -180 ... 180 RPM. Values outside of this interval are invalid.

```
ros2 launch ardagvmotor forward_velocity_control.launch.py
```

CAREFUL, running this should make your robot move:
```
ros2 topic pub /forward_velocity_controller/commands std_msgs/msg/Float64MultiArray '{data: [1000, 1000]}'
```


### Direct access to the CANopen drives

If you want to bypass ros2_control, you may start the "raw" CiA 402 nodes using:

```
ros2 launch canopen_core canopen.launch.py bus_config:=/home/runner/ros2_ws/install/ardagvmotor/share/ardagvmotor/config/bus.yml master_config:=/home/runner/ros2_ws/install/ardagvmotor/share/ardagvmotor/config/master.dcf can_interface_name:=can0 node_id:=1
```

Initialize the motor:  
```
ros2 service call motor_right/init std_srvs/Trigger '{}'
ros2 service call motor_right/velocity_mode std_srvs/Trigger '{}'
```

Control motor with a speed between -4000 and 4000 (corresponding to an approx 180 RPM maximum in either direction):
```
ros2 service call motor_right/target canopen_interfaces/srv/COTargetDouble '{"target": 500}'
```

### Dislike ROS?

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

## Set up Docker with storage on the SD card

Get an SD card and move any data out of it (it will be erased).
Insert SD card into carrier.
The OS will have automatically mounted it. Unmount with `sudo umount /dev/mmcblk1p*`
Partition it using `parted` or `fdisk` (one big partition).
Create an ext2 filesystem using `sudo mkfs.ext2 /dev/mmcblk1p1`. FAT will **not** work, because we intend to use this partition for Docker's internal storage, and it requires ext2-like permissions.

Set up fstab to automatically mount the SD card at `/mnt/sdcard`:
```
sudo mkdir -p /mnt/sdcard
echo '/dev/mmcblk1p1 /mnt/sdcard auto defaults 1 2' | sudo tee -a /etc/fstab
sudo systemctl daemon-reload
sudo mount /mnt/sdcard
```

[Install docker as per the official Debian instructions.](https://docs.docker.com/engine/install/debian/)
Run `sudo update-alternatives --set iptables /usr/sbin/iptables-legacy` . This is supposedly done when installing docker, but we have found it to be unreliable and to need to be run explicitly just to be sure.

```
sudo usermod -aG docker analog
```

Stop docker and remove the contents of `/var/lib/docker` (to be shortly moved to the SD card):
```
sudo systemctl stop docker
sudo rm -rf /var/lib/docker
```

Create a folder for docker files on the SD card (not Dockerfiles :P) and link it:
```
sudo mkdir -p /mnt/sdcard/docker-files
sudo ln -s /mnt/sdcard/docker-files /var/lib/docker
```

Start docker back up. There should be no errors.
```
sudo systemctl start docker
```

## Bring up docker container

Create a docker container based on `shooteu/portenta-ros2` and the following flags:  

```
docker run -it --network=host shooteu/portenta-ros2
```

Note: parameters will soon change to also support ToF.

Inside the container, enter the ros2 workspace, and load it:  

```
cd ros2_ws
source install/setup.sh
```
