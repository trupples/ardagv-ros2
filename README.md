# Arduino AGV platform

# Portenta linux bringup

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

Create a docker container based on `shooteu/portenta-ros2` and the following flags:  

```
docker run -it --network=host shooteu/portenta-ros2
```

Note: parameters will soon change to also support ToF.

Inside the container, enter the ros2 workspace, and load it:  

```
cd ros2_ws
source install/setup.sh
```

## Have fun with the robot

### ToF node

Outside the container, run `/home/analog/Workspace/media_config_16D_16AB_8C.sh` to initialize the ToF camera. This is required once per reboot.

Inside the container, with the ros2 workspace loaded:

```
ros2 launch adi_3dtof_adtf31xx adi_3dtof_adtf31xx_launch.py
```

## IMU node

```
ros2 run imu_ros2 imu_ros2_node --ros-args -p iio_context_string:="ip:localhost"
```

### Motor control node

```
ros2 launch canopen_core canopen.launch.py bus_config:=/home/analog/ros2_ws/install/ardagvmotor/share/ardagvmotor/config/bus.yml master_config:=/home/analog/ros2_ws/install/ardagvmotor/share/ardagvmotor/config/master.dcf can_interface_name:=can0 node_id:=1
```

Initialize motor:  
```
ros2 service call right_wheel_joint/init std_srvs/Trigger '{}'
ros2 service call right_wheel_joint/velocity_mode std_srvs/Trigger '{}'
```

Control motor with a speed between -4000 and 4000 (corresponding to an approx 180 RPM maximum in either direction):
```
ros2 service call right_wheel_joint/target canopen_interfaces/srv/COTargetDouble '{"target": 500}'
```
