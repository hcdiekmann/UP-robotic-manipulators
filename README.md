# UP-robotic-manipulators
ROS 2 packages for interfacing with the Interbotix X-Series arms mounted on the UP dev kit AMR.

## Installation
Download the installation script and make it executable. 
```bash
curl 'https://raw.githubusercontent.com/hcdiekmann/UP-robotic-manipulators/main/amd64_install.sh' > amd64_install.sh
chmod +x amd64_install.sh
```
Execute the script with optional flags.
```bash
./amd64_install.sh [-h][-d DISTRO][-p PATH]
```
| Flag     | Description |
| ----------- | ----------- |
| -h   | Help message on script usage.                                                                  |
| -d   | ROS 2 distro. If not given, installs the ROS distro compatible withyour Ubuntu version.        |
| -p   | Absolute installation path for the ROS workspace                                               |
