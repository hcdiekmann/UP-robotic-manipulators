#!/usr/bin/env bash

# USAGE: ./amd64_install.sh [-h][-d DISTRO][-p PATH][-n]
#
# Install the UP Interbotix X-Series Arms packages and their dependencies.

OFF='\033[0m'
RED='\033[0;31m'
GRN='\033[0;32m'
BLU='\033[0;34m'

BOLD=$(tput bold)
NORM=$(tput sgr0)

ERR="${RED}${BOLD}"
RRE="${NORM}${OFF}"

PROMPT="> "

ROS2_VALID_DISTROS=('galactic' 'humble' 'rolling')
FOCAL_VALID_DISTROS=('galactic')
JAMMY_VALID_DISTROS=('humble' 'rolling')

NONINTERACTIVE=false
DISTRO_SET_FROM_CL=false
INSTALL_PATH=~/interbotix_ws

_usage="${BOLD}USAGE: ./amd64_install.sh [-h][-d DISTRO][-p PATH][-n]${NORM}

Install the Interbotix X-Series Arms packages and their dependencies.

Options:

  -h              Display this help message and quit

  -d DISTRO       Install the DISTRO ROS distro compatible with your Ubuntu version. See
                  'https://github.com/Interbotix/.github/blob/main/SECURITY.md' for the list of
                  supported distributions. If not given, installs the ROS distro compatible with
                  your Ubuntu version.

  -p PATH         Sets the absolute install location for the Interbotix workspace. If not specified,
                  the Interbotix workspace directory will default to '~/interbotix_ws'.

  -n              Install all packages and dependencies without prompting. This is useful if
                  you're running this script in a non-interactive terminal like when building a
                  Docker image."


function help() {
  # print usage
  cat << EOF
$_usage
EOF
}

# https://stackoverflow.com/a/8574392/16179107
function contains_element () {
  # check if an element is in an array
  local e match="$1"
  shift
  for e; do [[ "$e" == "$match" ]] && return 0; done
  return 1
}

function failed() {
  # Log error and quit with a failed exit code
  echo -e "${ERR}[ERROR] $@${RRE}"
  echo -e "${ERR}[ERROR] Interbotix Installation Failed!${RRE}"
  exit 1
}

function validate_distro() {
  if contains_element $ROS_DISTRO_TO_INSTALL "${ROS2_VALID_DISTROS[@]}"; then
    ROS_DISTRO_TO_INSTALL=$ROS_DISTRO_TO_INSTALL
    echo -e "${GRN}${BOLD}Chosen Version: ROS 2 $ROS_DISTRO_TO_INSTALL${NORM}${OFF}"
  else
    failed "The chosen ROS 2 distribution '$ROS_DISTRO_TO_INSTALL' is not supported."
  fi
  return 0
}

function check_ubuntu_version() {
 # check if the chosen distribution is compatible with the Ubuntu version
  case $UBUNTU_VERSION in

    20.04 )
      if contains_element $ROS_DISTRO_TO_INSTALL "${FOCAL_VALID_DISTROS[@]}"; then
        echo -e "${GRN}Ubuntu version check complete. ${OFF}"
      else
        failed "Chosen ROS distribution '$ROS_DISTRO_TO_INSTALL' is not supported on Ubuntu ${UBUNTU_VERSION}."
      fi
      ;;

    22.04 )
      if contains_element $ROS_DISTRO_TO_INSTALL "${JAMMY_VALID_DISTROS[@]}"; then
        echo -e "${GRN}Ubuntu version check complete. ${OFF}"
      else
        failed "Chosen ROS distribution '$ROS_DISTRO_TO_INSTALL' is not supported on Ubuntu ${UBUNTU_VERSION}."
      fi
      ;;

    *)
      failed "Something went wrong."
      ;;

  esac
}

function install_essential_packages() {
  # Install necessary core packages
  sudo apt -y install openssh-server curl
  sudo apt -y install python3-pip
  sudo pip3 install transforms3d
  python3 -m pip install modern_robotics
}

function install_ros2() {
  # Install ROS 2
  if [ $(dpkg-query -W -f='${Status}' ros-$ROS_DISTRO_TO_INSTALL-desktop 2>/dev/null | grep -c "ok installed") -eq 0 ]; then
    echo -e "${GRN}Installing ROS 2 $ROS_DISTRO_TO_INSTALL desktop...${OFF}"
    sudo apt install -y software-properties-common
    sudo add-apt-repository universe
    sudo apt install -y curl gnupg lsb-release
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    sudo apt update
    sudo apt install -y ros-$ROS_DISTRO_TO_INSTALL-desktop
    if [ -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
      sudo rm /etc/ros/rosdep/sources.list.d/20-default.list
    fi
    echo "source /opt/ros/$ROS_DISTRO_TO_INSTALL/setup.bash" >> ~/.bashrc
    sudo apt -y install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential python3-colcon-common-extensions
    sudo rosdep init
    rosdep update --include-eol-distros
  else
    echo "ros-$ROS_DISTRO_TO_INSTALL-desktop-full is already installed!"
  fi
  source /opt/ros/$ROS_DISTRO_TO_INSTALL/setup.bash

  # Install AprilTag unoffical ROS 2 port from Interbotix
  APRILTAG_WS=~/apriltag_ws
  if [ ! -d "$APRILTAG_WS/src" ]; then
    echo -e "${GRN}Installing Apriltag ROS Wrapper...${OFF}"
    mkdir -p $APRILTAG_WS/src
    cd $APRILTAG_WS/src
    git clone https://github.com/Interbotix/apriltag_ros.git -b ros2-port
    cd $APRILTAG_WS
    rosdep install --from-paths src --ignore-src -r -y
    colcon build --cmake-args -DCMAKE_CXX_FLAGS="-w" # warnings as errors unrelated to ROS - disables this behavior
    if [ $? -eq 0 ]; then
      echo -e "${GRN}${BOLD}Apriltag ROS Wrapper built successfully!${NORM}${OFF}"
    else
      failed "Failed to build Apriltag ROS Wrapper."
    fi
    echo "source $APRILTAG_WS/install/setup.bash" >> ~/.bashrc
  else
    echo "Apriltag ROS Wrapper already installed!"
  fi
  source $APRILTAG_WS/install/setup.bash
  
  # Install Arm packages
  if [ ! -d "$INSTALL_PATH/src" ]; then
    echo -e "${GRN}Installing ROS 2 packages for the Interbotix Arm...${OFF}"
    mkdir -p $INSTALL_PATH/src
    cd $INSTALL_PATH/src
    git clone https://github.com/hcdiekmann/UP-robotic-manipulators.git -b $ROS_DISTRO_TO_INSTALL
    # TODO remove below when moveit_visual_tools is available in apt repo
    #git clone https://github.com/ros-planning/moveit_visual_tools.git -b ros2
    cd interbotix_ros_core/interbotix_ros_xseries/interbotix_xs_sdk
    sudo cp 99-interbotix-udev.rules /etc/udev/rules.d/
    sudo udevadm control --reload-rules && sudo udevadm trigger
    cd $INSTALL_PATH
    rosdep install --from-paths src --ignore-src -r -y
    colcon build
    if [ $? -eq 0 ]; then
      echo -e "${GRN}${BOLD}Interbotix Arm ROS 2 Packages built successfully!${NORM}${OFF}"
    else
      failed "Failed to build Interbotix Arm ROS 2 Packages."
    fi
    echo "source $INSTALL_PATH/install/setup.bash" >> ~/.bashrc
  else
    echo "UP Interbotix Arm ROS packages already installed!"
  fi
  source $INSTALL_PATH/install/setup.bash
}

function setup_env_vars() {
  # Setup Environment Variables
  if [ -z "$ROS_IP" ]; then
    echo -e "${GRN}Setting up Environment Variables...${OFF}"
    echo "# AAEON Interbotix Configurations" >> ~/.bashrc
    echo 'export ROS_IP=$(echo `hostname -I | cut -d" " -f1`)' >> ~/.bashrc
    echo -e 'if [ -z "$ROS_IP" ]; then\n\texport ROS_IP=127.0.0.1\nfi' >> ~/.bashrc
  else
    echo "Environment variables already set!"
  fi
}

# parse command line arguments
while getopts 'hnd:p:' OPTION;
do
  case "$OPTION" in
    h) help && exit 0;;
    n) NONINTERACTIVE=true;;
    d) ROS_DISTRO_TO_INSTALL="$OPTARG" && DISTRO_SET_FROM_CL=true && validate_distro;;
    p) INSTALL_PATH="$OPTARG";;
    *) echo "Unknown argument $OPTION" && help && exit 0;;
  esac
done
shift "$(($OPTIND -1))"

if ! command -v lsb_release &> /dev/null; then
  sudo apt update
  sudo apt-get install -y lsb-release
fi

UBUNTU_VERSION="$(lsb_release -rs)"

# set default ROS distro before reading clargs
if [ "$DISTRO_SET_FROM_CL" = false ]; then
  elif [ $UBUNTU_VERSION == "20.04" ]; then
    ROS_DISTRO_TO_INSTALL="galactic"
  elif [ $UBUNTU_VERSION == "22.04" ]; then
    ROS_DISTRO_TO_INSTALL="humble"
  else
    echo -e "${BOLD}${RED}Unsupported Ubuntu verison: $UBUNTU_VERSION.${NORM}${OFF}"
    failed "UP Interbotix Arms only work with Ubuntu 20.04 Focal or 22.04 Jammy."
  fi
fi

check_ubuntu_version

echo -e "\n\n"
echo -e "${GRN}${BOLD}**********************************************${NORM}${OFF}"
echo ""
echo -e "${GRN}${BOLD}            Starting Installation!            ${NORM}${OFF}"
echo ""
echo -e "${GRN}${BOLD}**********************************************${NORM}${OFF}"
echo -e "\n\n"

sleep 4
start_time="$(date -u +%s)"

echo -e "\n# AAEON Interbotix Configurations" >> ~/.bashrc

# Update the system
sudo apt update && sudo apt -y upgrade
sudo apt -y autoremove

install_essential_packages
install_ros2
setup_env_vars

end_time="$(date -u +%s)"
elapsed="$(($end_time-$start_time))"

echo -e "${GRN}Installation complete, took $elapsed seconds in total.${OFF}"
echo -e "${GRN}NOTE: Remember to reboot the UP board before using the robot arm!${OFF}"
