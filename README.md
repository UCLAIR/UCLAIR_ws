# UCL_AIR_ROS

## Set Up Catkin workspace

We use `catkin build` instead of `catkin_make`. Please install the following:

```
sudo apt-get install python3-wstool python3-rosinstall-generator python3-catkin-lint python3-pip python3-catkin-tools
pip3 install osrf-pycommon
```

Then, initialize the catkin workspace:
```
cd
git clone https://github.com/KhalidAgha20/UCLAIR_ws.git
cd ~/UCLAIR_ws
catkin init
```
## Install ArduPilot

### Clone ArduPilot
In home directory:
```
cd ~
sudo apt install git
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot
```

#Install dependencies
```
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile
```

### Checkout Latest Copter Build
```
git checkout Copter-4.3.2
git submodule update --init --recursive
```

### Run SITL
```
cd ~/ardupilot/ArduCopter
sim_vehicle.py -w
```


## Install `mavros` and `mavlink`

Install `mavros` and `mavlink` from source:
```
cd ~/UCLAIR_ws
wstool init ~/UCLAIR_ws/src

rosinstall_generator --upstream mavros | tee /tmp/mavros.rosinstall
rosinstall_generator mavlink | tee -a /tmp/mavros.rosinstall
wstool merge -t src /tmp/mavros.rosinstall
wstool update -t src
rosdep install --from-paths src --ignore-src --rosdistro `echo $ROS_DISTRO` -y

catkin build
```
Add a line to end of `~/.bashrc` by running the following command:
```
echo "source ~/UCLAIR_ws/devel/setup.bash" >> ~/.bashrc
```

update global variables
```
source ~/.bashrc
```

### Start MAVROS
```
roslaunch mavros apm2.launch fcu_url:=udp://localhost:14550@
```

## Install `pygeodesy`

```
pip3 install pygeodesy
```


