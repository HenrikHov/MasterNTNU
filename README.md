This project uses ROS Noetic and runs on Ubuntu 20.04. Python 3 is used for all development. The project requires a setup process involving workspace directories and Catkin, the ROS build system. Due to a non-disclosure agreement (NDA), detailed setup information cannot be shared, but a general overview is provided below.

## Linux OS and ROS Noetic

1. **Setup Ubuntu 20.04**: If using Windows, consider setting up a virtual machine or dual boot to access Linux OS.
2. **Install Dependencies**: Run the following commands:
    - CMake: `sudo apt install cmake`
    - Git: `sudo apt install git`
    - MoveIt!: `sudo apt install ros-noetic-moveit`
    - Build Essentials: `sudo apt install build-essential`
    - Python 3: `sudo apt install python3 python3-pip`

## Workspace Setup

1. **Create Workspace**:
    - Open a terminal and source ROS: `source /opt/ros/noetic/setup.bash`
    - Create a new directory: `mkdir -p ~/mbot_ws/src`
2. **Directory and Source File**:
    - Navigate to the workspace: `cd ~/mbot_ws/src`
    - Initialize Catkin: `catkin_init_workspace`
3. **Initialize Catkin**:
    - Go back to the workspace root: `cd ~/mbot_ws`
    - Build with Catkin: `catkin_make`
4. **GitHub Repositories**:
    - Clone the required repositories if available within the NDA constraints: `git clone <repository_url>`

## Python Virtual Environment and Sourcing

1. **Virtual Environment Setup**:
    - Navigate to the workspace: `cd ~/mbot_ws`
    - Create a virtual environment: `python3 -m venv .venv/mBot`
2. **Activation**:
    - Add this function to `.bashrc` to source both ROS and the Python environment:
    ```bash
    function mBot() {
        source ~/mbot_ws/devel/setup.bash
        source ~/mbot_ws/.venv/mBot/bin/activate
    }
    ```
3. **Execute**: Type `mBot` to activate the combined environment.

## Building with Catkin

1. **Activate Environment**: Make sure the `mBot` environment is active: `mBot`
2. **Build**:
    - Use `catkin build` to compile the ROS packages: `catkin build`
3. **Source**:
    - Run `source devel/setup.bash` to ensure the terminal recognizes the built packages.

## RabbitMQ Server

1. **Install RabbitMQ**: 
    - `sudo apt install rabbitmq-server`
2. **Start RabbitMQ**: 
    - `docker run -it --network=host rabbitmq:3`

## Launch Files

- **proximity_sensors.launch**: `roslaunch <package_name> proximity_sensors.launch`
- **single_ended.launch**: `roslaunch <package_name> single_ended.launch`
- **double_ended.launch**: `roslaunch <package_name> double_ended.launch`

After implementing these steps, initiate the digital twin in the desired launch file to test the home positioning and docking source code.

To test the autonomous movements of the digital twin with proximity_sensors.launch, run:
`rosrun <package_name> autonomous_launcher.py`

For single_ended.launch, run:
`rosrun <package_name> demo_autonomous_launcher.py`
