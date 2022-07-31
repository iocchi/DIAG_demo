# Install

Tested on Linux Ubuntu systems with Docker CLI version 19.03.6 and docker-compose version 1.28.2.

* [docker](http://www.docker.com)

    Install Docker CLI (not Docker Desktop!!!).

    See also 
    [Post-installation steps for Linux](https://docs.docker.com/install/linux/linux-postinstall/).
    In particular, add your user to the `docker` group and log out and in again, before proceeding.

* [docker-compose](https://docs.docker.com/compose/install/)

    You need to install the latest version 1.28, check it with

        docker-compose --version

* For Nvidia drivers, install nvidia-docker2

        https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html



# Setup

* Clone this repository

        git clone https://github.com/iocchi/DIAG_demo.git


* VNC-based xserver (if you want to run it GUI-less or have problems with your graphic card)

    Note: docker image for stage simulator does not run with Nvidia drivers.
    If you have Nvidia drivers on your host OS, use the VNC server.

        docker pull devrt/xserver


* Download Contingent-ff

Download [Contingent-ff](https://fai.cs.uni-saarland.de/hoffmann/cff.html) and copy it in the `bin` folder of this repository


# Run

On host OS, move to `bin` folder of this repository

        cd DIAG_demo/bin


* Start docker containers

    Start using OS display (it may not work with Nvidia cards)

        ./start.bash

    or start with Nvidia drivers

        ./start.bash vnidia

    or start with VNC

        ./start.bash vnc

    and open a browser at `http://localhost:3000` to see the simulator.


* Start a scenario

    Start a scenario script in `DIAG_demo/bin` folder

        ./scenario1.bash

    This script will launch ROS nodes and action proxies

* Run a plan

        ./run_plan.bash [<plan_name>|stop]

    Example for scenario1: 

        ./run_plan.bash DIAG_printer_2

    The robot goes to the printer and checks the light color, if it is red, the robot tells it to the blue person, then it goes back home.

    See other plans in the `plans` folder.


# Scenario configuration

Copy and edit `scenario<n>.bash` and `actions_scenario<n>.bash` 
to customize a scenario configuration. Do not modify the files
in the folder as they are under `git` control and will prevent further updates. 


* Set robot pose

        ./setrobotpose.bash <X> <Y> <th_deg>

    Example

        ./setrobotpose.bash 2.0 2.0 0

    Note: if localizer is already started, make sure to re-localize the robot after this command (for example, by using rviz)


* Set person pose

        ./setpersonpose.bash <name> <X> <Y> <th_deg>

    Example

        ./setpersonpose.bash alice 5.4 2.2 1.57


* Change light status

        ./setlight.bash [red|green]

    Run the plan to see different behaviors according to the light color.



# Stop demo and quit all services

        ./stop.bash


# Development

For development or to run the last versions of code, follow these additional instructions. 

* Download MARRtino apps and other repos in `$HOME/src`

        cd $HOME/src
        git clone --depth 1 https://bitbucket.org/iocchi/marrtino_apps.git
        git clone --depth 1 https://bitbucket.org/iocchi-/stage_environments.git
        git clone --depth 1 https://github.com/Imperoli/gradient_based_navigation.git
        git clone --depth 1 https://github.com/iocchi/stagepersondetection.git
        git clone --depth 1 https://github.com/iocchi/PetriNetPlans


* Set environment variable `MARRTINO_APPS_HOME` 

    Set `MARRTINO_APPS_HOME` to  `marrtino_apps` folder

    In `~/.bashrc`

        export MARRTINO_APPS_HOME=<PATH_TO>/marrtino_apps

* Set environment variable `DEMO_DIR`

    Set demo folder in environment variable `DEMO_DIR`

        export DEMO_DIR=<path to demo>

* If using `.bashrc`, open a new terminal before running the demo
    
    Check the settings

        echo $MARRTINO_APPS_HOME
        echo $DEMO_DIR

* Start all services in development mode

        ./start.bash dev

