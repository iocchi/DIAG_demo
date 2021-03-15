# Install

Tested only on Linux systems.

* [docker](http://www.docker.com)

    See also 
    [Post-installation steps for Linux](https://docs.docker.com/install/linux/linux-postinstall/).
    In particular, add your user to the `docker` group and log out and in again, before proceeding.

* [docker-compose](https://docs.docker.com/compose/install/)

    You need to install the latest version 1.28, check it with

        docker-compose --version


# Setup

* Setup folder for source files

        mkdir -p $HOME/src

    Note: if you want to change this base folder, you should change something in the docker-compose files.


* This repository

    Clone this repository in `$HOME/src`

        cd $HOME/src
        git clone https://github.com/iocchi/DIAG_demo.git

* Stage simulator

        docker pull iocchi/stage_environments

    Note: this docker image does not work on OS systems wuth Nvidia graphic cards and Nvidia drivers!

* Petri Net Plans

        docker pull iocchi/pnp

* MARRtino apps

        cd $HOME/src
        git clone --depth 1 https://bitbucket.org/iocchi/marrtino_apps.git
        cd marrtino_apps/docker
        docker build -t marrtino:base -f Dockerfile.base .
        docker build -t marrtino:navigation -f Dockerfile.navigation .

* MARRTINO_APPS_HOME environment variable

    Set `MARRTINO_APPS_HOME` to  `marrtino_apps` folder

    In `~/.bashrc`

        export MARRTINO_APPS_HOME=$HOME/src/marrtino_apps


* ROS_IP environment variable

    Set `ROS_IP` to localhost `127.0.0.1`

    In `~/.bashrc`

        export ROS_IP=127.0.0.1

* Open a new terminal before running the demo
    
    Check the settings

        echo $ROS_IP
        echo $MARRTINO_APPS_HOME

# Run

On host OS, move to `bin` folder of this repository

    cd $HOME/src/DIAG_demo/bin


* Start all services

        ./start.bash


* Test actions

        ./test_actions.bash


* Test plan

        ./test_plan.bash

    The robot goes to the printer and checks the light color, if it is red,
    the robot tells it to the blue person, then it goes back home.


* Change light status

        ./setlight.bash [red|green]

    Run the plan to see different behaviors according to the light color.

* Stop all services

        ./stop.bash

