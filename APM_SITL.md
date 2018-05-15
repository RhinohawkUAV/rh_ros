before you begin: Follow these steps to install ROS.
30 GB recomended for you vm

https://www.evernote.com/shard/s201/sh/fd17ab52-69a0-4390-8c36-259788e89891/2b22344beb9a4935


Instructions to install Ardupilot, and complete the SITL

reference to Ardupilot: http://ardupilot.org/dev/docs/building-the-code.html

mavproxy install instructions: https://ardupilot.github.io/MAVProxy/html/getting_started/download_and_installation.html#linux

clone ardupilot and init submodules

    git clone https://github.com/ArduPilot/ardupilot
    cd ardupilot
    git submodule update --init --recursive
        

Install all prerequisites for ardupilot this should include the arm-gcc.

    Tools/scripts/install-prereqs-ubuntu.sh -y
    
Reload the bash profile
    
    . ~/.profile

Make user a member of dialout: this is done in similer autopilots
    
    sudo usermod -a -G dialout $USER
    
Unpack a specific version of gcc arm required by ardupilot

    tar -xjvf gcc-arm-none-eabi-4_9-2015q3-20150921-linux.tar.bz2
    
If there have been updates to some git submodules you may need to do a full clean build. To do that use:
This also is important if modifying parts of the firmware.

    make px4-clean
    
SITL Setup:

export these paths so other applications can find ardupilot.

    export PATH=$PATH:$HOME/ardupilot/Tools/autotest
    export PATH=/usr/lib/ccache:$PATH

reload bashrc:
    
    . ~/.bashrc
    
Start the generic SITL for arducoptor:

    cd ardupilot/ArduCopter
    
For first run, this command will init all setings
    
    sim_vehicle.py -w
    
use this command to load the simulator with the map.

    sim_vehicle.py --console --map
note: The google map does not work, choose a different map provider in the gui.



load the example waypoint mission.

    wp load ../Tools/autotest/copter_mission.txt
    
commands to start the run in the ardupilot terminal. 

        help : yields a more complete list of commands.
        
        arm throttle   
        mode auto
        rc 3 1500
        

additional INFO (WIP)
* Information on ros with SITL. http://ardupilot.org/dev/docs/ros-sitl.html
               I got this running but did not go any further
* Simulator: gazebo http://ardupilot.org/dev/docs/using-gazebo-simulator-with-sitl.html
                This would not respond for me.
*real flight:  http://ardupilot.org/dev/docs/sitl-with-realflight.html I have this running.




More to come soon



    