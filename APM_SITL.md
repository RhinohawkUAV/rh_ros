before you begin: Follow these steps to install ROS.

https://www.evernote.com/shard/s201/sh/fd17ab52-69a0-4390-8c36-259788e89891/2b22344beb9a4935


Instructions to install Ardupilot, and complete the SITL

reference to Ardupilot: http://ardupilot.org/dev/docs/building-the-code.html

clone ardupilot and init submodules

    git clone https://github.com/ArduPilot/ardupilot
    cd ardupilot
    git submodule update --init --recursive
        

Install all prerequisites for ardupilot

    Tools/scripts/install-prereqs-ubuntu.sh -y
    
reload the bash profile
    
    . ~/.profile

make user a member of dialout: this is done in similer autopilots
    
    sudo usermod -a -G dialout $USER
    
unpack a specific version of gcc arm required by ardupilot

    tar -xjvf gcc-arm-none-eabi-4_9-2015q3-20150921-linux.tar.bz2

    