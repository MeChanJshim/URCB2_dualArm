# URCB2_dualArm

## HARDWARE
* ROBOT: UR10CB2  
* FT SENSOR: AIDIN robotics AFT200-D80

## OS
* UBUNTU 22.04
* ROS2 HUMBLE

## REQUIRED PACKAGE
### OSQP
  > ** Build from source file **  
  > [Requrements]  
  > $ sudo apt update  
  > $ sudo apt install git cmake build-essential  

  > [Download source]  
  > $ cd ~  
  > $ git clone --recursive https://github.com/osqp/osqp.git  
  > $ cd osqp  

  > [Download source]  
  > $ cd ~   
  > $ git clone --recursive https://github.com/osqp/osqp.git  
  > $ cd osqp  

  > [Generate the build directory & Build]  
  > $ mkdir build && cd build  
  > $ cmake -G "Unix Makefiles" -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local ..
  > $ make -j$(nproc) 

  > [Install]  
  > $ sudo make install

  > [Update the library cash]  
  > $ sudo ldconfig  
 
