# URCB2_dualArm

### HARDWARE
* ROBOT: UR10CB2  
* FT SENSOR: AIDIN robotics AFT200-D80

### OS
* UBUNTU 22.04
* ROS2 HUMBLE

### REQUIRED PACKAGE
* OSQP
  > 1. Build from source file
  > [requrements]
  > sudo apt update
  > sudo apt install git cmake build-essential

# OSQP 소스 다운로드
cd ~
git clone --recursive https://github.com/osqp/osqp.git
cd osqp

# 빌드 디렉토리 생성 및 빌드
mkdir build && cd build
cmake -G "Unix Makefiles" -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local ..
make -j$(nproc)

# 설치
sudo make install

# 라이브러리 캐시 업데이트
sudo ldconfig    
