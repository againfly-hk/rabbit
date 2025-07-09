# 树莓派 zero2w 配置顺序：

# 1、更换清华源 https://mirrors.tuna.tsinghua.edu.cn/raspbian
sudo apt update
sudo apt upgrade
sudo apt autoremove

# 2、设置cam raspicam支持和SPI支持
sudo raspi-config

# 3、安装必要的包
sudo apt install cmake git gedit
sudo apt install libopencv-dev
sudo apt install pigpio

# 4、安装raspicam c++ api
git clone https://github.com/cedricve/raspicam.git
cd raspicam
mkdir build
cd build
cmake ..
sudo make install
sudo ldconfig
