# Introduction

It is only necessary to install libraries for those cameras, robots, or input devices that you intend to use.


# Cameras

### Azure Kinect (Kinect 4)
- On Ubuntu 18 install azure kinect SDK with apt
- On Ubuntu 20 download libk4a*(-dev) and libk4abt*(-dev) from https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/libk/
  and k4atools from https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/k/k4a-tools \
  Install with `sudo dpkg -i`

- Install Open3D in your Python env with `pip install open3d`

- For default usage, start `$ python robot_io/cams/kinect4/kinect4.py`

### RealSense SR300/SR305

First follow installation instructions for librealsense2 [here](https://github.com/IntelRealSense/librealsense)
```
pip install pyrealsense2
python robot_io/cams/realsense/realsenseSR300_librs2.py  # to test
```

### Framos D435e
```
groups | grep video  # user must be in video group, otherwise ask Michael K.
file /usr/src/librealsense2  # (see below a)
diff misc/framos_setup_files/setup.py /usr/src/librealsense2/setup.py
cp -r /usr/src/librealsense2 .
cd librealsense2
pip uninstall pyrealsense2
pip install -e .

cd ../robot_io/cams/realsense
python realsense.py  # test script
```
- a) If `/usr/src/librealsense2` does not exist, download FRAMOS software package from https://www.framos.com/en/industrial-depth-cameras#downloads. Follow installation instructions, make sure to use local admin user (e.g. xam2) to install (file system may NOT be network mounted).
- b) Use Ethernet sockets on the ceiling for PoE. 


### Marker Detector
Make sure OpenCV and Eigen are installed:
```
sudo apt install libopencv-dev python3-opencv
sudo apt install libeigen3-dev
pip install Cython
cd robot_io/marker_detection/apriltag_detection
python setupBatch.py build_ext --inplace
```


# Robots

## KUKA iiwa

Clone Kuka Java Repository on aisgit
```git clone https://aisgit.informatik.uni-freiburg.de/hermannl/kuka_java_interface_no_ros```

`robot_io/robot_interface/iiwa_interface.py` is a ROS free python controller for KUKA iiwa. It sends UDP messages to the iiwa_java_controller.

Supported control modes:
- Joint position control
    - degrees / radians
- Cartesian position control
    - PTP / LIN motions
    - with / without impedance
    - absolute / relative coordinates

For more information please read README of [kuka_java_interface_no_ros](https://aisgit.informatik.uni-freiburg.de/hermannl/kuka_java_interface_no_ros)

## Franka Emika Panda

### IK fast
IK fast is an analytic IK solver. In order to use IK fast, first install `ikfast-pybind`:
```
git clone --recursive https://github.com/yijiangh/ikfast_pybind
cd ikfast_pybind
# copy panda IK solution .cpp and .h to ikfast_pybind
cp ../robot_io/misc/ik_fast_files/ikfast.h ./src/franka_panda/
cp ../robot_io/misc/ik_fast_files/ikfast0x10000049.Transform6D.0_1_2_3_4_5_f6.cpp ./src/franka_panda/
pip install .
```
For creating different IK solutions (e.g. in case of a different gripper) please refer to: 
`http://docs.ros.org/en/kinetic/api/framefab_irb6600_support/html/doc/ikfast_tutorial.html`

### Frankx
```
git clone git@github.com:lukashermann/frankx
cd frankx
git clone git@github.com:pantor/affx
git clone git@github.com:pantor/ruckig
cd affx; git checkout -b frankx_version dabe0ba; cd ..
cd ruckig; git checkout -b frankx_version 31f50f0; cd ..

conda install pybind11
vim setupy.py  # add "-DFranka_DIR=/opt/ros/noetic/share/franka/cmake/"
pip install -e .
firefox https://192.168.180.87/desk/  # unlock joints
export LD_LIBRARY_PATH=/opt/ros/noetic/lib/:$LD_LIBRARY_PATH

cd robot_io/robot_interface
python panda_frankx_interface.py  # test robot
```

# Input Devices

## SpaceMouse
```
sudo apt install libspnav-dev spacenavd
conda activate robot
pip install spnav
```

Next test if it works, some common pitfalls are:
1. Turn on SpaceMouse in the back
2. May not work while charging.
3. Wireless range is quite limited.
4. Comment the following two lines in `site-packages/spnav/__init__.py`
```
#pythonapi.PyCObject_AsVoidPtr.restype = c_void_p
#pythonapi.PyCObject_AsVoidPtr.argtypes = [py_object]
```

To test execute the following program. When moving the mouse you should
see numbers scrolling by.
```
python robot_io/input_devices/space_mouse.py
```


## VR Teleoperation

Please see (here.)[docs/teleoperation.md]

# Sensors

## Digit Sensor
This is for installing the Facebook Digit Sensor
https://digit.ml/.

```
pip install digit-interface
```
