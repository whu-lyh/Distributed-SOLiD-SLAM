<div align="center">
  <h1>Distributed SOLiD SLAM</h1>
  <a href="https://github.com/sparolab/solid/tree/master/"><img src="https://img.shields.io/badge/-C++-blue?logo=cplusplus" /></a>
  <a href="https://github.com/sparolab/solid/tree/master/"><img src="https://img.shields.io/badge/-Linux-grey?logo=linux" /></a>
  <a href="https://hub.docker.com/r/cokr6901/solid-a-loam/tags"><img src="https://badges.aleen42.com/src/docker.svg" /></a>
</div>

## What is Distributed SOLiD SLAM?
* Distributed SOLiD SLAM is a Distributed SOLiD-based LiDAR SLAM Framework, which is a modified version of [DiSCo-SLAM](https://github.com/RobustFieldAutonomyLab/DiSCo-SLAM). 

## What has improved in Distributed SOLiD SLAM?
*

## Dependencies
* Ubuntu 20.04
* [GTSAM (Develop version)](https://github.com/borglab/gtsam.git)
* [libnabo 1.0.7](https://github.com/norlab-ulaval/libnabo/tree/1.0.7) 

## How to use the Distributed SOLiD SLAM?
* Linux
 ```
    $ cd ~/catkin_ws/src
    $ git clone https://github.com/sparolab/Distributed-SOLiD-SLAM.git
    $ cd ..
    $ catkin_make
    $ source devel/setup.bash
    $ roslaunch lio_sam run.launch
    $ rosbag play (your dataset).bag
  ```

* Docker
 ```
    $ cd ~/catkin_ws/src
    $ git clone https://github.com/sparolab/Distributed-SOLiD-SLAM.git
    $ docker pull cokr6901/distributed_solid_slam
    $ docker run --privileged --gpus all \
	-it --name distributed_solid_slam --ipc=host --shm-size=512M \
	--device=/dev/video0:/dev/video0 -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
	-e DISPLAY=unix$DISPLAY -v /root/.Xauthority:/root/.Xauthority \
	--env="QT_X11_NO_MITSHM=1" \
	-v ~/catkin_ws/src/:/home/test_ws/src -v (your dataset folder path)/:/home/test_ws/storage cokr6901/distributed_solid_slam:latest
    $ cd /home/test_ws/
    $ catkin_make
    $ source devel/setup.bash
    $ roslaunch lio_sam run.launch
    $ rosbag play (your dataset).bag
  ```



## To DO
* [ ] Change LiDAR-Odometry (LIO-SAM &rightarrow; FAST-LIO2)
* [ ] Add Outlier Rejection (e.g. RANSAC)

## Special Thanks
* We appreciate Prof. [Brendan Englot](https://scholar.google.com/citations?user=Nd6tX_kAAAAJ&hl=ko)'s RobustFieldAutonomyLab, particularly [Yewei Huang](https://scholar.google.com/citations?user=8g3U_tkAAAAJ&hl=ko&oi=sra), for publishing the DiSCo-SLAM.

## Citation
  ```
	@article{kim2024narrowing,
	  title={Narrowing your FOV with SOLiD: Spatially Organized and Lightweight Global Descriptor for FOV-constrained LiDAR Place Recognition},
	  author={Kim, Hogyun and Choi, Jiwon and Sim, Taehu and Kim, Giseop and Cho, Younggun},
	  journal={IEEE Robotics and Automation Letters},
	  year={2024},
	  publisher={IEEE}
	}
  ```
## Contact
* Hogyun Kim (hg.kim@inha.edu)

## License
* For academic usage, the code is released under the BSD 3.0 license. For any commercial purpose, please contact the authors.
