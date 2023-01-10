# Data Processor

Maintainer: Myung-Hwan Jeon (myunghwan.jeon.offl@gmail.com)

## 1. Download repo and Build workspace

```
$ git clone git@github.com:MyungHwanJeon/data_processor.git
$ cd path/to/data_processor
$ bash catkin build
```

## 2. Run file player

```
$ source devel/setup.bash
$ roslaunch player player.launch
```

## 3. Download example data

1. [seq1.zip](https://drive.google.com/file/d/1xjqbbVBi_ywKNBaiLADE-RErApyTHISv/view?usp=sharing). 
2. unzip seq1.zip

## 4. Load data files and play

1. Click 'Load' button.
2. Choose data folder including sensor_data folder.
3. The player button starts publishing data in the ROS message.

## 5. Contributor
* Jinyong Jeong : The original author
