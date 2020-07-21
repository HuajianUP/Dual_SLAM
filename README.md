# Dual-SLAM

## Publication:

## If you find this work useful in your research, please consider citing our paper:

    @article{
     }

# 2. Prerequisites
We have tested the library in **Ubuntu 18.04**.

## C++11 or C++0x Compiler
We use the new thread and chrono functionalities of C++11.

## Pangolin
We use [Pangolin](https://github.com/stevenlovegrove/Pangolin) for visualization and user interface. Dowload and install instructions can be found at: https://github.com/stevenlovegrove/Pangolin.

## OpenCV
We use [OpenCV](http://opencv.org) to manipulate images and features. Dowload and install instructions can be found at: http://opencv.org. **Required at leat 2.4.3. Tested with OpenCV 3.2**.

## Eigen3
Required by g2o (see below). Download and install instructions can be found at: http://eigen.tuxfamily.org. **Required at least 3.1.0**.

## DBoW2 and g2o (Included in Thirdparty folder)
We use modified versions of the [DBoW2](https://github.com/dorian3d/DBoW2) library to perform place recognition and [g2o](https://github.com/RainerKuemmerle/g2o) library to perform non-linear optimizations. Both modified libraries (which are BSD) are included in the *Thirdparty* folder.

## GMS (Included in Thirdparty folder)
[GMS-Feature-Matcher-master](https://github.com/JiawangBian/GMS-Feature-Matcher)

# 3. Building Dual-SLAM library and examples

Clone the repository:
```
git clone https://github.com/ Dual_SLAM
```

Execute:
```
cd Dual_SLAM
chmod +x build.sh
./build.sh
```

This will create **libDual_SLAM.so**  at *lib* folder and the executables **dual_tum_mono**, **dual_kitti**, **dual_mono** in *Examples/Dual-SLAM* folder.

# 4. Examples


## KITTI Dataset  

1. Download the dataset (grayscale images) from http://www.cvlibs.net/datasets/kitti/eval_odometry.php 

2. Execute the following command. Change `KITTIX.yaml`by KITTI00-02.yaml, KITTI03.yaml, KITTI04-12 or KITTI13-21.yaml for sequence 0 to 2, 3, 4 to 12, and 13 to 21 respectively. 

3. In yaml files, change `sequenceDir` to the uncompressed sequence folder and `imageDir` to the image folder.
```
./Examples/Dual-SLAM/dual_kitti Vocabulary/ORBvoc.bin config/KITTIX.yaml
```

## TUM Monocular Dataset

1. Download a sequence from https://vision.in.tum.de/data/datasets/mono-dataset and undistort sequences.

2. Execute the following command. Change `TUM_MonoX.yaml` to TUM_Mono1.yaml or TUM_Mono2.yaml for different sequences respectively. 

3. In yaml files, change `sequenceDir` to the uncompressed sequence folder and `imageDir` to the image folder.

```
./Examples/Dual-SLAM/dual_tum_mono Vocabulary/ORBvoc.bin config/TUM_MonoX.yaml
```

## Processing your own sequences
You will need to create a settings file with the calibration of your camera. See the settings file provided for the TUM and KITTI datasets for monocular. 


# 1. License

ORB-SLAM2 is released under a [GPLv3 license](https://github.com/raulmur/ORB_SLAM2/blob/master/License-gpl.txt). For a list of all code/library dependencies (and associated licenses), please see [Dependencies.md](https://github.com/raulmur/ORB_SLAM2/blob/master/Dependencies.md).


### Reference

[Monocular] Raúl Mur-Artal, J. M. M. Montiel and Juan D. Tardós. **ORB-SLAM: A Versatile and Accurate Monocular SLAM System**. *IEEE Transactions on Robotics,* vol. 31, no. 5, pp. 1147-1163, 2015. (**2015 IEEE Transactions on Robotics Best Paper Award**). **[PDF](http://webdiis.unizar.es/~raulmur/MurMontielTardosTRO15.pdf)**.


