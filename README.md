# Dual-SLAM: A framework for robust single camera navigation

<img src="https://github.com/HuajianUP/Dual_SLAM/blob/master/img/framework.jpg" alt="framework" width=2850>

## Publication:

## If you find this work useful in your research, please consider citing our paper:

    @inproceedings{hhuang2020dualslam,
	  author={Huang, Huajian and Lin, Wen-Yan and Liu, Siying and Zhang, Dong and Yeung, Sai-Kit},
	  booktitle={2020 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)}, 
	  title={Dual-SLAM: A framework for robust single camera navigation}, 
	  year={2020},
	  volume={},
	  number={},
	  pages={4942-4949},
	  doi={10.1109/IROS45743.2020.9341513}
}


# 1. Prerequisites
This implementation is based on [ORB-SLAM](https://github.com/raulmur/ORB_SLAM2), an excellent feature-based monocular SLAM. The requirements are almost as same as ORB-SLAM. We have tested this implementation in **Ubuntu 18.04**.

### C++11 or C++0x Compiler
We use the new thread and chrono functionalities of C++11.

### Pangolin
We use [Pangolin](https://github.com/stevenlovegrove/Pangolin) for visualization and user interface. Dowload and install instructions can be found at: https://github.com/stevenlovegrove/Pangolin.

### OpenCV
We use [OpenCV](http://opencv.org) to manipulate images and features. Dowload and install instructions can be found at: http://opencv.org. **Required at leat 2.4.3. Tested with OpenCV 3.2**.

### Eigen3
Required by g2o (see below). Download and install instructions can be found at: http://eigen.tuxfamily.org. **Required at least 3.1.0**.

### DBoW2 and g2o (Included in Thirdparty folder)
We use modified versions of the [DBoW2](https://github.com/dorian3d/DBoW2) library to perform place recognition and [g2o](https://github.com/RainerKuemmerle/g2o) library to perform non-linear optimizations. Both modified libraries (which are BSD) are included in the *Thirdparty* folder.

### GMS (Included in Thirdparty folder)
[GMS-Feature-Matcher-master](https://github.com/JiawangBian/GMS-Feature-Matcher)

# 2. Building

Clone the repository:
```
git clone https://github.com/HuajianUP/Dual_SLAM.git Dual_SLAM
```

Execute:
```
cd Dual_SLAM
chmod +x build.sh
./build.sh
```

# 3. Examples

## KITTI Dataset  

1. Download the dataset (grayscale images) from http://www.cvlibs.net/datasets/kitti/eval_odometry.php 

2. In general, the more features are extracted, the more stable the system will be, but the processing time per frame will be longer. When extract 2000 features per frame, standard ORB-SLAM still potentially suffers from tracking loss and fail to constructs an intact map on several KITTI sequences (KITTI 00, 01, 02, 08, 09, 12, and 17). You could run the system on these sequences and see the recovery effect.

3. Execute the following command. Change `KITTIX.yaml`by KITTI00-02.yaml, KITTI03.yaml, KITTI04-12 or KITTI13-21.yaml for sequence 0 to 2, 3, 4 to 12, and 13 to 21 respectively. 

4. In yaml files, change `sequenceDir` to your uncompressed sequence folder and `imageDir` to the image folder. Set `recoveryFlag` to `1` to activate recovery, `initByGMS` to `1` to change initialization algorithm.


```
./Examples/Dual-SLAM/dual_kitti Vocabulary/ORBvoc.bin config/KITTIX.yaml
```

## Processing your own sequences
You will need to create a settings file with the calibration of your camera. See the settings file provided for the KITTI datasets for monocular. 



### Reference

[**ORB-SLAM**] Raúl Mur-Artal, J. M. M. Montiel and Juan D. Tardós. ORB-SLAM: A Versatile and Accurate Monocular SLAM System. *IEEE Transactions on Robotics,* vol. 31, no. 5, pp. 1147-1163, 2015.[PDF](http://webdiis.unizar.es/~raulmur/MurMontielTardosTRO15.pdf).


