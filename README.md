

The visual SLAM for equirectangular cameras (e.g. RICOH THETA series, insta360 series, etc) 



The solution is based on [Stella-SLAM](https://github.com/stella-cv/stella_vslam), which is a community fork of [OpenVslam](https://github.com/xdspacelab/openvslam).


**Requirement**
1. [Stella-SLAM](https://github.com/stella-cv/stella_vslam)
2. [Pangolin](https://github.com/stevenlovegrove/Pangolin) viewer.
3. Linear algebra library [Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page)
4. Graph optimization library [g2o](https://github.com/RainerKuemmerle/g2o)
5. Visual Vocabulary,Fast Bag of Words [FBoW](https://github.com/stella-cv/FBoW)
6. [OpenCV](https://opencv.org)



**Results**//
Results are shown in the folder [results](https://github.com/yazanmaalla/panorama_visual_slam/tree/main/results) ://
1. Trajectory of the camera in the TUM form:
        tx,ty,tz,qx,qy,qz,qw
2. Point cloud of the map in PLY form.
