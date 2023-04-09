# 3D Object Tracking (Sensor Fusion Nanodegree)

By completing all the lessons, now I have a solid understanding of keypoint detectors, descriptors, and methods to match them between successive images. Also, I know how to detect objects in an image using the YOLO deep-learning framework. And finally, I know how to associate regions in a camera image with Lidar points in 3D space. Let's take a look at our program schematic to see what we have already accomplished and what's still missing.

<img src="images/course_code_structure.png" width="779" height="414" />

In this final project, I implemented the missing parts in the schematic. To do this, I completed four major tasks: 
1. First, I developed a way to match 3D objects over time by using keypoint correspondences. 
2. Second, I computed the TTC based on Lidar measurements. 
3. I then proceeded to do the same using the camera, which requires to first associate keypoint matches to regions of interest and then to compute the TTC based on those matches. 
4. And lastly, I conducted various tests with the framework. My goal was to identify the most suitable detector/descriptor combination for TTC estimation and also to search for problems that can lead to faulty measurements by the camera or Lidar sensor. In the last course of this Nanodegree, The Kalman filter, will be used a great way to combine the two independent TTC measurements into an improved version which is much more reliable than a single sensor alone can be.

## Code Guide
### FP.1 Match 3D Objects
Task | Description | Details
--- | --- | ---
FP.1 Match 3D Objects | Implement the method "matchBoundingBoxes", which takes as input both the previous and the current data frames and provides as output the ids of the matched regions of interest (i.e. the boxID property). Matches must be the ones with the highest number of keypoint correspondences. | Code is at camFusion_Student.cpp (lines 351:391)
FP.2 Compute Lidar-based TTC | Compute the time-to-collision in second for all matched 3D objects using only Lidar measurements from the matched bounding boxes between current and previous frame. | computeTTCLidar function is at camFusion_Student.cpp (lines 321:348). There are other helper functions called by computeTTCLidar as well.
FP.3 Associate Keypoint Correspondences with Bounding Boxes | Prepare the TTC computation based on camera measurements by associating keypoint correspondences to the bounding boxes which enclose them. All matches which satisfy this condition must be added to a vector in the respective bounding box. | clusterKptMatchesWithROI function is at camFusion_Student.cpp (lines 195:208)
FP.4 Compute Camera-based TTC | Compute the time-to-collision in second for all matched 3D objects using only keypoint correspondences from the matched bounding boxes between current and previous frame. | computeTTCCamera function is at camFusion_Student.cpp (lines 212:273)
FP.5 Performance Evaluation | Find examples where the TTC estimate of the Lidar sensor does not seem plausible. Describe your observations and provide a sound argumentation why you think this happened. | Pls see the “TTC_Lidar” sheet in the enclosed Excel file “TTCLidar_TTCCamera_Performances.xlsx”



## Demo
![Demo](https://github.com/hedeya1980/Images/raw/main/3D_object_tracking.gif)

## Dependencies for Running Locally
* cmake >= 2.8
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* OpenCV >= 4.1
  * This must be compiled from source using the `-D OPENCV_ENABLE_NONFREE=ON` cmake flag for testing the SIFT and SURF detectors.
  * The OpenCV 4.1.0 source code can be found [here](https://github.com/opencv/opencv/tree/4.1.0)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory in the top level project directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./3D_object_tracking`.
