# projectimageonpointcloud
Small tool to project a 2D image onto a point cloud then either render or save the result. Uses OpenCV and PointCloudLibrary and incorporates CornerDetect by Geiger et al (2012) for image registration.

# CornerDetect
Realization of "Automatic Camera and Range Sensor Calibration using a single Shot" by C++;
This is a key work to detect checkerboard corners and plays a role in camera calibration.
![image](https://github.com/qibao77/cornerDetect/blob/master/cornor_detect.png)
# Reference
Geiger A, Moosmann F, Car Ö, et al. Automatic camera and range sensor calibration using a single shot[C]//Robotics and Automation (ICRA), 2012 IEEE International Conference on. IEEE, 2012: 3936-3943.