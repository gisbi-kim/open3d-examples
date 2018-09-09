freiburgCampus360_3D


CONTENT
-------
This dataset contains 77 3D scans. For each scan i there are 3 files with the following informations:

Scan_i_robotPoses.dat contains the positions of the robot according to its wheel odometry (first line) and graph-SLAM (second line). The poses are given as 3D position and Euler angles in the order x, y, z, roll, pitch, yaw.

Scan_i_points.dat contains the measured 3D points in the robot coordinate system and the position of the sensor from which this point was measured. The order is sensorPosX, sensorPosY, sensorPosZ, pointX, pointY, pointZ.

Scan_i_farRanges.dat contains the measurements of the laser that returned far ranges. First, the position of the sensor is given and then a point on the line of sight. I.e., the laser beam startet in the first point and went through the second one. The order is sensorPosX, sensorPosY, sensorPosZ, pointX, pointY, pointZ.

The coordinates are in a right handed coordinate system. x to the front, y to the left and z to the top. All the angles are given in radians.


GENERAL INFORMATION ABOUT THE DATASET
-------------------------------------
The dataset was acquired on the campus of the Faculty of Engineering, University of Freiburg, Germany (79110 Freiburg, Georges-Koehler-Allee). It was captured using a wheeled
robot equipped with a SICK LMS laser range scanner mounted on a
pan-tilt unit. The pan-tilt unit was moved to acquire a 360 degree view of the surrounding. The scans were obtained in a stop-and-go fashion and it took our platform about 25s to capture a scan. The dataset contains 77 3D scans captured along a 723\,m long trajectory. The average distance between the captured scans was about 10m. Each scan consists of
150,000-200,000 points.


LICENCE
-------
This dataset is made available under the Creative Commons Attribution Licence 3.0: the licensor permits others to copy, distribute, display, and perform the work. In return, licensees must give the original author credit.
See http://creativecommons.org/licenses/by/3.0
By way of attribution, please include an acknowledgement and/or citation for any results published using this dataset.


CONTACT INFORMATION
-------------------
Bastian Steder
steder@informatik.uni-freiburg.de
University of Freiburg
Dept. of Computer Science
Autonomous Intelligent Systems
Georges-Koehler-Allee 079
D-79110 Freiburg
Germany
