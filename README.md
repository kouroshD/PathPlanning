# PathPlanning

This repository contains the following pkgs:

- path planning using rrt* module and the communication module for that. The package is integrated with ROS as service (rrtStar_point, and rrtStar_msgs_point).
 
- path planning using the extended-rrt* module and the communication module for that. The package is integrated with ROS as service (rrtStar_volume, and rrtStar_msgs_volume).
   
 
 - S. Karaman, E. Frazzoli, "Sampling-based algorithms for optimal motion planning", The International Journal of Robotics Research, 30(7), 846–894. [link](https://journals.sagepub.com/doi/abs/10.1177/0278364911406761#articleCitationDownloadContainer)

- My thesis, which explains the advantages of extended-rrt* wrt rrt*.

Extended-rrt* computes a feasible path for an object/robot with a volume in the workspace, while rrt* computes the path for a point in space. For further details, please check the mentioned thesis, or write to me.

This library is distributed in the hope that it will be useful but WITHOUT ANY WARRANTY, including the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. The authors allow the users of the PathPlanning library to use and modify it for their own research. Any commercial application, redistribution, etc... has to be arranged between users and authors individually.

For further license information, please contact the author.

kourosh.darvish@gmail.com


TODO: there are some matlab codes for visualizations the workspace, found path, ... . It is located in stand-alone module. I should be updated later.



