## Project: Kinematics Pick & Place
### Writeup Template: You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.

---
Stephen D. Williams sdw@lig.net stephendwilliams@gmail.complete

**Steps to complete the project:**  


1. Set up your ROS Workspace.
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code. 


[//]: # (Image References)

[image1]: ./misc_images/misc1.png
[image2]: ./misc_images/misc3.png
[image3]: ./misc_images/misc2.png

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  


### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

Created a series of refinements of kinematic analysis of the kr210 joint
specifications, parameters, etc. Iterated to get positions to match working
robot, determining what was missing and how different parameters affected the
system. Needed to thoroughly examine sequence, sample after each step, and look
at all examples for sample code and missing steps.

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

Created this DH parameters table, initially from examining the URDF file
although several mistakes came out of that pass. While debugging, refined and
validated fields one joint at a time.

# Create Modified DH parameters
s = {alpha0: 0,      a0: 0,     d1: 0.75,    q1: q1,
     alpha1: -pi/2,  a1: 0.35,  d2: 0,       q2: q2-pi/2,
     alpha2: 0,      a2: 1.25,  d3: 0,       q3: q3,
     alpha3: -pi/2,  a3: -0.054,d4: 1.5,     q4: q4,
     alpha4:  pi/2,  a4: 0,     d5: 0,       q5: q5,
     alpha5: -pi/2,  a5: 0,     d6: 0,       q6: q6,
     alpha6: 0,      a6: 0,     d7: 0.303, q7: 0
}

Initially created hand edited sequence of homogeneous transforms, but switched
to the posted TF_Matrix() function found when debugging and reviewing for better code.

Computation was very slow. Moved all of the symbolic setup and simplification to
static & global space in IK_debug and IK_server so that much less is done on
each loop and request.

The last bug was that while the end location showed little error, there was too
much other error in joint positions and effector orientation. Found that the
correcting rotation sequence was x * y * z rather than z * y * x.

#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

As suggested, computed first joint angle by directly pointing to desired angle.
The next two angles where computed based on the right triangle atan2 & via
hypotenuse. Provided feedback that the diagram showing the use of the right
triangle is misleading.

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 

The resulting computation is accurate, but not efficient as it produces a
sequence of solutions. A better computation method would reduce extra movement
and more directly avoid limitations.

Producing the FK & IK was suprisingly difficult to get right and difficult to
debug. While the test rig was very helpful, it would be even better to visualize
both the joint relationships and how computation is or is not compatible and
accurate.  This would help determine which terms at which stage are incorrect.

Need to have a better, faster, easier way to get the appropriate equations
together for a particular robotic configuration.

The symbolic computation in Python is cool, but it becomes very slow quickly.
Need a better solution.

