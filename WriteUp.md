# Udacity SDC Nanodegree
## Term 3 - Project 11
## Path Planning Project


The aim of this project was to write a path planner for the term 3 simulator.

*Machine setup:*
* Intel i5-6500 CPU
* 16GB RAM
* Nvidia GTX 1060 GPU
* Linux Mint 18 (based on Ubuntu 16.04)

*Problems encountered:*

* `getXY()`-function was unsuitable for transforming the quintic curve (s(t),d(t)) into Cartesian coordinates. Inspection of this function revealed that it utilizes linear interpolation between the 42 map points. This however leads to the phenomenon that the curved line will be broken at the map points.



#### Behavior Planner
The Behavior Planner is based on the principle that it plans a trajectory of a fixed distance of ![](https://latex.codecogs.com/gif.latex?%5CDelta%20s) ahead of the ego vehicle (![](https://latex.codecogs.com/gif.latex?%5CDelta%20s) = 90 meters).
The Trajectory Class accepts the total

* LC - Lane Keep
* LCL - Lane Change Left
* LCR - Lane Change Right

Figure ?? illustrates the state machine:

![](./images/Drawing1.png)

`current_lane` switches to `intended_lane` when the ego vehicle has entered the intended lane.

As indicated in the following figure, we subdivide the road in front of the vehicle (and to some extend also behind) into "buckets":

A for-loop iterates over all cars in the proximity of the ego vehicle (these where already filtered in a loop in the main()-function) and places them into a backet.
Each bucket has a certain cost. For example, cars farer away from the ego-vehicle are given a lower cost, and vehicles in the proximity of the ego vehicle are given a higher cost.

![](./images/Drawing2.png)


A second cost function is designed relatively straightforward

The advantage of this "bucket-approach" seems to be much less dependency on and fine-tuning of the cost-functions' weights.



#### Trajectory Generation
The Trajectory Class accepts the total time ![](https://latex.codecogs.com/gif.latex?%5CDelta%20T) as input as well as initial and final positions, and initial and final longitudinal velocities. The total time ensures that the mean speed along the interior of the trajectory stays in a range definied by the initial and final velocities. E.g., choosing ![](https://latex.codecogs.com/gif.latex?%5CDelta%20T) relatively large leads to the ego vehicle first decelerating and then accelerating again. Choosing ![](https://latex.codecogs.com/gif.latex?%5CDelta%20T) relatively large leads to the ego vehicle first accelerating and then decelerating again.



#### getXY()-function
The `getXY()`-function is of critical importance. As described above, it had to be modified in order to become useful.

![](https://latex.codecogs.com/gif.latex?%5C%7B%20P_0%2C%20P_1%2C%20P_2%2C%20P_3%20%5C%7D)

![](https://latex.codecogs.com/gif.latex?%5Cmathcal%7BK%7D_0%20%3D%20%5C%7B%20P_0%2C%20P_1%2C%20P_2%20%5C%7D)

![](https://latex.codecogs.com/gif.latex?%5Cmathcal%7BK%7D_1%20%3D%20%5C%7B%20P_1%2C%20P_2%2C%20P_3%20%5C%7D)

The algorithm makes use of the spline library by T.?. It picks the closest two map points in front of the ego vehicle, and the closest two map points behind it. It then generates two spline curves between the first three points and the last three points.

![](./images/Drawing3.png)


The two splines are not identical in the section between points 1 and 2. In order to find a smooth approximation for the whole track, we define the following weights:

![](https://latex.codecogs.com/gif.latex?p%20%3D%20d_2%5E%7B-2%7D) and ![](https://latex.codecogs.com/gif.latex?q%3D%20d_1%5E%7B-2%7D).

The norm is defined as:
![](https://latex.codecogs.com/gif.latex?n%20%3D%20p&plus;q)

This norm has the advantage that
  \lim_{p\to} Norm = 1

In short, if the ego vehicle approaches point 2 (i.e. ![](https://latex.codecogs.com/gif.latex?d_2%20%5Cto%200) and ![](https://latex.codecogs.com/gif.latex?d_1%20%5Cto%20%5CDelta%20d)),

![](https://latex.codecogs.com/gif.latex?%5Clim_%7Bp%5Cto%200%7D%20%5Cfrac%7Bp%7D%7Bn%7D%20%3D%20%5Clim_%7Bp%5Cto%200%7D%20%5Cfrac%7B1%7D%7B1&plus;p%5E2/q%5E2%7D%20%3D%201)

![](https://latex.codecogs.com/gif.latex?%5Clim_%7Bq%20%5Cto%200%7D%20%5Cfrac%7Bq%7D%7Bn%7D%20%3D%20%5Clim_%7Bq%20%5Cto%200%7D%20%5Cfrac%7B1%7D%7B1&plus;p%5E2/q%5E2%7D%20%3D%200)

Whereas, if the ego vehicle approaches point 1 (think of reversing time), we have ![](https://latex.codecogs.com/gif.latex?d_2%20%5Cto%20%5CDelta%20d) and ![](https://latex.codecogs.com/gif.latex?d_1%20%5Cto%200), hence

What then happens when the ego vehicle crosses point 2 is rather obvious: A new point is added as point 4, and point 0 is eliminated from the list.



#### Failed attempts
Since there were only relatively little guidelines for this project,

**Stochastic trajectory generation**
By this we mean choosing random endpoints for the trajectories and choosing the one with the lowest costs. The problem with this approach is that while it worked in principle, it has eaten too many resources.

**Calculating cost of trajectory**
I.e. predicting the movement of each target vehicle and calculating collision trajectories. Again, this methods has taken too much resources. Calculating a few dozen trajectories takes a few seconds.



#### Possible Modifications
One possible modification which I have only shortly delved into is the possibility to enable near real-time path planning akin to the MPC project.



#### Comment on the Provided Framework
Most students I have communicated with share the same opinion that fixing the was quite distracting from the primary objective of
