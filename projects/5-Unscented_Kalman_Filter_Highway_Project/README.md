# SFND_Unscented_Kalman_Filter
## Sensor Fusion UKF Highway Project Starter Code

<img src="media/ukf_highway_tracked.gif" width="700" height="400" />

In this project you will implement an Unscented Kalman Filter to estimate the state of multiple cars on a highway using noisy lidar and radar measurements. Passing the project requires obtaining RMSE values that are lower that the tolerance outlined in the project rubric.

Download pcd data from [here](https://github.com/udacity/SFND_Unscented_Kalman_Filter/tree/master/src/sensors/data/pcd) to [./src/sensors/data/pcd/](./src/sensors/data/pcd/)

The main program can be built and ran by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./ukf_highway

Note that the programs that need to be written to accomplish the project are `src/ukf.cpp`, and `src/ukf.h`

The program `main.cpp` has already been filled out, but feel free to modify it.

<img src="media/ukf_highway.png" width="700" height="400" />

`main.cpp` is using `highway.h` to create a straight 3 lane highway environment with 3 traffic cars and the main ego car at the center.
The viewer scene is centered around the ego car and the coordinate system is relative to the ego car as well. The ego car is green while the other traffic cars are blue. The traffic cars will be accelerating and altering their steering to change lanes. Each of the traffic car's has it's own UKF object generated for it, and will update each indidual one during every time step.

The red spheres above cars represent the (x,y) lidar detection and the purple lines show the radar measurements with the velocity magnitude along the detected angle. The Z axis is not taken into account for tracking, so you are only tracking along the X/Y axis.

---

## Additional Project Details

### Highway Parameters

In `highway.h` there are a number of parameters that can be modified to help with testing and understanding.

```cpp
// Parameters
// --------------------------------
// Set which cars to track with UKF
std::vector<bool> trackCars = {true,true,true};
// Visualize sensor measurements
bool visualize_lidar = true;
bool visualize_radar = true;
bool visualize_pcd = false;
// Predict path in the future using UKF
double projectedTime = 0;
int projectedSteps = 0;
// --------------------------------
```

The `trackCars` list can be used to toggle on/off cars for the UKF objects to track. The default is to track all three cars on the road, but for testing it can be nice to toggle to only track one at a time. For instance to only track the first car `{true,false,false}`.

<img src="media/ukf_highway_projected.gif" width="700" height="400" />

The animation above shows what it looks like if the `projectedTime` and `projectedSteps` variables are used. Also in the animation above the visualization for lidar and radar has been set to `false` and `projectedTime = 2` and `projectedSteps = 6`. The green spheres then show the predicted position for the car in the future over a 2 second interval. The number of steps increases the number of positions to interpolate the time interval.

It's interesting to see the predicted path which is constrained by the motion model. In this project the motion model used is CTRV, which assumes constant velocity and turning rate. Since our cars do not have constant turning rates you can see the predicted paths swing around and take a while to correct after the car begins moving straight again.

<img src="media/ukf_highway_pcd.gif" width="700" height="400" />

The `visualize_pcd` parameter can be set to `true` to visualize the cars from the lidar's perspective from point clouds. The traffic pcd data is available in `src/sensors/data/pcd` directory. As a bonus assignment this data could be used to cluster the individual cars using techniques from Lidar Obstacle Detection and then bounding boxes could be fitted around the car clusters. The bounding box center (x,y) point could then be used to represent the lidar marker that would be fed into the UKF instead of the project pre-generated lidar marker from `tools.cpp` `lidarSense` function.

---

## Other Important Dependencies
* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
 * PCL 1.2

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./ukf_highway`

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html) as much as possible.

## Generating Additional Data

This is optional!

If you'd like to generate your own radar and lidar modify the code in `highway.h` to alter the cars. Also check out `tools.cpp` to change how measurements are taken, for instance lidar markers could be the (x,y) center of bounding boxes by scanning the PCD environment and performing clustering. This is similar to what was done in Sensor Fusion Lidar Obstacle Detection.

## Project Instructions and Rubric

This information is only accessible by people who are already enrolled in Sensor Fusion.
If you are enrolled, see the project page in the classroom for instructions and the project rubric.

---

## Project Implementation

- [project_rubric.pdf](./project_rubric.pdf)
- code implementation of the methods and solutions described above is in [src](./src/)
