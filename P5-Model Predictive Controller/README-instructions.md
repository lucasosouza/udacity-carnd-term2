# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

## The Model, Polynomial Fitting and MPC Preprocessing

<!---
Student describes their model in detail. This includes the state, actuators and update equations.
-->

<!---
A polynomial is fitted to waypoints.

If the student preprocesses waypoints, the vehicle state, and/or actuators prior to the MPC procedure it is described.
-->

In this project, we implement a Model Predictive Control model, also called Receding Horizon Control. The idea behind MPC is simple, and I will describe it step by step below.

#### Step 1: Capture current state
From the path planning module, we receive a set of coordinates x and y which indicates the path the car needs to follow in order to arrive at its destination, or the reference trajectory.

#### Step 2: Convert coordinates
At this project, the coordinates are given in the map coordinate system. Since we will model MPC based on the car coordinate system, we first need to convert the coordinates from map to car coordinate system.

In the local reference frame, or the car coordinate system, the car is positioned at the intersection of x and y, facing east. So px, py and orientation are set to 0 in the initial state, and the remaining points from the reference trajectory transformed from global to local reference frame.

#### Step 3: Fit a polynomial 

With our converted coordinates, we fit a 3rd degree polynomial which indicates the reference trajectory. This polynomial will be the input to the MPC.

#### Step 4: Defining our initial state
We define our state with six variables:

1. position x (px)
2. position y (py)
3. orientation (psi)
4. velocity (v)
5. cross track error (cte)
6. orientation error (epsi).

In MPC we will optimize steering angle and acceleration, which is given by delta and alpha respectively.

#### Step 5: Optimizing

We frame the MPC problem as an optimization problem, with the state and polynomial as our inputs.

To set the optimizer, we first define our objectives and constraints. The goal of the optimizer is to reduce the cost function, define at fg[0]. 

How we define the constant function is paramount to the results we aim to achieve. Several aspects are considered in the cost calculation. To minimize distance between state and reference state, we add the following to the cost:

* `(cte - reference_cte)^2`
* `10*(epsi - reference_epsi)^2`
* `(v - reference_v)^2`

To minimize magnitude of actuators, it is added:

* `100 * (delta)^2`
* `(alpha)^2`

And to minimize rate of change in actuators:

* `100000 * (delta_t - delta_t-1)^2`
* `(alpha_t - alpha_t-1)^2`

A high weight has been attributed to the orientation error, magnitude and rate of change. The high weight attributed to these aspects penalizes sudden changes in steering angles or high values, leading to a "smoother ride".

The constraints are given by the vehicle dynamics model, which indicate the transition functions from state t to state t+1. We are using the kinematics model, which has the following transition functions:

* `x_{t+1} = x_t + v_t * cos(psi_t) * dt`
* `y_{t+1} = y_t + v_t * sin(psi_t) * dt`
* `psi_{t+1} = psi_t + (v_t / L_f) * delta * dt`
* `v_{t+1} = v_t + a_t * dt`

The only variable we have not discussed yet is L_f, which measures the distance between the front of the vehicle and its center of gravity, and impacts the how fast the vehicle can turn. It is a fixed physical characteristic of the vehicle which was measured empirically. 

Every variable value i is limited by an upper and lower bound, that indicates the range of possible values that can be attributed to the variable. We define the bounds of all non-actuators variables to be the max negative and positive values, the bounds of steering angle delta to be -25 and +25 degrees in radians inclusive (the steering limit defined in the simulator), and the bouds of acceleration to be -1 and +1 inclusive (also limits defined in the simulator, in which throttle ranges from -1 to +1).

#### Step 6: Actuating

The optimizer will calculate the optimal trajectory which yields the lowest cost given the constraints.

We are interested only in the actuator values, specifically the first set of actuators (or one of the first, as seen later under Latency). This set {delta, alpha} is handled back to the car which executes the command. The rest of values are not required and can be discarded.

#### Step 7: Loop

After we execute the command, the car will get into a new state. We go back to Step 1 to capture current state.


## Timestep Length and Elapsed Duration (N & dt)

<!---
Student discusses the reasoning behind the chosen N (timestep length) and dt (elapsed duration between timesteps) values. Additionally the student details the previous values tried.
-->

How long into the future will our MPC predict? To answer that, we define a prediction horizon T, which is composed of two parts: N, the timestep length, and dt, the elapsed duration between timesteps. Since we are using only the first set of actuators (or one of the firsts, as discussed in the latency topic), there is no need to predict a large horizon.

Predicting farther into the future is a harder problem, as you might need a higher degree polynomial to fit a more complex path. It is also computationally more expensive. So for the simulation we picked the smallest value for N, which would still be enough for the car to drive safely and within requirements. This value was chosen experimentally. We started with N=5, which drove the car offroad, and gradually increased. Around N=30, the 3rd degree polynomial is not suitable to fit the path 30 timesteps in the future, and the green line showing predicted path starts to diverge. The final value chosen is N=15, which performs as well as higher values with less computational cost. Dt was set to 0.05, so the total predicted horizon is 600ms (50ms x 12).

## Model Predictive Control with Latency

<!---
The student implements Model Predictive Control that handles a 100 millisecond latency. Student provides details on how they deal with latency.
-->

In an actual implementation, there is a delay between the moment you issue the actuation command and the car executes the command. That is the problem called "latency".

For the purpose of this simulation, we considered the latency of 100ms. To deal with latency, instead of picking the first set of actuators calculated by MPC, we pick the actuator values at the time the car will actually execute the command.

In this case dt is 0.05 seconds, or 50ms. So if we pick the third set of values, we handle to the car the actuation command needed 100ms in the future, which is equivalent to the latency. So the command will get executed in the correct time frame as predicted in our MPC model.

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Fortran Compiler
  * Mac: `brew install gcc` (might not be required)
  * Linux: `sudo apt-get install gfortran`. Additionall you have also have to install gcc and g++, `sudo apt-get install gcc g++`. Look in [this Dockerfile](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/Dockerfile) for more info.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * Mac: `brew install ipopt`
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/) or the [Github releases](https://github.com/coin-or/Ipopt/releases) page.
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `bash install_ipopt.sh Ipopt-3.12.1`. 
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions


1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./
