# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

## Reflection

### Effect each of the P, I, D components in implementation.

<!-- Student describes the effect of the P, I, D component of the PID algorithm in their implementation. Is it what you expected? 
Visual aids are encouraged, i.e. record of a small video of the car in the simulator and describe what each component is set to.
-->

PID controller is a control loop feedback mechanism. The correction applied is based on three terms, which gives the name:

* P stands for proportional gain. We apply a correction inversely proportional to the cross track error, which is the difference between current y position and expected y position. Controllers only based on P tend to overshoot, and get marginally stable, wobbling around the reference trajectory.

* D stands for differential gain. To minimize the cross track error rate, we add a temporal derivative of cross track error at time t, given by `cte_t - cte_{t-1} / delta_t`. Adding this term to our correction will ensure smoother changes to steering angle and help converge to reference trajectory

* I stands for integral gain. Environmental factors or mechanical defects may add a systematic bias, which leads to steady state error in PD controllers. So we add to the correction a term proportional to the sum of all cross track errors observed. This will allow the controller to correct for systematic biases identified.

Kp, Kd and Ki are the constants used to weight each of these gains.

In our project we first set Kp, until the car starts to wobble. At this point the car can drive the track, but response is jerky and certainly not comfortable to ride in. It can also go off road in curves depending on its starting position at the beggining of the curve.

To correct for that, we set D, until the car sets in a stable riding.

In our implementation, no visible systematic bias were identified, although there is a possibility there is a small bias. So our car often performed better with Ki set to 0.

### How the final hyperparameters were chosen.

<!-- Student discusses how they chose the final hyperparameters (P, I, D coefficients). This could be have been done through manual tuning, twiddle, SGD, or something else, or a combination! -->

This project was easy to tune by hand, by starting with the values discussed by Sebastian (Kp=0.2, Kd=3, Ki=0.004) and tuning from there, following the approach discussed above.

However for an extra challenge I implemented twiddle, setting start values to 0 and probing interval from -1 to +1.

At every 3200 time frames (a full lap at throttle 0.3) I would run twiddle and reset the simulator. Reseting the simulator is required to ensure we have the same track section and the errors are comparable.

Oddly enough, twiddle seemed to converged to the same proportion discussed in the lesson, although I did not run twiddle to to conversion due to lack of time (each lap takes about 1 minute). Final choice of values is {0.25, 3.0, 0.00001}.

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
* [uWebSockets](https://github.com/uWebSockets/uWebSockets) == 0.13, but the master branch will probably work just fine
  * Follow the instructions in the [uWebSockets README](https://github.com/uWebSockets/uWebSockets/blob/master/README.md) to get setup for your platform. You can download the zip of the appropriate version from the [releases page](https://github.com/uWebSockets/uWebSockets/releases). Here's a link to the [v0.13 zip](https://github.com/uWebSockets/uWebSockets/archive/v0.13.0.zip).
  * If you run OSX and have homebrew installed you can just run the ./install-mac.sh script to install this
* Simulator. You can download these from the [project intro page](https://github.com/udacity/CarND-PID-Control-Project/releases) in the classroom.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

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
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/e8235395-22dd-4b87-88e0-d108c5e5bbf4/concepts/6a4d8d42-6a04-4aa6-b284-1697c0fd6562)
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
