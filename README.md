# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

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
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

There's an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

## PID Control Parameters and Selection

PID Control is used to generate control signals suitable for maintaining system (car in this case) in desired state (middle of the road in this case) among dynamic conditions (wiggling road in this case). It uses amount of current deviation from desired state as the input, and provides amount of corrective action as output (steering angle in this case). Control output is calculated as linear combination of 3 kinds of error (present, differential and integral) multiplied by respective coefficients (Kp, Ki, and Kd). Intuitive behavior of these parameters is as follows:
* Kp controls the magnitude of controller's tendency to bring the system into desired state. Higher value will mean more pronounced controller response. Excessive value results in exaggerated output, while too small results in insufficient output.
* Ki controls the amount by which controller tends to compensate for system bias (which is normal).
* Kd controls the tendency of controller to correct for overshooting the target state due to Kp being higher than needed (which is also normal).

Choice of PID parameters in this case is affected by following factors:
* High or low values result in car over or under steering and thus running off road.
* Increased speed results in increased effects of steering output. Thus PID parameters that are appropriate at lower speed tend to be too high when speed increases.

In order to find parameters adequate to get the car around the track, I have resorted to stabilizing velocity using separate PID controller for throttle. I have also implemented Twiddle algorithm using state machine approach. Twiddle algorithm is supposed to converge to optimal parameter set in theory, but during the course of search it is prone to trying inadequate parameters which result in car running off road and thus interrupt the search. Recovery from this situation poses it's own challenges. My approach was to run twiddle algorithm until first crash and then pick the set of parameters that resulted in minimum error. I was able to find parameters that work within requirements for speed of 15 mph. At higher speeds, it was difficult to get the twiddle algorithm to  produce anything useful before it causes car to crash, thus calling for robust solution to restart/recovery problem. So I stayed at 15 MPH for now.
