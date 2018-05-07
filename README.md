# CarND-Controls-MPC
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

In this project, I use a model-predictive controller (MPC) 
for setting both the throttle and the steering angle 
of a nonholonomic (Ackermann-like) vehicle.

![final parameters](doc/mpc.gif)

Here, we model vehicle motion with a so-called bicycle model,
where the vehicle's state is given by an (x,y) translation,
a yaw/heading angle ψ,
and a speed v.
The model is parameterized by a length `Lf`,
the distance between the steering axle and the center-of-mass.
This is one feature distinguishing it from the simpler differential-drive model
(and is the source of its nonholonomicity).

Briefly, the MPC is a shooting method
in which the dynamics of the model are simulated forward in time a short duration
(here, about 0.72 seconds), with the controls over the simulation chosen 
to minimize some loss function (such as difference from a reference trajectory).
This optimization is carried out anew for each outer timestep
(i.e., each new observation prompts a new optimization and then control response).

In addition to the sum of squared errors of the projected trajectory
from a reference trajectory (the road centerline), both in (x,y) and ψ,
we minimize the SSE of the velocity from some goal velocity,
the sum of squared steering angles (to discourage sharp turns),
the sum of squared throttle values (to discourage hard acceleration/deceleration),
and the sum of discrete time derivatives of these two actuation values
(to discourage sudden changes or wobble).

These multiple optimization targets are combined with manually chosen coefficients on each,
and tuning these coefficients produces drastically different closed-loop trajectories.



## Code structure

The main loop can be roughly described with five steps:

1. Receive data from simulator. Data consists of
  a. x and y points for road center in map (global) coordinates, and
  b. current car (x, y, ψ, v)
2. Make transformation from map to vehicle coordinates.
3. Transform road center points to vehicle coordinates.
4. Fit a polynomial in vehicle coordinates.
5. Request MPC solution (d, a).

Computing MPC solution (and the projected solution) in vehicle coordinates
helps to avoid singularities--i.e., for short-time projections, 
we can be reasonably certain that y will be a function of x
(where x is now defined as the forward direction of the car's initial point,
and positive y extends to the car's left).

The inner MPC solution relies on the IPOPT solver.
The projected trajectory is computed using a simple forward-Euler integration scheme,
and all quantities are computed not as simple floating-point,
but instead using `CppAD::AD<double>` instances. 
`CppAD` is then able to compute an analytical Jacobian
which greatly enhances IPOPT's ability to find an optimal (d, a) pair.
Generally, solutions are found in less than a dozen IPOPT iterations.
It's possible this could be further enhanced by supplying the solution
as an initial guess to the solver.



## Coefficient tuning

Since each closed-loop response requires the solution of a multi-objective optimization problem,
where the objectives are linearly combined with manually-chosen coefficients.

Initially, I chose reasonable values for these coefficients 
by monitoring the "typical" ranges of variability of the several components of the objective
during a moderat deviation of the car from centerline.
Since all the quantities are in drastically different units,
these typical ranges naturally varied significantly.

I then proceeded to tune the coefficients iteratively,
by obtaining acceptable performance at a low target speed
and incrementally increasing the speed 
and making changes to individual coefficients 
to correct in an intuitive way the problems that arose.

From this, I made several observations on the effect of the coefficients:

  *



## Building and Running

#### Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
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

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


##### Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`
5. Download the latest [Udacity Term 2 Simulator][4] and extract.
6. Run `term2_sim.x86_64` or `term2_sim.x86` as appropriate, and select the MPC sim.
