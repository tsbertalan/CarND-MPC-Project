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
helps to avoid singularities -- i.e., for short-time projections, 
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


#### Update equations

The update equations are implied in a set of constraints supplied to IPOPT. 

While the constraints reduce the number of degrees of freedom,
IPOPT works in terms of a vector of variables
that includes the full projected trajectory.
Looping over a time index `t` from 1 to the number of projected timesteps `N`, 
we have for example

```c++
AD<double> x0 = vars[x_start + t - 1];
AD<double> x1 = vars[x_start + t];
```
and so on for `x`, `y`, `psi`, `cte`, `epsi`, `d`, and `a`.
We also evaluate our fitted polynomial surrogate for the lane centerline
(in vehicle coordinates) as `yDesired0`,
and compute the desired yaw as the arctangent of the polynomial slope.

State is then propagated forward according to the bicycle motion model,
in the form of imposed constraints.
```c++
fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
```

CTE at the next step is computed as the previous CTE
plus the growth due to current psi error.
```c++
fg[1 + cte_start + t] = cte1 - ((yDesired0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
```

Finally psi and psi error both grow their current values due to the turning rate.
```c++
auto dpsi = v0 * d0 / Lf * dt;
fg[1 + psi_start + t] = psi1 - (psi0 + dpsi);
auto epsi0_calc = psi0 - psiDesired0;
fg[1 + epsi_start + t] = epsi1 - (epsi0_calc + dpsi);
```



#### Choice of projective timestep size

The number-of-timesteps parameter `N` and timestep-size parameter `dt`
must be chosen to balance a several requirements. Namely,
 1. The product `N*dt` must be large enough to give the projection a meaningful predictive horizon.
 2. The timestep `dt` must be small enough to reduce to manageability the error imposed by the Euler-integration approximtion.
 3. The count `N` must be low enough that the MPC solution does not add excessive latency. 
 (Increasing `N` might impose a more-than-O(N) slowdown 
 since IPOPT must compute more control actions and thus must search a larger solution space.)
So, I increased or decreased these values to meet requirements with these heuristics in mind. 

When experimenting with objective-term coefficients to promote braking on tight turns
I experimented with larger `N` values (to allow for pre-emptive braking),
but didn't meet with much success for several possible reasons.



#### Dealing with latency

The coefficients chosen below were primarily tuned on one laptop
(a Thinkpad t470 with an i5 CPU),
which added a typical 67 milliseconds of latency per observation/actuation.
To this I add an extra 100 ms of artificial latency.
Without compensation,
removing the artificial 100ms latency actually appears to worsen performance.
I attribute this to the control actions being sent not at the moment of data measurment, 
but some time afterwards, so that the simulator environment at time of actuation
is not precisely that which was used when computing the MPC solution.
As such, the coefficients used were tuned 
for a qualitatively different dynamical system than
that that which emerges when no artificial latency is applied.

To make the method more robust,
I keep track of recent latency durations in a circular buffer
and take a running mean.
I then assume that this is the degree of staleness of the system state data
by the time the MPC-solution actuations are applied.
So, before passing a starting state to the MPC solver 
(but after transforming the road centerline points to the current vehicle frame) 
I apply the state update equations to predict where the vehicle will be 
by the time the solution is available.

The code for performing this projection mostly regurgitates
the state update equations (constraints) from the MPC solver.
The function `estimate_latency` computes the running mean of latency measurments,
and the actuations `d` and `a` are saved from the previous MPC solution
(or are zero at system start).
```c++
std::vector<double> get_delayed_state(double px, double py, double psi, double v) {
    double dt_est = estimate_latency();
    double px_delay = px + v * cos(psi) * dt_est;
    double py_delay = py + v * sin(psi) * dt_est;
    double psi_delay = psi + v / 2.67 * d * dt_est;
    double v_delay = v + a * dt_est;

    return {px_delay, py_delay, psi_delay, v_delay};
}
```
This is bundled in a `DelayPredictor` class which is only used within `main.cpp`.

It should be noted that this is just one step of Euler integration -- 
if latency is large, it may be beneficial 
to use something with higher-order accuracy, like a single Runge-Kutta step.
As an aside, the same might be said for the projected trajectories
used in the MPC objective function.

Adding this adjustment made it possible to re-tune the objective function weights/coefficients to achieve a top speed of about 97 mph rather than 56
(though the controller drops the speed as to low as 46 mph for the tight turns).

An alternative method
might be to add `(int) LATENCY/dt` dummy timesteps
to the projected trajectory,
during which the control actuations are constrained to be their previous values,
and not available to IPOPT for optimization.

In production, it may be desirable to use something like `ros::Rate` 
to promote consistent loop timing; 
or it could be that the latency compensation described above 
works best used with no rate-limiting.



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

  1. The coefficient on v error (from target) should be small -- it seems to work well
  to have a large target velocity that we don't expect to actually achieve.
  2. Wobble is best dealt with using the coefficients on the time-derivative of the actuator values.
  3. The coefficients on the CTE (essentially, y-error) and the ψ error need to be increased simultaneously.
  Raising these together will eventually lead to a dynamic where the car will aggressively brake before creeping around tighter turns.

With point 3 in mind, I had hoped to find parameter where I could achieve very high speeds
(about 80 mph) on the straight segments, and then automatically reduce to 40 or 50 mph for the curves.
So, I set about increasing these two primary coefficients with the v coefficient (1) 
to promote tighter adherance to the target speed.

![some braking](doc/somebraking.gif)

However, this generally led instead of to good centerline-following with braking before turns
to near curb-strikes (see above).
As I increased the coefficient on the v error, these got worse.
It's possible I could avoid this by decreasing the coefficient on turning angle d,
thus allowing for tighter turns, but I wasn't able to find acceptable parameters with extensive experimentation.
I suspect that it would be necessary to project further in the future to make this method really work,
for which I'd need to do some more code-optimization or perhaps get a faster machine.





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
