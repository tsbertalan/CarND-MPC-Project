#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"


using CppAD::AD;
using std::cout;
using std::cout;




//////// PARAMETERS ////////

// Set the timestep length and duration
static const size_t N = 10;
static const double dt = 0.14;

// Set the target velocity (converted from mph to m/s)
static const double ref_v = 120 * 5280. / 3.2808 / 3600.;

// Set the scaling factors for terms in the objective function.
static const double scale_cte = 120;
static const double scale_epsi = 5000;
static const double scale_v = 4;
static const double scale_delta = 1e4;
static const double scale_a = 10;
static const double scale_ddelta = 1e8;
static const double scale_da = 1e6;


// If debugging the MPC solver33, set print_level to "4".
#define DEBUG false
std::string print_level = "0";
std::string tol = "1e-6";




// The solver takes all the state variables and actuator
// variables in a singular vector. Thus, we should to establish
// when one variable starts and another ends to make our lifes easier.
size_t x_start = 0;
size_t y_start = x_start + N;
size_t psi_start = y_start + N;
size_t v_start = psi_start + N;
size_t cte_start = v_start + N;
size_t epsi_start = cte_start + N;
size_t delta_start = epsi_start + N;
size_t a_start = delta_start + N - 1;





// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;

// Evaluate a polynomial.
AD<double> polyevalAD(Eigen::VectorXd coeffs, AD<double> x) {
    AD<double> result = 0.0;
    for (int i = 0; i < coeffs.size(); i++) {
        result += coeffs[i] * CppAD::pow(x, i);
    }
    return result;
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
    double result = 0.0;
    for (int i = 0; i < coeffs.size(); i++) {
        result += coeffs[i] * pow(x, i);
    }
    return result;
}

class FG_eval {
public:
    // Fitted polynomial coefficients
    Eigen::VectorXd coeffs;

    FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

    typedef CPPAD_TESTVECTOR(AD<double>) ADvector;

    void operator()(ADvector &fg, const ADvector &vars) {
        // `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)
        // NOTE: You'll probably go back and forth between this function and
        // the Solver function below.
        // The cost is stored is the first element of `fg`.
        // Any additions to the cost should be added to `fg[0]`.
        fg[0] = 0;



        // Reference State Cost
        // Define the cost related the reference state.
        for (unsigned int t = 0; t < N; t++) {
            // Minimize the CTE.
            // Typical unscaled value is ~ 1e2;
            fg[0] += scale_cte * CppAD::pow(vars[cte_start + t], 2);

            // Minimize the angular error.
            // Typical unscaled value is ~ 1e-5;
            fg[0] += scale_epsi * CppAD::pow(vars[epsi_start + t], 2);

            // Minimize the difference from reference velocity.
            // Typical unscaled value is ~ 1e3;
            fg[0] += scale_v * CppAD::pow(vars[v_start + t] - ref_v, 2);

            if (t < N - 1) {
                // Minimize the control actions.
                // Typical unscaled value is ~ 1e-5;
                fg[0] += scale_delta * CppAD::pow(vars[delta_start + t], 2);

                // Typical unscaled value is ~ 1e-2;
                fg[0] += scale_a * CppAD::pow(vars[a_start + t], 2);

                if (t < N - 2) {
                    // Minimize the abruptness of control action changes.
                    // Typical unscaled value is ~ 1e-5;
                    fg[0] += scale_ddelta * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);

                    // Typical unscaled value is ~ 1e-3;
                    fg[0] += scale_da * CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
                }
            }
        }



        //
        // Setup Constraints
        //
        // Initial constraints
        //
        // We add 1 to each of the starting indices due to cost being located at
        // index 0 of `fg`.
        // This bumps up the position of all the other values.
        fg[1 + x_start] = vars[x_start];
        fg[1 + y_start] = vars[y_start];
        fg[1 + psi_start] = vars[psi_start];
        fg[1 + v_start] = vars[v_start];
        fg[1 + cte_start] = vars[cte_start];
        fg[1 + epsi_start] = vars[epsi_start];

        // The rest of the constraints
        for (unsigned int t = 1; t < N; t++) {
            AD<double> x0 = vars[x_start + t - 1];
            AD<double> x1 = vars[x_start + t];

            AD<double> y0 = vars[y_start + t - 1];
            AD<double> y1 = vars[y_start + t];

            AD<double> psi0 = vars[psi_start + t - 1];
            AD<double> psi1 = vars[psi_start + t];

            AD<double> v0 = vars[v_start + t - 1];
            AD<double> v1 = vars[v_start + t];

            AD<double> cte0 = vars[cte_start + t - 1];
            AD<double> cte1 = vars[cte_start + t];

            AD<double> epsi0 = vars[epsi_start + t - 1];
            AD<double> epsi1 = vars[epsi_start + t];

            AD<double> d0 = vars[delta_start + t - 1];
            AD<double> a0 = vars[a_start + t - 1];
            AD<double> yDesired0 = polyevalAD(coeffs, x0);
            AD<double> psiDesired0 = CppAD::atan(coeffs[1]);

            // NOTE: The use of `AD<double>` and use of `CppAD`!
            // This is also CppAD can compute derivatives and pass
            // these to the solver.

            // Setup the rest of the model constraints
            fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
            fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
            fg[1 + v_start + t] = v1 - (v0 + a0 * dt);

            // CTE at the next step is the previous CTE plus the growth due to current psi error.
            fg[1 + cte_start + t] = cte1 - ((yDesired0 - y0) + (v0 * CppAD::sin(epsi0) * dt));

            // Psi and psi error both grow their current values due to the turning rate.
            auto dpsi = v0 * d0 / Lf * dt;
            fg[1 + psi_start + t] = psi1 - (psi0 + dpsi);
            auto epsi0_calc = psi0 - psiDesired0; // Why isn't this just epsi0?
            fg[1 + epsi_start + t] = epsi1 - (epsi0_calc + dpsi);
        }
    }
};


// Print a squared value, perhaps with a scaling. Really only for debugging.
double psquared(double x, std::string name, double scale=1.0) {
    double err = scale * pow(x, 2);
    cout << name << "^2 ";
    if(scale != 1.0) {
        cout << "* " << scale << " ";
    }
    cout << "= ";
    cout << err << endl;
    return err;
}

//
// MPC class definition implementation.
//
MPC::MPC() {}

MPC::~MPC() {}

MPC_Solution MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
    bool ok = true;
    size_t i;
    typedef CPPAD_TESTVECTOR(double) Dvector;

    // Set the number of model variables (includes both states and inputs).
    // (State is (x, y, psi, v, cte, epsi) and actuators is (d, a).
    const size_t n_vars = 6 * N + 2 * (N - 1);

    // Set the number of constraints
    const size_t n_constraints = N * 6;

    const double x = 0;//state[0];
    const double y = 0;//state[1];
    const double psi = 0;//state[2];

    const double v = state[3];
    const double cte = state[4];
    const double epsi = state[5];

    // Initial value of the independent variables.
    // SHOULD BE 0 besides initial state.
    Dvector vars(n_vars);
    for (i = 0; i < n_vars; i++) {
        vars[i] = 0;
    }

    // Set lower and upper limits for variables.
    Dvector vars_lowerbound(n_vars);
    Dvector vars_upperbound(n_vars);

    // Set all non-actuators upper and lower limits
    // to the max negative and positive values.
    for (i = 0; i < delta_start; i++) {
        vars_lowerbound[i] = -1.0e19;
        vars_upperbound[i] = 1.0e19;
    }

    // The upper and lower limits of delta are set to -25 and 25
    // degrees (values in radians).
    // NOTE: Feel free to change this to something else.
    for (i = delta_start; i < a_start; i++) {
        vars_lowerbound[i] = -0.436332;
        vars_upperbound[i] = 0.436332;
    }

    // Acceleration/deceleration upper and lower limits.
    // NOTE: Feel free to change this to something else.
    for (i = a_start; i < n_vars; i++) {
        vars_lowerbound[i] = -1.0;
        vars_upperbound[i] = 1.0;
    }

    // Lower and upper limits for the constraints
    // Should be 0 besides initial state.
    Dvector constraints_lowerbound(n_constraints);
    Dvector constraints_upperbound(n_constraints);
    for (i = 0; i < n_constraints; i++) {
        constraints_lowerbound[i] = 0;
        constraints_upperbound[i] = 0;
    }

    // Initial state.
    constraints_lowerbound[x_start] = x;
    constraints_lowerbound[y_start] = y;
    constraints_lowerbound[psi_start] = psi;
    constraints_lowerbound[v_start] = v;
    constraints_lowerbound[cte_start] = cte;
    constraints_lowerbound[epsi_start] = epsi;

    constraints_upperbound[x_start] = x;
    constraints_upperbound[y_start] = y;
    constraints_upperbound[psi_start] = psi;
    constraints_upperbound[v_start] = v;
    constraints_upperbound[cte_start] = cte;
    constraints_upperbound[epsi_start] = epsi;

    // object that computes objective and constraints
    FG_eval fg_eval(coeffs);

    //
    // NOTE: You don't have to worry about these options
    //
    // options for IPOPT solver
    std::string options;
    // Uncomment this if you'd like more print information
    options += "Integer print_level  " + print_level + "\n";
    // NOTE: Setting sparse to true allows the solver to take advantage
    // of sparse routines, this makes the computation MUCH FASTER. If you
    // can uncomment 1 of these and see if it makes a difference or not but
    // if you uncomment both the computation time should go up in orders of
    // magnitude.
    options += "Sparse  true        forward\n";
    options += "Sparse  true        reverse\n";
    // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
    // Change this as you see fit.
    options += "Numeric max_cpu_time          0.5\n";

    options += "Numeric tol " + tol + "\n";

    // place to return solution
    CppAD::ipopt::solve_result<Dvector> solution;

    // solve the problem
    CppAD::ipopt::solve<Dvector, FG_eval>(
            options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
            constraints_upperbound, fg_eval, solution);

    // Check some of the solution values
    ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;


    // Print some parts of the objective.
    if(DEBUG) {
        vector<double> parts;
        const vector<string> names = {"cte", "epsi", "v", "d", "a", "dd", "da"};
        cout << endl << "Objective Components:" << endl;
        parts.push_back(psquared(cte, "cte", scale_cte));
        parts.push_back(psquared(epsi, "epsi", scale_epsi));
        parts.push_back(psquared(v-ref_v, "(v-ref_v)", scale_v));

        parts.push_back(psquared(solution.x[delta_start], "d", scale_delta));
        parts.push_back(psquared(solution.x[a_start], "a", scale_a));

        double dd = solution.x[delta_start+1] - solution.x[delta_start];
        parts.push_back(psquared(dd, "dd", scale_ddelta));
        double da = solution.x[a_start+1] - solution.x[a_start];
        parts.push_back(psquared(da, "da", scale_da));

        // Sort names by corresponding objective component values.
        vector<int> indices = {0, 1, 2, 3, 4, 5, 6};
        sort(indices.begin(), indices.end(), [&](int x, int y){return parts[x] > parts[y];});

        cout << "Order of objective contributions: ";
        for(int index : indices) {
            cout << names[index] << " > ";
        }
        cout << "0" << endl;


        // Cost
        auto cost = solution.obj_value;
        cout << "Cost " << cost << endl;
    }


    // Return the first actuator values. The variables can be accessed with
    MPC_Solution result;
    result.variables = {solution.x[x_start], solution.x[y_start],
                        solution.x[psi_start], solution.x[v_start],
                        solution.x[cte_start], solution.x[epsi_start],
                        solution.x[delta_start], solution.x[a_start]};
    for(i=0; i<N-1; i++) {
        double x_vehicle = solution.x[x_start+i];
        double y_vehicle = solution.x[y_start+i];
        result.path.x.push_back(x_vehicle);
        result.path.y.push_back(y_vehicle);

        double x_fit = x_vehicle;
        double y_fit = polyeval(coeffs, x_fit);
        result.fit.x.push_back(x_fit);
        result.fit.y.push_back(y_fit);
    }

    return result;
}
