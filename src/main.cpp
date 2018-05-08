#include <math.h>
#include <algorithm>  // std::min, std::max
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

// For latency measurment/compensation.
#include <boost/circular_buffer.hpp>
#include <chrono>




//////// PARAMETERS ////////
#define poly_order 3
// NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
// SUBMITTING.
#define LATENCY 100
#define MAX_DELAYS 10




// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }

double deg2rad(double x) { return x * pi() / 180; }

double rad2deg(double x) { return x * 180 / pi(); }


// Return a duple.
struct Pair {
    double x;
    double y;
};


// Wrap a rotation+translation in a nice class.
class Transformation_Matrix {
private:
    Eigen::Matrix<double, 3, 3> A;
    double theta, xt, yt;

public:
    Transformation_Matrix(double theta, double xt, double yt) {
        this->theta = theta;
        this->xt = xt;
        this->yt = yt;

        double c = cos(theta);
        double s = sin(theta);
        A << c,-s, xt,
            s, c, yt,
            0, 0, 1;
    }

    Pair operator()(double xa, double ya) {
        Eigen::Matrix<double, 3, 1> XA;
        XA << xa, ya, 1;
        auto XB = A * XA;
        Pair result;
        result.x = XB(0, 0);
        result.y = XB(1, 0);
        return result;
    }

    Transformation_Matrix inverse() {
        // Derived by inverting A with Mathematica.
        double xr = -xt * cos(theta) - yt * sin(theta);
        double yr =  xt * sin(theta) - yt * cos(theta);
        return Transformation_Matrix(-theta, xr, yr);
    }
};


// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
    auto found_null = s.find("null");
    auto b1 = s.find_first_of("[");
    auto b2 = s.rfind("}]");
    if (found_null != string::npos) {
        return "";
    } else if (b1 != string::npos && b2 != string::npos) {
        return s.substr(b1, b2 - b1 + 2);
    }
    return "";
}


// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
    assert(xvals.size() == yvals.size());
    assert(order >= 1 && order <= xvals.size() - 1);
    Eigen::MatrixXd A(xvals.size(), order + 1);

    for (int i = 0; i < xvals.size(); i++) {
        A(i, 0) = 1.0;
    }

    for (int j = 0; j < xvals.size(); j++) {
        for (int i = 0; i < order; i++) {
            A(j, i + 1) = A(j, i) * xvals(j);
        }
    }

    auto Q = A.householderQr();
    auto result = Q.solve(yvals);
    return result;
}



class DelayPredictor {
private:
    double d, a;
    std::chrono::time_point<std::chrono::system_clock> t_prev;
    boost::circular_buffer<double> previous_delay_times;

public:
    DelayPredictor() : previous_delay_times(MAX_DELAYS) {
        d = 0;
        a = 0;
        previous_delay_times.push_back(.001*LATENCY);
        t_prev = std::chrono::system_clock::now();
    }

    double estimate_latency() {
        double dt_est = 0;
        for(auto dt : previous_delay_times) {
            dt_est += dt;
        }
        dt_est /= previous_delay_times.size();
        return dt_est;
    }

    std::vector<double> get_delayed_state(double px, double py, double psi, double v) {
        double dt_est = estimate_latency();
        double px_delay = px + v * cos(psi) * dt_est;
        double py_delay = py + v * sin(psi) * dt_est;
        double psi_delay = psi + v / 2.67 * d * dt_est;
        double v_delay = v + a * dt_est;

        return {px_delay, py_delay, psi_delay, v_delay};
    }

    void set_previous_control_actuations(double steer_value, double throttle_value) {
        d = steer_value * -deg2rad(25.0);
        a = throttle_value;

        // Measure the latency.
        std::chrono::time_point<std::chrono::system_clock> t
        = std::chrono::system_clock::now();
        std::chrono::duration<double> dt = t - t_prev;
        set_time(t);
        previous_delay_times.push_back(dt.count());
    }

    void set_time(std::chrono::time_point<std::chrono::system_clock> t) {
        t_prev = t;
    }

    void set_time() {
        set_time(std::chrono::system_clock::now());
    }
};



int main() {
    uWS::Hub h;

    // MPC is initialized here!
    MPC mpc;

    // Keep track of most-recent actuations.
    DelayPredictor delay_predictor;

    h.onMessage([&mpc, &delay_predictor](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                       uWS::OpCode opCode) {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        string sdata = string(data).substr(0, length);
        if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
            string s = hasData(sdata);
            if (s != "") {
                auto j = json::parse(s);
                string event = j[0].get<string>();
                if (event == "telemetry") {

                    // Extract the current state and nearby road midpoints.
                    vector<double> ptsx = j[1]["ptsx"];
                    vector<double> ptsy = j[1]["ptsy"];
                    double px = j[1]["x"];
                    double py = j[1]["y"];
                    double psi = j[1]["psi"];
                    double v = j[1]["speed"];

                    // Estimate a transformation from vehicle to map and v/v.
                    Transformation_Matrix vehicle2map(psi, px, py);
                    Transformation_Matrix map2vehicle = vehicle2map.inverse();

                    // Predict the next state with latency.
                    // Running the MPC takes time, so, realistically, it should assume
                    // not a starting point of the current state,
                    // but the current state plus a small uncontrolled delay.
                    // See MPC.cpp for some more details on this motion model.
                    auto delayed_state = delay_predictor.get_delayed_state(px, py, psi, v);
                    px = delayed_state[0];
                    py = delayed_state[1];
                    psi = delayed_state[2];
                    v = delayed_state[3];

                    // Fit a polynomial to the centerline coordinates.
                    // Sadly, polyfit wants a VectorXD, not a vector<double>.
                    Eigen::VectorXd ptsx_v(ptsx.size());
                    Eigen::VectorXd ptsy_v(ptsy.size());
                    ptsx_v.fill(0);
                    ptsy_v.fill(0);
                    vector<double> ptsx_vehicle, ptsy_vehicle;
                    for(unsigned int i=0; i<ptsx.size(); i++) {
                        Pair polyxy = map2vehicle(ptsx[i], ptsy[i]);
                        ptsx_v[i] = polyxy.x;
                        ptsy_v[i] = polyxy.y;
                        ptsx_vehicle.push_back(polyxy.x);
                        ptsy_vehicle.push_back(polyxy.y);
                    }
                    auto coeffs = polyfit(ptsx_v, ptsy_v, std::min((int) ptsx.size()-1, poly_order));

                    // calculate the cross track error
                    // Negative sign is here because if the poly evaluates positive, our y coordinate (0) is too small.
                    double cte = -polyeval(coeffs, px);

                    // calculate the orientation error
                    // Negative sign is here because if slope is positive, angle is positive,
                    // and our angle of 0 radians is too small.
                    double epsi = -atan(coeffs[1]);

                    // First three state values (x, y, psi) are all zero because we're considering MPC solutions
                    // that start from the car's position in its own coordinate frame.
                    Eigen::VectorXd state(6);
                    state << 0, 0, 0, v, cte, epsi;

                    /*
                    * Calculate steering angle and throttle using MPC.
                    *
                    * Both are in between [-1, 1].
                    *
                    */
                    auto result = mpc.Solve(state, coeffs);
                    auto vars = result.variables;
                    double steer_value = -vars[6] / deg2rad(25.0);
                    // If braking is causing the simulator to stick, consider only using the gas.
                    //double throttle_value = max(vars[7], 0.0);
                    double throttle_value = vars[7];

                    json msgJson;
                    msgJson["steering_angle"] = steer_value;
                    msgJson["throttle"] = throttle_value;

                    //Display the MPC predicted trajectory
                    //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
                    // the points in the simulator are connected by a green line
                    vector<double> mpc_x_vals = result.path.x;
                    vector<double> mpc_y_vals = result.path.y;
                    msgJson["mpc_x"] = mpc_x_vals;
                    msgJson["mpc_y"] = mpc_y_vals;

                    //Display the waypoints/reference line
                    //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
                    // the points in the simulator are connected by a yellow line
                    vector<double> next_x_vals = result.fit.x;
                    vector<double> next_y_vals = result.fit.y;
                    msgJson["next_x"] = next_x_vals;
                    msgJson["next_y"] = next_y_vals;


                    auto msg = "42[\"steer\"," + msgJson.dump() + "]";
                    // Latency
                    // The purpose is to mimic real driving conditions where
                    // the car does actuate the commands instantly.
                    //
                    // Feel free to play around with this value but should be to drive
                    // around the track with 100ms latency.
                    this_thread::sleep_for(chrono::milliseconds(LATENCY));
                    delay_predictor.set_previous_control_actuations(steer_value, throttle_value);
                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                }
            } else {
                // Manual driving
                std::string msg = "42[\"manual\",{}]";
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
        }
    });

    // We don't need this since we're not using HTTP but if it's removed the
    // program
    // doesn't compile :-(
    h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                       size_t, size_t) {
        const std::string s = "<h1>Hello world!</h1>";
        if (req.getUrl().valueLength == 1) {
            res->end(s.data(), s.length());
        } else {
            // i guess this should be done more gracefully?
            res->end(nullptr, 0);
        }
    });

    h.onConnection([&h, &delay_predictor](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
        std::cout << "Connected!!!" << std::endl;

        // Only set t0 for latency estimation once we've actually connected.
        delay_predictor.set_time();
    });

    h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                           char *message, size_t length) {
        ws.close();
        std::cout << "Disconnected" << std::endl;
    });

    int port = 4567;
    if (h.listen(port)) {
        std::cout << "Listening to port " << port << std::endl;
    } else {
        std::cerr << "Failed to listen to port" << std::endl;
        return -1;
    }
    h.run();
}
