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

#define poly_order 3

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


void print_vector(vector<double> v, std::string name) {
    std::cout << name << " = [";
    for(auto x : v) {
        std::cout << x << ", ";
    }
    std::cout << "]" << std::endl;
}

void print_matrix(Eigen::MatrixXd m, std::string name) {
    std::cout << name << " = [" << endl;
    for(unsigned int i=0; i<m.rows(); i++) {
        for(unsigned int j=0; j<m.cols(); j++) {
            std::cout << m(i, j) << ", ";
        }
        std::cout << std::endl;
    }
    std::cout << "]" << std::endl;
}


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





int main() {
    uWS::Hub h;

    // MPC is initialized here!
    MPC mpc;

    h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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
                    // j[1] is the data JSON object
                    vector<double> ptsx = j[1]["ptsx"];
                    vector<double> ptsy = j[1]["ptsy"];
                    double px = j[1]["x"];
                    double py = j[1]["y"];
                    double psi = j[1]["psi"];
                    double v = j[1]["speed"];

                    Transformation_Matrix vehicle2map(psi, px, py);
                    Transformation_Matrix map2vehicle = vehicle2map.inverse();

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

                    // Fit a polynomial to the above x and y coordinates
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
                    double throttle_value = max(vars[7], 0.0);

                    json msgJson;
                    // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
                    // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
                    msgJson["steering_angle"] = steer_value;
                    msgJson["throttle"] = throttle_value;

                    //Display the MPC predicted trajectory
                    //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
                    // the points in the simulator are connected by a green line
                    vector<double> mpc_x_vals = result.path.x;
                    vector<double> mpc_y_vals = result.path.y;
//                    vector<double> mpc_x_vals = result.fit.x;
//                    vector<double> mpc_y_vals = result.fit.y;
                    msgJson["mpc_x"] = mpc_x_vals;
                    msgJson["mpc_y"] = mpc_y_vals;

                    //Display the waypoints/reference line
                    //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
                    // the points in the simulator are connected by a yellow line
                    vector<double> next_x_vals = result.fit.x;
                    vector<double> next_y_vals = result.fit.y;
//                    vector<double> next_x_vals = ptsx_vehicle;
//                    vector<double> next_y_vals = ptsy_vehicle;
                    msgJson["next_x"] = next_x_vals;
                    msgJson["next_y"] = next_y_vals;


                    auto msg = "42[\"steer\"," + msgJson.dump() + "]";
                    // Latency
                    // The purpose is to mimic real driving conditions where
                    // the car does actuate the commands instantly.
                    //
                    // Feel free to play around with this value but should be to drive
                    // around the track with 100ms latency.
                    //
                    // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
                    // SUBMITTING.
                    this_thread::sleep_for(chrono::milliseconds(0));
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

    h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
        std::cout << "Connected!!!" << std::endl;
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
