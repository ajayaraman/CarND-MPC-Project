#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

// for convenience
using json = nlohmann::json;

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

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
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
    cout << sdata << endl;
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

          //get previously applied controls from simulator
          double delta = j[1]["steering_angle"];
          double acceleration = j[1]["throttle"];

          /*
          * TODO: Calculate steering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */
          Eigen::VectorXd ptsxv(ptsx.size());
          Eigen::VectorXd ptsyv(ptsy.size());
          
          // transform points to vehicle coordinate system
          for (int i=0; i < ptsx.size(); i++) {
            ptsxv(i) = (ptsx[i] - px) * cos(-psi) - (ptsy[i] - py) * sin(-psi);
            ptsyv(i) = (ptsx[i] - px) * sin(-psi) + (ptsy[i] - py) * cos(-psi);
          }

          //Fit 3rd order polynomial to desired waypoints
          auto coeffs = polyfit(ptsxv, ptsyv, 3);

          //Initialize MPC State
          Eigen::VectorXd state(6);

          //Calculate initial cross track error
          //Since polynomial is w.r.t car, we calculate cte at x = 0 in car coordinates 
          double cte = polyeval(coeffs, 0) - 0;
          //derivative is
          // epsi = psi - atan( coeffs[1] + 2 * coeffs[2] * px + 3 * coeffs[3] * px * px )
          //But since px = 0 at start, we simplify it to epsi = atan(coeffs[1])
          double epsi = 0 - atan(coeffs[1]);
          state << 0, 0, 0, v, cte, epsi;

          const double Lf = 2.67;

          //Predict using kinematics where the vehicle will be if the current steering controls are applied
          double latency = 0.1; //100 ms
          state(0) += v * cos(state(2)) * latency; //x
          state(1) += v * sin(state(2)) * latency; //y
          state(2) += -v * delta / Lf * latency;    //psi
          state(3) += acceleration * latency;      //v
          state(4) += v * sin(epsi) * latency;     //cte
          state(5) += -v * delta / Lf * latency;    //epsi

          //Solve for optimal controls
          auto vars = mpc.Solve(state, coeffs);

          double steer_value = vars[0] / (deg2rad(25) * Lf);
          double throttle_value = vars[1];

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory 
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line
          
          //start from i = 2 because the first two values returned by mpc.Solve are the controls we must apply
          for (int i = 2; i < vars.size(); i++)
          {
            // x coordinates are in even indexes starting from 0
            if( i %2 == 0)
            {
              mpc_x_vals.push_back(vars[i]);
            }
            else
            {
              mpc_y_vals.push_back(vars[i]);
            }
          }

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          //In 2 m increments, we plot 30 points
          double xinc = 2;
          const int nPts = 30;
          for(int i = 0; i < nPts; i++)
          {
            double x = xinc * i;
            next_x_vals.push_back(x);
            next_y_vals.push_back(polyeval(coeffs, x));
          }

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(100));
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
