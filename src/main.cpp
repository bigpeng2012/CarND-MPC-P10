#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <string>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "MPC.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }


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
    std::cout << sdata << std::endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {

			// Extract location data and guide points(path way points),
			// converting from global to vehicle coordinates.
			vector<double> ptsx = j[1]["ptsx"]; //way points
			vector<double> ptsy = j[1]["ptsy"]; // global coordinates
			const int n = ptsx.size();
			double px = j[1]["x"];
			double py = j[1]["y"];
			double psi = j[1]["psi"];

			Eigen::VectorXd pts_x(n);  // vehicle coordinates
			Eigen::VectorXd pts_y(n);
			for (int i=0; i < n; i++) {
				float dx = ptsx[i] - px;
				float dy = ptsy[i] - py;
				float r = sqrt(dx*dx + dy*dy); // distance r to the point from vehicle
				float beta = atan2(-dy, dx);  // angle of vector in global space
				float alpha = beta+psi;       // angle of vector in vehicle space

				pts_x[i] = r*cos(alpha);      // x, vehicle space
				pts_y[i] = -r*sin(alpha);     // y, vehicle space
			}

			// Extract dynamic information and create our state
			// vector.  Account for latency by advancing the car along the current
			// speed and turning angle.  Compute cross-track error and our
			// desired turning angle.
			Eigen::VectorXd state(6);
			double v = j[1]["speed"];
			double rho = j[1]["steering_angle"];
			auto coeffs = polyfit(pts_x, pts_y, 2);   // Fit a quadratic guide wire

			// Account for latency
			const double latency = 0.1;  // 100 ms
			const double Lf = 2.67;
			px = v * latency;
			psi = - v * rho / Lf * latency ;

			// Compute cross track error as the y where we're supposed to be,
			// in local coordinates, minus our current y, 0, or the origin.
			double cte = polyeval(coeffs, px);

			// Estimate our desired steering angle as the angle of a line
			// tangent to the start of our guideware curve.
			double epsi = -atan(coeffs[1]);

			// Store everything in an initial state, which we'll then
			// use a model to project forward in time.
			state << px, 0, psi, v, cte, epsi;

			// Choose our throttle and steering angle by finding an optimal
			// number of settings over the next few seconds, based on simulating
			// our vehicle and how it will likely move forward kinematically.
			auto vars = mpc.Solve(state, coeffs);

			// Store our actuators in JSON to fire them off via a websocket
			// back into the simulator engine.
			json msgJson;
			msgJson["throttle"] = vars[0];
			msgJson["steering_angle"] = -vars[1]/deg2rad(25.0);

			// Add our predicted trajectory, in green
			const int npts = (int) vars[2];
			vector<double> mpc_x_vals(npts);
			vector<double> mpc_y_vals(npts);

			// Points are in reference to the vehicle's coordinate system
			// the points in the simulator are connected by a Green line
			for (int i=0; i < npts; i++) {
				mpc_x_vals[i] = vars[i*2+3];
				mpc_y_vals[i] = vars[i*2+4];
				//
				// Uncomment this line to see our guidewire instead of our
				// predicted path forward.
				//
				//mpc_y_vals[i] = polyeval(coeffs, pts_x[i]);
			}

			msgJson["mpc_x"] = mpc_x_vals;
			msgJson["mpc_y"] = mpc_y_vals;

			// Display the waypoints/reference line in yellow
			vector<double> next_x_vals(n);
			vector<double> next_y_vals(n);
			for (int i=0; i < n; i++) {
				double x = pts_x[i];
				next_x_vals[i] = x;
				next_y_vals[i] = pts_y[i];
			}
			msgJson["next_x"] = next_x_vals;
			msgJson["next_y"] = next_y_vals;


			auto msg = "42[\"steer\"," + msgJson.dump() + "]";
			//std::cout << "Sending " << msg << std::endl;
			// Latency
			// The purpose is to mimic real driving conditions where
			// the car does actuate the commands instantly.
			//
			// Feel free to play around with this value but should be to drive
			// around the track with 100ms latency.
			//
			// NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
			// SUBMITTING.
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
			ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
		}

      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

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