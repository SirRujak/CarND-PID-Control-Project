#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main()
{
  uWS::Hub h;

  PID pid;
  // Initialize the pid variable.
  /* These are values the tiddle algorithm arrived at.
   * Unfortunately they tended towards far too high Kd values.
   * Through multiple runs of the twiddle algorithm the following
   * PID values were found:
        1.21018, 6.26384, 0.0172493
        1.16, 6.36, 0.013
        1.21018, 8.46385, 0.0172493
        3.63433, 28.8131, 0.0584075
        2.0, 10.0, 0.04
        1.43433, 14.7539, 0.00620688
  */
  /*  After attempting twiddling without a steady solution
   *  manual alteration was used to find the following:

      1.43433, 0.0002, 8.5
      1.43433, 0.0002, 5.5
      1.43433, -0.0002, 8.5
      1.43433, 0.00002, 8.5
      1.43433, 0.000002, 8.5
      1.43433, 0.000002, 15.5
      1.43433, 0.02, 5.5
      1.43433, 0.2, 8.5
      1.43433, 0.1, 8.5
      2.43433, 0.1, 8.5
      2.0, 0.05, 8.5
      2.0, 0.05, 10.5
      0.025, 0.0, 0.0
      0.025, 0.0001, 0.0
      0.025, 0.0001, 2.9

      0.025, 0.0004, 2.9

      0.125, 0.00025, 1.9
  */

  // The final PID values.
  pid.Init(0.14, 0.0004, 1.5);

  // This is to activate or deactivate twiddling.
  // Set pid.twiddle_done to false to start twiddling.
  pid.twiddle_done = true;
  pid.use_d = true;
  pid.use_i = true;

  h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {

    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value;
          /*
          * Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */
          pid.UpdateError(cte);
          steer_value = pid.TotalError();

          if (!pid.twiddle_done) {
            if (pid.twiddle_counter > pid.twiddle_skip_first_max) {
              pid.twiddle_error += cte * cte + std::abs(steer_value);
            }
            if (pid.twiddle_error > pid.twiddle_best_error) {
              pid.twiddle_counter = pid.twiddle_max + 1;
            }
            pid.twiddle_counter += 1;
            std::cout << "Twiddle counter: " << pid.twiddle_counter << std::endl;
            if (pid.twiddle_counter > pid.twiddle_max) {
              // We have completed a round.
              pid.twiddle_max = 2000;
              if (pid.twiddle_error < pid.twiddle_best_error){
                // If our last twiddle gave better results.
                pid.twiddle_back_calc = false;
                pid.twiddle_best_error = pid.twiddle_error;
                pid.twiddle_dp_values[pid.twiddle_current_index] *= 1.2;
              } else {
                if (!pid.twiddle_back_calc) {
                  // We are on the forwards pass.
                  pid.twiddle_p_values[pid.twiddle_current_index] -= 2.0 *
                    pid.twiddle_dp_values[pid.twiddle_current_index];
                  pid.twiddle_back_calc = true;
                } else {
                  pid.twiddle_p_values[pid.twiddle_current_index] +=
                    pid.twiddle_dp_values[pid.twiddle_current_index];
                  pid.twiddle_dp_values[pid.twiddle_current_index] *= 0.85;
                  pid.twiddle_back_calc = false;
                }
              }

              // Move to the next p value to twiddle.
              if (!pid.twiddle_back_calc) {
                pid.twiddle_current_index += 1;
                if (pid.twiddle_current_index > 2) {
                  pid.twiddle_current_index = 0;
                }
                // Add the current dp value to start the next round of tests.
                pid.twiddle_p_values[pid.twiddle_current_index] +=
                  pid.twiddle_dp_values[pid.twiddle_current_index];
              }
              pid.i_error = 0;
              pid.twiddle_counter = 0;
              pid.twiddle_skip_counter = 0;
              pid.twiddle_past_skip = false;
              pid.twiddle_error = 0;
              pid.initialized = false;
              pid.current_sum = pid.twiddle_dp_values[0] +
                                   pid.twiddle_dp_values[1] +
                                   pid.twiddle_dp_values[2];
              if (pid.current_sum < 0.002) {
                  pid.twiddle_done = true;
                }
              pid.Kp = pid.twiddle_p_values[0];
              pid.Kd = pid.twiddle_p_values[1];
              pid.Ki = pid.twiddle_p_values[2];
              std::string reset_msg = "42[\"reset\",{}]";
              ws.send(reset_msg.data(),reset_msg.length(), uWS::OpCode::TEXT);
            }
          }

          /*
          // DEBUG
          std::cout << "On back calc? " << pid.twiddle_back_calc << std::endl;
          std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;
          std::cout << "Current PD Sum: " << pid.current_sum << std::endl;
          std::cout << "P: [" << pid.twiddle_p_values[0] << ", " <<
                       pid.twiddle_p_values[1] << ", " <<
                       pid.twiddle_p_values[2] << "]" << std::endl;
          std::cout << pid.Kp << ", " << pid.Kd << ", " << pid.Ki << std::endl;
          std::cout << "DP: [" << pid.twiddle_dp_values[0] << ", " <<
                       pid.twiddle_dp_values[1] << ", " <<
                       pid.twiddle_dp_values[2] << "]" << std::endl;
          std::cout << "Error: " << pid.i_error << std::endl;
          std::cout << "Twiddle Error: " << pid.twiddle_error << std::endl;
          std::cout << "Twiddle Best: " << pid.twiddle_best_error << std::endl;
          */
          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = 0.3;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
