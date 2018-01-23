#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>

#define MAX_FRAME_NUM 1600
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
  //double best_err = 0;
  pid.param[0] = 0.379757;//0.12;
  pid.param[1] = 3.01556;
  pid.param[2] = 0.00401556;
  pid.dparam[0] = 0.1;
  pid.dparam[1] = 1.0;
  pid.dparam[2] = 0.001;
  pid.dparam_change_pattern[0] = 1;
  pid.dparam_change_pattern[1] = -2;
  pid.dparam_change_pattern[2] = 1;
  //, 3.0, 0.004};
  //double dparam[3] = [0,0,0];

  // TODO: Initialize the pid variable.
  pid.Init(pid.param[0], pid.param[1], pid.param[2]);

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
          //std::string data = j[1].get<std::string>();
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value;
          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */

          // DEBUG
          //std::cout << "        [Debug] cte: " << cte << " ,speed: " << speed << " ,angle: "  << angle << std::endl;
          pid.UpdateError(cte);
          steer_value = pid.TotalError();

          if (pid.total_frame < MAX_FRAME_NUM){
                pid.total_frame += 1;
          }
          else{
                pid.total_frame = 0;
                //update param with dparam
                // pattern {1,-2,1}

                std::cout << " best_err: " << pid.best_err << " ,twiddle_error: " << pid.twiddle_error<< std::endl;
                if (pid.best_err == 0){
                        pid.best_err = pid.twiddle_error;
                }else{
                        if (pid.twiddle_error < pid.best_err){
                                //Better
                                pid.best_err = pid.twiddle_error;
                                //dp[i] *= 1.1
                                pid.param_index = 0;
                                pid.change_pattern_index = 0;
                                pid.dparam[pid.param_index] *= 1.1;
                        }
                        else{
                                // Move param pattern
                                //p[i] += pattern * dp[i]
                                // last pattern
                                //dp[i] *= 0.9
                                pid.dparam[pid.param_index] *= 0.9;
                        }
                }
                pid.param[pid.param_index] += pid.dparam_change_pattern[pid.change_pattern_index] * pid.dparam[pid.param_index];
                pid.change_pattern_index +=1;
                if (pid.change_pattern_index > 2){
                        pid.change_pattern_index = 0;
                        pid.param_index +=1;
                        if (pid.param_index > 2){
                                pid.param_index = 0;
                        }
                }
                pid.Init(pid.param[0], pid.param[1], pid.param[2]);
          }
          //std::cout << "        [Debug] CTE: " << cte << " Steering Value: " << steer_value << std::endl;

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
