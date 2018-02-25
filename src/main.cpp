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

/*
	Init
*/

	PID pid;
	// TODO: Initialize the pid variable.
	//pid.Init(0.8, 7, 0.00001);

	//pid.Init(0.8, 7, 0.00001);

	//pid.Init(1.0, 30.0, 0.00001); // yes

	pid.Init(1.0, 20.0, 0.00001); // yes

	//

	// Ok The car can complete the track.
	//pid.Init(29.6951, 32.7596, 0.000457);

	//pid.Init(1.2, 5.5, 0.000457);

	pid.Init_PD(0.5, 0.01) ;

	pid.counter = -1; // -1
	pid.N_it = 100;

	pid.control_K = 0;
	pid.control_stage = 0;
	pid.previous_time = 0;

	pid.loop_number = 0;

	std::clock_t begin_time = std::clock();
	pid.previous_time = begin_time;

	

	pid.dK = VectorXd(3);
	//pid.dK << 0.2, 0.2, 0.0001;

	pid.dK << 0.2, 0.2, 0.0001;
	pid.sum_dK = pid.dK(0)+pid.dK(1)+pid.dK(2);

	h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      	auto s = hasData(std::string(data).substr(0, length));
		if (s != "") 
		{
			auto j = json::parse(s);
			std::string event = j[0].get<std::string>();
			if (event == "telemetry") 
        	{
        		//std::clock_t begin_time = std::clock();
				
				

				// j[1] is the data JSON object
				double cte = std::stod(j[1]["cte"].get<std::string>());
				double speed = std::stod(j[1]["speed"].get<std::string>());
				double angle = std::stod(j[1]["steering_angle"].get<std::string>());
				double steer_value;
				double throttle;
				/*
				* TODO: Calcuate steering value here, remember the steering value is
				* [-1, 1]. - 25 degrees to 25 degrees
				* NOTE: Feel free to play around with the throttle and speed. Maybe use
				* another PID controller to control the speed!
				*/

				std::cout << " " << std::endl;
				std::cout << " " << std::endl;



				/*
					Time
				*/
				double dt;
				dt = 1;

				/*
				dt = (std::clock() - pid.previous_time ) / double( CLOCKS_PER_SEC);	// 0 or 0.015625???
				pid.previous_time = std::clock();
				*/
				


				// UpdateError
				pid.UpdateError(cte, dt);

				// CONTROLLER RESET
				
				// Reset if the error is bigger than the threshold

				pid.MyCounter(cte);
				std::cout << " Counter:  " << pid.counter << std::endl;

				std::cout<< "sum_dK: " << pid.sum_dK << std::endl;


				// 	AUTOMATIC TUNING
				/*

				if (pid.counter == (pid.N_it - 1)) //
				{
					std::cout << fabs(pid.current_cte_avg) << std::endl;

					if (fabs(pid.current_cte_avg) > 0.08 && pid.sum_dK > 0.01) ///
					{	
						pid.Twiddle();
						pid.Init(pid.Kp, pid.Kd, pid.Ki);
					} 

				}
				*/
				

				// STEERING VALUE

				// steering value
				steer_value = pid.TotalError();


				// SPEED CONTROL


				// error = target_value - sensor_reading;
				double error_speed = 15 - speed;

				//
				pid.UpdateErrorSpeed(error_speed, dt);

				throttle = pid.TotalError_throttle();

				if (fabs(fabs(steer_value) > 0.16) && pid.loop_number > 50)
					throttle = 0;

				// DEBUG
				std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;
				std::cout << "Previous Average CTE: " << pid.previous_cte_avg << std::endl;
				std::cout << "Current Average CTE: " << pid.current_cte_avg << std::endl;

				std::cout << "Kp: " << pid.Kp << std::endl;
				std::cout << "Kd: " << pid.Kd << std::endl;
				std::cout << "Ki: " << pid.Ki << std::endl;

				if (pid.counter < (pid.N_it - 1))
					pid.counter++;
				else
					pid.counter =0;


				std::cout << " " << std::endl;
				std::cout << " " << std::endl;

				json msgJson;
				msgJson["steering_angle"] = steer_value;
				msgJson["throttle"] = throttle;
				auto msg = "42[\"steer\"," + msgJson.dump() + "]";
				std::cout << msg << std::endl;
				ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        	}

  		} 
  		else 
  		{
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
