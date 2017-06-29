#include <uWS/uWS.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <functional>
#include "json.hpp"
#include "FusionEKF.h"
#include "tools.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

// A simple command line parser. Taken from:
// https://stackoverflow.com/questions/865668/how-to-parse-command-line-arguments-in-c
class InputParser {
public:
  InputParser(int &argc, char **argv) 
  {
    for (auto i = 1; i < argc; ++i)
    {
      this->tokens.push_back(string(argv[i]));
    }
  }
  const string& getCmdOption(const string &option) const
  {
    auto itr = find(this->tokens.begin(), this->tokens.end(), option);

    if (itr != this->tokens.end() && ++itr != this->tokens.end())
    {
      return *itr;
    }

    static const string empty_string("");

    return empty_string;
  }
  
  bool cmdOptionExists(const string &option) const
  {
    return find(this->tokens.begin(), this->tokens.end(), option) != this->tokens.end();
  }

private:
  vector <string> tokens;
};

// This represents one line in the mearuement output file
class ResultOutput
{
  public:
    double p_x;     // estimated x
    double p_y;     // estimated y
    double v1;      // estimated velocity x
    double v2;      // estimated velocity y
    double m_px;    // measured x
    double m_py;    // measured y
    double x_gt;    // ground truth x
    double y_gt;    // ground truth y
    double vx_gt;   // ground truth velocity x
    double vy_gt;   // ground truth velocity y
    double rmse_x;  // RSME of x
    double rmse_y;  // RSME of y
    double rmse_vx; // RSME of vx
    double rmse_vy; // RSME of vy

    string toString() const
    {
      stringstream ss;
      ss << p_x
        << "\t" << p_y
        << "\t" << v1
        << "\t" << v2
        << "\t" << m_px
        << "\t" << m_py
        << "\t" << x_gt
        << "\t" << y_gt
        << "\t" << vx_gt
        << "\t" << vy_gt
        << "\t" << rmse_x
        << "\t" << rmse_y
        << "\t" << rmse_vx
        << "\t" << rmse_vy
        << endl;

      return ss.str();
    }
};

void printUsage()
{
  cout << "Usage: ExtendedKF [-f filname | -h]" << endl;
  cout << "CmdLine args description:" << endl;
  cout << "-f filename   Path to the output file" << endl;
  cout << "-h            Help description" << endl;
}

int main(int argc, char **argv)
{
  InputParser input(argc, argv);
  if (input.cmdOptionExists("-h")) 
  {
    printUsage();
    return 0;
  }
  const auto& filename = input.getCmdOption("-f");

  std::function<void(const ResultOutput&)> printOutput;
  fstream resultStream;

  if (!filename.empty()) {
    resultStream.open(filename.c_str(), fstream::out);

    printOutput = [&resultStream](const ResultOutput& result)
    {      
      resultStream << result.toString();
      resultStream.flush();
    };
  }
  else
  {
    printOutput = [](const ResultOutput& result) {;};
  }

  uWS::Hub h;

  // Create a Kalman Filter instance
  FusionEKF fusionEKF;

  // used to compute the RMSE later
  Tools tools;
  vector<VectorXd> estimations;
  vector<VectorXd> ground_truth;

  h.onMessage([&fusionEKF, &tools, &estimations, &ground_truth, &printOutput] (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event

    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data));
      if (s != "") {

        auto j = json::parse(s);

        std::string event = j[0].get<std::string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object

          string sensor_measurment = j[1]["sensor_measurement"];

          MeasurementPackage meas_package;
          istringstream iss(sensor_measurment);
          long long timestamp;

          // reads first element from the current line
          string sensor_type;
          iss >> sensor_type;

          if (sensor_type.compare("L") == 0) {
            meas_package.sensor_type_ = MeasurementPackage::LASER;
            meas_package.raw_measurements_ = VectorXd(2);
            float px;
            float py;
            iss >> px;
            iss >> py;
            meas_package.raw_measurements_ << px, py;
            iss >> timestamp;
            meas_package.timestamp_ = timestamp;
          }
          else if (sensor_type.compare("R") == 0) {
            meas_package.sensor_type_ = MeasurementPackage::RADAR;
            meas_package.raw_measurements_ = VectorXd(3);
            float ro;
            float theta;
            float ro_dot;
            iss >> ro;
            iss >> theta;
            iss >> ro_dot;
            meas_package.raw_measurements_ << ro, theta, ro_dot;
            iss >> timestamp;
            meas_package.timestamp_ = timestamp;
          }

          float x_gt;
          float y_gt;
          float vx_gt;
          float vy_gt;

          iss >> x_gt;
          iss >> y_gt;
          iss >> vx_gt;
          iss >> vy_gt;
          
          VectorXd gt_values(4);
          gt_values(0) = x_gt;
          gt_values(1) = y_gt;
          gt_values(2) = vx_gt;
          gt_values(3) = vy_gt;

          ground_truth.push_back(gt_values);

          //Call ProcessMeasurment(meas_package) for Kalman filter
          fusionEKF.ProcessMeasurement(meas_package);

          //Push the current estimated x,y positon from the Kalman filter's state vector
          VectorXd estimate(4);

          auto p_x = fusionEKF.ekf_.x_(0);
          auto p_y = fusionEKF.ekf_.x_(1);
          auto v1 = fusionEKF.ekf_.x_(2);
          auto v2 = fusionEKF.ekf_.x_(3);

          estimate(0) = p_x;
          estimate(1) = p_y;
          estimate(2) = v1;
          estimate(3) = v2;

          estimations.push_back(estimate);

          auto RMSE = tools.CalculateRMSE(estimations, ground_truth);

          json msgJson;
          msgJson["estimate_x"] = p_x;
          msgJson["estimate_y"] = p_y;
          msgJson["rmse_x"] = RMSE(0);
          msgJson["rmse_y"] = RMSE(1);
          msgJson["rmse_vx"] = RMSE(2);
          msgJson["rmse_vy"] = RMSE(3);
          auto msg = "42[\"estimate_marker\"," + msgJson.dump() + "]";
          
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

          //Output file format :
          //est_px est_py est_vx est_vy meas_px meas_py gt_px gt_py gt_vx gt_vy rmse_x rmse_y rmse_vx rmse_vy
          auto currentMeasurement = meas_package.currentMeasurement();
          auto m_px = currentMeasurement[0];
          auto m_py = currentMeasurement[1];

          ResultOutput result;
          result.p_x = p_x;
          result.p_y = p_y;
          result.v1 = v1;
          result.v2 = v2;
          result.m_px = m_px;
          result.m_py = m_py;
          result.x_gt = x_gt;
          result.y_gt = y_gt;
          result.vx_gt = vx_gt;
          result.vy_gt = vy_gt;
          result.rmse_x = RMSE[0];
          result.rmse_y = RMSE[1];
          result.rmse_vx = RMSE[2];
          result.rmse_vy = RMSE[3];
          
          printOutput(result);
        }
      }
      else {

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
    std::cout << "Connected" << std::endl;
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
