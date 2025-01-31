#include <chrono>
#include <cmath>
#include <fstream>
#include <google/protobuf/util/time_util.h>
#include <iostream>
#include <string>
#include <ctime>

#include "detection_msg.pb.h"

#include "frame_conversions.h"
#include <zmq.hpp>

#include "Mocap_msg.h"
#include "Mocap_msgPubSubTypes.h"
#include "domain_participant.h"
#include "publisher.h"

#include "Item.h"
#include "Quad.h"

int main() {
  long frame_id = 0;
  std::string log;
  // FastDDS default participant
  std::unique_ptr<DefaultParticipant> dp =
      std::make_unique<DefaultParticipant>(0, "raptor_vision");
  Item quad("Drone", dp, "mocap_srl_raptor_multi");
  Item box("Box", dp, "mocap_srl_box");
  //   Item quad("Cam", dp, "mocap_srl_realsense");

  DDSPublisher pub = DDSPublisher(idl_msg::Mocap_msgPubSubType(),
                                  "vision_srl_box", dp->participant());

 

  //  Prepare our context and socket
  zmq::context_t context(1);
  zmq::socket_t socket(context, ZMQ_REQ);
  socket.bind("tcp://*:2222");

  std::ofstream output;
  std::time_t timestamp =
      std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
  std::string date = std::ctime(&timestamp);

  output.open("logs/" + date + ".csv");
  output << "vision_x"
         << ","
         << "vision_y"
         << ","
         << "vision_z"
         << ","
         << "mocap_x"
         << ","
         << "mocap_y"
         << ","
         << "mocap_z"
         << ","
         << "quad_x"
         << ","
         << "quad_y"
         << ","
         << "quad_z"
         << ","
         << "quad_roll"
         <<  ","
         << "quad_pitch"
         << ","
         << "quad_yaw"
         << ","
         << "error_x"
         << ","
         << "error_y"
         << ","
         << "error_z"
         << ","
         << "trans_x"
         << ","
         << "trans_y"
         << ","
         << "trans_z"
         << ","
         << "t"
         << "\n";

  cpp_msg::Mocap_msg last_known_position;
  last_known_position.position.x = 0.0;
  last_known_position.position.y = 0.0;
  last_known_position.position.z = 0.0;
  last_known_position.occluded = 0;
  
  while (true) {
    std::cout << frame_id << std::endl;
    frame_id++;
    //  Send initial message that detection can move on
    // Save position of the drone - this should be synced up with the detection
    // data from the camera
    std::vector<float> quad_position;
    std::vector<float> quad_orientation;
    cpp_msg::Mocap_msg quad_pose = quad.getPose();

    cpp_msg::Mocap_msg item_pose = box.getPose();
    std::vector<float> item_position;

    // std::cout << "got quad pose from mocap" << std::endl;

    quad_position.push_back(quad_pose.position.x);
    quad_position.push_back(quad_pose.position.y);
    quad_position.push_back(quad_pose.position.z);

//     std::cout << "Quad position:" << std::endl;
//     std::cout << quad_position.at(0) << std::endl;
//     std::cout << quad_position.at(1) << std::endl;
//     std::cout << quad_position.at(2) << std::endl;

//     std::cout << "Box position:" << std::endl;
//     std::cout << box.getPose().position.x << std::endl;
//     std::cout << box.getPose().position.y << std::endl;
//     std::cout << box.getPose().position.z << std::endl;

    quad_orientation.push_back(quad_pose.orientation.roll * M_PI / 180.0f);
    quad_orientation.push_back(quad_pose.orientation.pitch * M_PI / 180.0f);
    quad_orientation.push_back(quad_pose.orientation.yaw * M_PI / 180.0f);


    item_position.push_back(item_pose.position.x);
    item_position.push_back(item_pose.position.y);
    item_position.push_back(item_pose.position.z);

//     std::cout << "Quad orientation:" << std::endl;
//     std::cout << quad_orientation.at(0) << std::endl;
//     std::cout << quad_orientation.at(1) << std::endl;
//     std::cout << quad_orientation.at(2) << std::endl;

    std::cout << "Translation difference:" << std::endl;
    float trans_x = quad_position.at(0) - item_position.at(0);
    float trans_y = quad_position.at(1) - item_position.at(1);
    float trans_z = quad_position.at(2) - item_position.at(2);
    std::cout << trans_x << std::endl;
    std::cout << trans_y << std::endl;
    std::cout << trans_z << std::endl;
    std::cout << quad_pose.orientation.yaw << std::endl;

    Vision::Detection pose;
    pose.set_x(quad_position.at(0));
    pose.set_y(quad_position.at(1));
    pose.set_z(quad_position.at(2));
    pose.set_roll(quad_orientation.at(0));
    pose.set_pitch(quad_orientation.at(1));
    pose.set_yaw(quad_orientation.at(2));

    std::string msg_string;
    pose.SerializeToString(&msg_string);
    zmq::message_t request(msg_string.size());
    memcpy((void *)request.data(), msg_string.c_str(), msg_string.size());
    socket.send(request, zmq::send_flags::none);
    auto res = socket.recv(request, zmq::recv_flags::none);

    // Receive answer with detection data
    // auto res = socket.recv(request, zmq::recv_flags::none);

    // Deserialize protobuf message
    Vision::Detection det;
    if (!det.ParseFromString(request.to_string())) {
      std::cerr << "Failed to parse protobuf message." << std::endl;
      return -1;
    }

    // std::cout << det.DebugString() << std::endl;

    // Frame conversions - we need to go from camera frame to drone frame and
    // then from drone frame to global frame

    // Point in camera frame - this is what we're getting back from the camera
   

    if(det.label() == "closing") {
      output.close();
      std::cout << "closing process" << std::endl;
      break;
    }
    
    std::vector<float> point_global{det.x(), det.y(), det.z()};

    float errx = item_position.at(0) - point_global.at(0);
    float erry = item_position.at(1) - point_global.at(1);
    float errz = item_position.at(2) - point_global.at(2);
    std::time_t ms = std::time(nullptr);
    
    if(det.label() != "Nothing") {


      std::cout << "ERROR ------------" << std::endl;
      std::cout << errx << std::endl;
      std::cout << erry << std::endl;
      std::cout << errz << std::endl;
    }

    output << point_global.at(0) << "," << point_global.at(1) << ","
           << point_global.at(2) << ",";
    output << item_position.at(0) << "," << item_position.at(1) << ","
           << item_position.at(2) << ",";
    output << quad_position.at(0) << "," << quad_position.at(1) << ","
           << quad_position.at(2) << ",";
    output << quad_orientation.at(0) << "," << quad_orientation.at(1) << ","
           << quad_orientation.at(2) << ",";
    output << errx << "," << erry << "," << errz << ",";
    output << trans_x << "," << trans_y << "," << trans_z << ",";

    output << ms << "\n";


    cpp_msg::Mocap_msg mocap;
    mocap.position.x = point_global.at(0);
    mocap.position.y = point_global.at(1);
    mocap.position.z = point_global.at(2);
    mocap.occluded = 0;

    if(det.label() == "Nothing") {
      std::cout << "received nothing, publishing position" << std::endl;
      std::cout << last_known_position.position.x << "\t" << last_known_position.position.y << "\t" << last_known_position.position.z << "\t" << std::endl;;
      pub.publish(last_known_position);
      continue;
    }
    
    std::cout << "publishing position" << std::endl;
    std::cout << mocap.position.x << "\t" << mocap.position.y << "\t" << mocap.position.z << "\t" << std::endl;;
    pub.publish(mocap);
    last_known_position = mocap;
  }
  output.close();
  return 0;
}
