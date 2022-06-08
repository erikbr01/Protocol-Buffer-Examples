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

  std::string log;
  // FastDDS default participant
  std::unique_ptr<DefaultParticipant> dp =
      std::make_unique<DefaultParticipant>(0, "raptor_vision");
  Quad quad("Drone", &log, dp, "mocap_srl_raptor", "pos_cmd");
  Item box("Box", dp, "mocap_srl_vision_bottle");
  //   Item quad("Cam", dp, "mocap_srl_realsense");

  DDSPublisher pub = DDSPublisher(idl_msg::Mocap_msgPubSubType(),
                                  "vision_srl_box", dp->participant());

  float camera_x_trans = 0.0f;
  float camera_y_trans = 0.0f;
  float camera_z_trans = 0.0f;

  float camera_roll = 0.0f;
  float camera_pitch = 0.0f;
  float camera_yaw = 0.0f;

  std::vector<float> camera_translations{camera_x_trans, camera_y_trans,
                                         camera_z_trans};

  std::vector<float> camera_orientation{camera_roll, camera_pitch, camera_yaw};

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
            ","
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
         << "t"
         << "\n";

  while (true) {
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
    std::cout << quad_position.at(0) - item_position.at(0) << std::endl;
    std::cout << quad_position.at(1) - item_position.at(1) << std::endl;
    std::cout << quad_position.at(2) - item_position.at(2) << std::endl;

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
    std::vector<float> point_global{det.x(), det.y(), det.z()};
    // // Rotate the point to the drone frame
    // std::vector<float> point_drone_r =
    //     util::apply_euler_frame_rotation(point_cam, camera_orientation);

    // // Apply the translation in the drone frame
    // std::vector<float> point_drone_rt =
    //     util::apply_frame_translation(point_drone_r, camera_translations);

    // // Now, convert this into a global frame
    // // Apply rotation into global frame
    // std::vector<float> point_global_r =
    //     util::apply_euler_frame_rotation(point_drone_rt, quad_orientation);

    // // Apply translation into global frame
    // std::vector<float> point_global_rt =
    //     util::apply_frame_translation(point_global_r, quad_position);

//     std::cout << "POINT IN TRANS/ROT COORDINATES ------------" << std::endl;
//     std::cout << point_global.at(0) << std::endl;
//     std::cout << point_global.at(1) << std::endl;
//     std::cout << point_global.at(2) << std::endl;

    float errx = item_position.at(0) - point_global.at(0);
    float erry = item_position.at(1) - point_global.at(1);
    float errz = item_position.at(2) - point_global.at(2);

    std::cout << "ERROR ------------" << std::endl;
    std::cout << errx << std::endl;
    std::cout << erry << std::endl;
    std::cout << errz << std::endl;

    output << point_global.at(0) << "," << point_global.at(1) << ","
           << point_global.at(2) << ",";
    output << item_position.at(0) << "," << item_position.at(1) << ","
           << item_position.at(2) << ",";
    output << quad_position.at(0) << "," << quad_position.at(1) << ","
           << quad_position.at(2) << ",";
    output << quad_orientation.at(0) << "," << quad_orientation.at(1) << ","
           << quad_orientation.at(2) << ",";
    output << errx << "," << erry << "," << errz << ",";

    std::time_t ms = std::time(nullptr);
    output << ms << "\n";

//     cpp_msg::Mocap_msg mocap;
//     mocap.position.x = point_global.at(0);
//     mocap.position.y = point_global.at(1);
//     mocap.position.z = point_global.at(2);
//     pub.publish(mocap);
  }
  output.close();
  return 0;
}
