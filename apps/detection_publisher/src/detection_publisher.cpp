#include <cmath>
#include <fstream>
#include <google/protobuf/util/time_util.h>
#include <iostream>
#include <string>

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
  Quad quad("Drone", &log, dp, "mocap_srl_realsense", "pos_cmd");
  Item box("Box", dp, "mocap_srl_box");
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
  socket.bind("tcp://*:5555");

  while (true) {
    //  Send initial message that detection can move on
    // Save position of the drone - this should be synced up with the detection
    // data from the camera
    std::vector<float> quad_position;
    std::vector<float> quad_orientation;
    cpp_msg::Mocap_msg quad_pose = quad.getPose();

    cpp_msg::Mocap_msg item_pose = box.getPose();
    std::vector<float> item_position;

    std::cout << "got quad pose from mocap" << std::endl;

    quad_position.push_back(quad_pose.position.x);
    quad_position.push_back(quad_pose.position.y);
    quad_position.push_back(quad_pose.position.z);

    std::cout << "Quad position:" << std::endl;
    std::cout << quad_position.at(0) << std::endl;
    std::cout << quad_position.at(1) << std::endl;
    std::cout << quad_position.at(2) << std::endl;

    std::cout << "Box position:" << std::endl;
    std::cout << box.getPose().position.x << std::endl;
    std::cout << box.getPose().position.y << std::endl;
    std::cout << box.getPose().position.z << std::endl;

    quad_orientation.push_back(quad_pose.orientation.roll * M_PI / 180.0f);
    quad_orientation.push_back(quad_pose.orientation.pitch * M_PI / 180.0f);
    quad_orientation.push_back(quad_pose.orientation.yaw * M_PI / 180.0f);

    std::cout << "Quad orientation:" << std::endl;
    std::cout << quad_orientation.at(0) << std::endl;
    std::cout << quad_orientation.at(1) << std::endl;
    std::cout << quad_orientation.at(2) << std::endl;

    item_position.push_back(item_pose.position.x);
    item_position.push_back(item_pose.position.y);
    item_position.push_back(item_pose.position.z);

    // Send request for detection data
    zmq::message_t request;
    socket.send(zmq::str_buffer("ok"), zmq::send_flags::none);

    // Receive answer with detection data
    auto res = socket.recv(request, zmq::recv_flags::none);

    // Deserialize protobuf message
    Vision::Detection det;
    if (!det.ParseFromString(request.to_string())) {
      std::cerr << "Failed to parse protobuf message." << std::endl;
      return -1;
    }

    std::cout << det.DebugString() << std::endl;

    // Frame conversions - we need to go from camera frame to drone frame and
    // then from drone frame to global frame

    // Point in camera frame - this is what we're getting back from the camera
    std::vector<float> point_cam{det.x(), det.y(), det.z()};

    // Rotate the point to the drone frame
    std::vector<float> point_drone_r =
        util::apply_euler_frame_rotation(point_cam, camera_orientation);

    // Apply the translation in the drone frame
    std::vector<float> point_drone_rt =
        util::apply_frame_translation(point_drone_r, camera_translations);

    // Now, convert this into a global frame
    // Apply rotation into global frame
    std::vector<float> point_global_r =
        util::apply_euler_frame_rotation(point_drone_rt, quad_orientation);

    // Apply translation into global frame
    std::vector<float> point_global_rt =
        util::apply_frame_translation(point_global_r, quad_position);

    std::cout << "POINT IN TRANS/ROT COORDINATES ------------" << std::endl;
    std::cout << point_global_rt.at(0) << std::endl;
    std::cout << point_global_rt.at(1) << std::endl;
    std::cout << point_global_rt.at(2) << std::endl;

    std::cout << "ERROR ------------" << std::endl;
    std::cout << item_position.at(0) - point_global_rt.at(0) << std::endl;
    std::cout << item_position.at(1) - point_global_rt.at(1) << std::endl;
    std::cout << item_position.at(2) - point_global_rt.at(2) << std::endl;

    // cpp_msg::Mocap_msg mocap;
    // mocap.position.x = point_global_rt.at(0);
    // mocap.position.y = point_global_rt.at(1);
    // mocap.position.z = point_global_rt.at(2);
    // pub.publish(mocap);
  }
  return 0;
}
