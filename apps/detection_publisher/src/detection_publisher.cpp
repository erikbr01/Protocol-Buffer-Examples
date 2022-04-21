#include <fstream>
#include <google/protobuf/util/time_util.h>
#include <iostream>
#include <string>

#include "detection_msg.pb.h"

#include <zmq.hpp>
#include "frame_conversions.h"

#include "Item.h"
#include "Quad.h"

int main() {

  std::string log;
  // FastDDS default participant
  std::unique_ptr<DefaultParticipant> dp =
  std::make_unique<DefaultParticipant>(0, "raptor_vision");
  Quad quad("Quad", &log, dp, "mocap_srl_quad", "pos_cmd");
  //Item box("box", dp, "mocap_srl_box");
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

    quad_position.push_back(quad_pose.position.x);
    quad_position.push_back(quad_pose.position.y);
    quad_position.push_back(quad_pose.position.z);

    quad_orientation.push_back(quad_pose.orientation.roll);
    quad_orientation.push_back(quad_pose.orientation.pitch);
    quad_orientation.push_back(quad_pose.orientation.yaw);


    // Receive the object detection data
    zmq::message_t request;
    socket.send(zmq::str_buffer("ok"), zmq::send_flags::none);
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
    std::vector<float> point_in_cam_frame{det.x(), det.y(), det.z()};

    std::vector<float> point_camera_to_drone = util::euler_frame_conversion(
        point_in_cam_frame, camera_orientation, camera_translations);

    std::vector<float> point_drone_to_global = util::euler_frame_conversion(
        point_camera_to_drone, quad_orientation, quad_position);

    
    std::cout << point_drone_to_global.at(0) << std::endl;
    std::cout << point_drone_to_global.at(0) << std::endl;
    std::cout << point_drone_to_global.at(0) << std::endl;
    
    // point_drone_to_global.at(0);
    // point_drone_to_global.at(1);
    // point_drone_to_global.at(2);
    
    
  }
  return 0;
}
