#include <fstream>
#include <google/protobuf/util/time_util.h>
#include <iostream>
#include <string>

#include "detection_msg.pb.h"
#include "addressbook.pb.h"
#include <zmq.hpp>

int main () {
    //  Prepare our context and socket
    zmq::context_t context (1);
    zmq::socket_t socket (context, ZMQ_REP);
    socket.bind ("tcp://*:5555");

    while (true) {
        zmq::message_t request;

        //  Wait for next request from client
        auto res = socket.recv (request, zmq::recv_flags::none);
        std::cout << "Received req" << std::endl;

        // deserialize protobuf message 
        // Detector::Detection det;
        tutorial::Detection det;
        if (!det.ParseFromString(request.to_string())) {
            std::cerr << "Failed to parse." << std::endl;
            return -1;
        }

	    std::cout << det.DebugString() << std::endl;


        //  Send reply back to client
        zmq::message_t reply (5);
        memcpy ((void *) reply.data (), "World", 5);
        socket.send (zmq::str_buffer("A"), zmq::send_flags::none);
    }
    return 0;
}