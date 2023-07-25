// C++ Standard Libraries
#include <atomic>
#include <cassert>
#include <deque>
#include <iostream>
#include <functional>
#include <map>
#include <mutex>
#include <string>
#include <string.h>
#include <tuple>
#include <iomanip>

#include "messages/testMavMessage/mavlink.h"
#include "messages/mavlink_types.h"




int main() {

    // TODO:
    mavlink_message_t* mavMessage = new mavlink_message_t;

    mavlink_heartbeat_t* myHeartbeat = new mavlink_heartbeat_t;
    myHeartbeat->autopilot = 0x4;
    myHeartbeat->type = 0x4;
    myHeartbeat->autopilot = 0x4;
    myHeartbeat->custom_mode = 0x4;
    myHeartbeat->system_status = 0x4;


    uint16_t size  = mavlink_msg_heartbeat_encode(mavMessage, myHeartbeat);

    std::cout << "ENCODED MESSAGE" << size <<std::endl;





    for(int i = 0; i<sizeof(mavlink_message_t); i++){
        //std::cout << std::hex << std::setw(2) << std::setfill('0') <<(int)(&mavMessage[i]) << ",";
        //std::cout << std::hex <<(uint8_t)(&mavMessage[i]) << ",";


        //printf("%02x",(unsigned int) ((char*)mavMessage)[i]);
        //printf(",");


    }

    for(int i = 0; i<MAVLINK_MSG_ID_HEARTBEAT_LEN; i++){

        printf("%02x",(unsigned int) ((char*)mavMessage->payload64)[i]);
        printf(",");
    }
    std::cout << std::endl;


    mavlink_heartbeat_t* myHeartbeatDecoded = new mavlink_heartbeat_t;

    mavlink_msg_heartbeat_decode(mavMessage, myHeartbeatDecoded);

    printf("decoded type:");
    printf("%02x",(unsigned int) ((char*)myHeartbeatDecoded->type));

    return 1;
}