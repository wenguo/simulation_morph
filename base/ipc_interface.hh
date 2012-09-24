/*
 *      Created on: 01/2012
 *          Author: Wenguo Liu (wenguo.liu@brl.ac.uk)
 *
*/
#ifndef IPC_INTERFACE_HH
#define IPC_INTERFACE_HH

namespace IPC{

    enum IPC_msg_type
    {
        STATUS_INFO = 0x1,
        CAMERA_POSE = 0x2,
        COMMAND = 0xA0,
        ORGANISM_INFO = 0xF0
    };

    //data type for status_t
    struct position_t
    {
        int32_t x;
        int32_t y;
        int32_t pa;
    }__attribute__((packed));

    struct status_t
    {
        int32_t id;
        int32_t state;
        position_t pos;
    }__attribute__((packed));

    //data type for camera_pose_t
    //this is used to control the camera position in robot3d
    struct camera_pose_t
    {
        int32_t id;
        int32_t x;
        int32_t y;
        int32_t z;
        int32_t yaw;
        int32_t pitch;
    }__attribute__((packed));


    //data type for command_t
    enum cmd_type_t
    {
        CMD_DOCKING = 0x1,
        CMD_UNDOCKING = 0xf1
    };

    struct command_t
    {
        int32_t id;
        uint8_t cmd_type;
        uint8_t data[5];
    }__attribute__((packed));

    //data type for organism_t
    struct organism_info_t
    {
        int32_t id;
        int32_t size;
        uint8_t * data;
    };

}
#endif
