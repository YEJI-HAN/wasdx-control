#include "ros/ros.h"

#include "key_yeji/key_msg.h"

 

#define w 0x77

#define d 0x64

#define a 0x61

#define s 0x73

#define x 0x78

 

void callback(const key_yeji::key_msg::ConstPtr& msg)

{
if(msg->key == 0x77 | msg->key == 0x64 | msg->key == 0x61 | msg->key == 0x73 | msg->key == 0x78)
    ROS_INFO("key = %c",msg->key);
else printf("input wasdx!\n");

}

 

int main(int argc, char **argv)

{

    ros::init(argc,argv,"key_sub");

    ros::NodeHandle nh;

    

    ros::Subscriber key_sub = nh.subscribe("key",100,callback);

    ros::spin();

    

    return 0;

}

 
