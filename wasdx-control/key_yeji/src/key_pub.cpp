#include "ros/ros.h"

#include "key_yeji/key_msg.h"

 

#define w 0x77

#define d 0x64

#define a 0x61

#define s 0x73

#define x 0x78

 

int main(int argc, char **argv)

{

    ros::init(argc,argv, "key_pub");

    ros::NodeHandle nh;

    

    ros::Publisher key_pub = nh.advertise<key_yeji::key_msg>("key",100);

    ros::Rate loop_rate(10);

    key_yeji::key_msg msg;

    

    while(ros::ok())

    {

        printf("forward:w, left:a, backward:s, right:d, stop:x" );

        scanf("%c",&msg.key);
	while(getchar() != '\n'); 


        key_pub.publish(msg);

    }

    return 0;

}
