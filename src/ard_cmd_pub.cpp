#include <ros/ros.h>
#include <std_msgs/UInt8.h>

ros::Publisher Cmd_pub;

int main(int argc, char** argv )
{
	ros::init(argc, argv, "cmd_input");
	ros::NodeHandle n;
	Cmd_pub = n.advertise<std_msgs::UInt8>("ros_chatter", 10);
	std_msgs::UInt8 msg;
	char cmd;

	while(ros::ok())
	{
		cmd = getchar();
		if (cmd == '1')
		{
			msg.data = 1;
			Cmd_pub.publish(msg);
		}
		else if(cmd == '2')
		{
			msg.data = 2;
			Cmd_pub.publish(msg);
		} 
		else 
		{
			if (cmd == '\n')
			{
				continue;
			}
			msg.data = 0;
			Cmd_pub.publish(msg);
		}
	}
}