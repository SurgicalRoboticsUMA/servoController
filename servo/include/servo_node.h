#if !defined(_SERVO_NODE_H)
#define _SERVO_NODE_H

#include "ROSCommonNode.h"
#include "servoController.h"
#include "std_msgs/Int16.h"

namespace srcHals {
	
  class servo_node: ROSCommonNode {
	  
	  public:
	  
		servo_node(int argc, char **argv, const char *node_name);
		~ servo_node(){};
	  
		int getServoPosition ();
	  
	  private:
	  
		servoController controller;
		
		bool servoRight, servoLeft;
		int positionRight, positionLeft;
		int rightMin, rightMax, leftMin, leftMax;
		
		ros::Subscriber subs[2];
		ros::Publisher pubs[2];
		
		void cbServoRightTarget(const std_msgs::Int16::ConstPtr& msg);
		void cbServoLeftTarget(const std_msgs::Int16::ConstPtr& msg);
	  
		void run();
		void heartBeatHandler() { };


 };
 
};


#endif
