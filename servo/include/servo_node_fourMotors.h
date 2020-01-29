#if !defined(_SERVO_NODE_FOURMOTORSH)
#define _SERVO_NODE_FOURMOTORSH

#include "ROSCommonNode.h"
#include "servoController.h"
#include "std_msgs/Int16.h"

namespace srcHals {
	
  class servo_node2: ROSCommonNode {
	  
	  public:
	  
		servo_node2(int argc, char **argv, const char *node_name);
		~ servo_node2(){};
	  
		int getServoPosition ();
	  
	  private:
	  
		servoController controller;
		
		bool servoOne, servoTwo, servoThree, servoFour;
		int positionOne, positionTwo, positionThree, positionFour;
		int oneMin, oneMax, twoMin, twoMax, threeMin, threeMax, fourMin, fourMax;
		
		ros::Subscriber subs[4];
		ros::Publisher pubs[4];
		
		void cbServoOneTarget(const std_msgs::Int16::ConstPtr& msg);
		void cbServoTwoTarget(const std_msgs::Int16::ConstPtr& msg);
		void cbServoThreeTarget(const std_msgs::Int16::ConstPtr& msg);
		void cbServoFourTarget(const std_msgs::Int16::ConstPtr& msg);
	  
		void run();
		void heartBeatHandler() { };


 };
 
};


#endif
