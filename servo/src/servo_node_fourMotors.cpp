#include "servo_node_fourMotors.h"


namespace srcHals {
	
	 servo_node2::servo_node2(int argc, char **argv, const char *node_name): ROSCommonNode(argc, argv, node_name),
	 servoOne(false),
	 servoTwo(false),
	 servoThree(false),
	 servoFour(false){
	 
		nh->param("oneMin", oneMin, 922);
		nh->param("oneMax", oneMax, 1922);
		
		nh->param("twoMin", twoMin, 922);
		nh->param("twoMax", twoMax, 1922);
		
		nh->param("threeMin", threeMin, 922);
		nh->param("threeMax", threeMax, 1922);
		
		nh->param("fourMin", fourMin, 922);
		nh->param("fourMax", fourMax, 1922);
		
		pubs[0] = nh->advertise<std_msgs::Int16>("/EndoWrist/servoOne/pos", 1000);
		pubs[1] = nh->advertise<std_msgs::Int16>("/EndoWrist/servoTwo/pos", 1000);
		pubs[2] = nh->advertise<std_msgs::Int16>("/EndoWrist/servoThree/pos", 1000);
		pubs[3] = nh->advertise<std_msgs::Int16>("/EndoWrist/servoFour/pos", 1000);
		
		subs[0] = nh->subscribe("/EndoWrist/servo1/target", 1000, &servo_node2::cbServoOneTarget, this);
		
		subs[1] = nh->subscribe("/EndoWrist/servo2/target", 1000, &servo_node2::cbServoTwoTarget, this);
		
		subs[2] = nh->subscribe("/EndoWrist/servo3/target", 1000, &servo_node2::cbServoThreeTarget, this);
		
		subs[3] = nh->subscribe("/EndoWrist/servo4/target", 1000, &servo_node2::cbServoFourTarget, this);
	 
		run();
	 
	 }
	 
	 int servo_node2::getServoPosition (){
		 
		 short positions[4];
		 
		 controller.getServoPosition(positions, 4);
		 
		 std_msgs::Int16 posOne, posTwo, posThree, posFour;
		 posOne.data = positions[0];
		 posTwo.data = positions[1];
		 posThree.data = positions[2];
		 posFour.data = positions [3];
		 
		 pubs[0].publish(posOne);
		 pubs[1].publish(posTwo);
		 pubs[2].publish(posThree);
		 pubs[3].publish(posFour);
	 }
	 
	 void servo_node2::run(){
			 
	     ros::Rate loop_rate(125);
	     
		controller.setTarget(oneMin, 0);
		controller.setTarget(twoMin, 1);
		controller.setTarget(threeMin, 2);
		controller.setTarget(fourMin, 3);
		
		 
		 while (ros::ok()){
			 
			getServoPosition();
			
			if (servoOne){
				std::cout<<"setTarget"<<std::endl;
				controller.setTarget(positionOne, 0);
				
				servoOne = false;
			}
			
			if (servoTwo){
				controller.setTarget(positionTwo, 1);
				servoTwo = false;
			}
			
			if (servoThree){
				std::cout<<"setTarget"<<std::endl;
				controller.setTarget(positionThree, 2);
				
				servoThree = false;
			}
			
			if (servoFour){
				controller.setTarget(positionFour, 3);
				servoFour = false;
			}
			 			
			
			ros::spinOnce();
			loop_rate.sleep();			 
		 }
		 
	 }
	 
	 void servo_node2::cbServoOneTarget(const std_msgs::Int16::ConstPtr& msg){
		 
		 servoOne = true;
		 positionOne = msg->data;
		 std::cout<<"cbServoOneTarget: "<<positionOne<<std::endl;
	 }
	 
	 void servo_node2::cbServoTwoTarget(const std_msgs::Int16::ConstPtr& msg){
		 
		 servoTwo = true;
		 positionTwo = msg->data;
		 std::cout<<"cbServoTwoTarget: "<<positionTwo<<std::endl;

	 }
	 
	  void servo_node2::cbServoThreeTarget(const std_msgs::Int16::ConstPtr& msg){
		 
		 servoThree = true;
		 positionThree = msg->data;
		 std::cout<<"cbServoThreeTarget: "<<positionThree<<std::endl;
	 }
	 
	 void servo_node2::cbServoFourTarget(const std_msgs::Int16::ConstPtr& msg){
		 
		 servoFour = true;
		 positionFour = msg->data;
		 std::cout<<"cbServoFourTarget: "<<positionFour<<std::endl;

	 }
	 
	 
}

// START OF THE ROS NODE
int main(int argc, char **argv) {
 
  srcHals::servo_node2 node(argc, argv, "servo_node");

  return 0;
}

