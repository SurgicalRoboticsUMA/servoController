#include "servo_node.h"


namespace srcHals {
	
	 servo_node::servo_node(int argc, char **argv, const char *node_name): ROSCommonNode(argc, argv, node_name),
	 servoRight(false),
	 servoLeft(false){
	 
		nh->param("rightMin", rightMin, 1200);
		nh->param("rightMax", rightMax, 1450);
		
		nh->param("leftMin", leftMin, 1540);
		nh->param("leftMax", leftMax, 1760);
		
		pubs[0] = nh->advertise<std_msgs::Int16>("/srcHals/servoRight/pos", 1000);
		pubs[1] = nh->advertise<std_msgs::Int16>("/srcHals/servoLeft/pos", 1000);
		
		subs[0] = nh->subscribe("/srcHals/servoRight/target", 1000, &servo_node::cbServoRightTarget, this);
		
		subs[1] = nh->subscribe("/srcHals/servoLeft/target", 1000, &servo_node::cbServoLeftTarget, this);
	 
		run();
	 
	 }
	 
	 int servo_node::getServoPosition (){
		 
		 short positions[2];
		 
		 controller.getServoPosition(positions, 2);
		 
		 std_msgs::Int16 posRight, posLeft;
		 posRight.data = (positions[0] - rightMin)/((rightMax-rightMin)/50.0);
		 posLeft.data = (positions[1] - leftMin)/((leftMax-leftMin)/50.0);
		 
		 pubs[0].publish(posRight);
		 pubs[1].publish(posLeft);
	 }
	 
	 void servo_node::run(){
			 
	     ros::Rate loop_rate(125);
	     
		controller.setTarget(rightMin, 0);
		controller.setTarget(leftMin, 1); 
		 
		 while (ros::ok()){
			 
			getServoPosition();
			
			if (servoRight){
				std::cout<<"setTarget"<<std::endl;
				controller.setTarget(positionRight, 0);
				
				servoRight = false;
			}
			
			if (servoLeft){
				controller.setTarget(positionLeft, 1);
				servoLeft = false;
			}
			 			
			
			ros::spinOnce();
			loop_rate.sleep();			 
		 }
		 
	 }
	 
	 void servo_node::cbServoRightTarget(const std_msgs::Int16::ConstPtr& msg){
		 
		 servoRight = true;
		 positionRight = (rightMax-rightMin)/50.0*(msg->data) + rightMin;
		 std::cout<<"cbServoRightTarget: "<<positionRight<<std::endl;
	 }
	 
	 void servo_node::cbServoLeftTarget(const std_msgs::Int16::ConstPtr& msg){
		 
		 servoLeft = true;
		 positionLeft = (leftMax-leftMin)/50.0*(msg->data) + leftMin;
	 }
	 
}

// START OF THE ROS NODE
int main(int argc, char **argv) {
 
  srcHals::servo_node node(argc, argv, "servo_node");

  return 0;
}

