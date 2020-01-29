//
//
//  @ Project   : Broca - Software Architecture
//  @ File Name : ROSCommonNode.h
//  @ Date      : 24/11/2014
//  @ Author    : Mª Carmen López, University of Malaga
//                Enrique Bauzano, University of Malaga
//  @ Copyright : Tecnalia & UMA
//
//

#if !defined(_ROSCOMMONNODE_H)
#define _ROSCOMMONNODE_H

#include <ros/ros.h>

namespace srcHals {

  /*! \brief ROSCommonNode class for ROS node creation.
   * 
   * The ROSCommonNode class initialize a ROS node and generate the ROS handle 
   * to manage all the communications. All the topic will be declared relative
   * to the node name.
   * 
   * It defines two main operation that have to be implemented in every ROS node:
   * - run(): node main loop.
   * - heartBeatHandler: method to propagate the ROS internal clock that will be
   *			 used as a communications heart beat.
   * 
   */
  class ROSCommonNode{

    protected:
    
      ros::NodeHandle *nh; /*!< ROS node handle. */
    
    /*! \brief Constructor of the ROSCommonNode class
     * 
     * This constructor can only be casted by passing the command line arguments 
     * and the node name. The ROS node initialization needs to see argc and argv
     * so that it can perform any ROS arguments and name remapping that were 
     * provided at the command line.
     * 
     * \param argc is the argument count. The number of arguments passed into the
     * 		program from the command line.
     * \param argv is the argument list.
     * \param node_name is the name of the node.
     */  
    ROSCommonNode(int argc, char **argv, const char *node_name) {
      
      ros::init(argc, argv, node_name);
      nh = new ros::NodeHandle("~");
      
    }


      /*! \brief ROS node main loop
       * 
       * This method must be overriden by the derived class.
       */
      virtual void run () = 0;              
      
      /*! \brief Heart beat handle method
       * 
       * Method to propagate the ROS internal clock to the driver modules of the robot
       * unit. It will be used as a communications heart beat; to detect any communications
       * lost.
       * 
       * This method must be overriden by the derived class.
       */
      //~ virtual void heartBeatHandler () = 0; 
  };
};
#endif
