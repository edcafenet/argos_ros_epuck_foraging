#ifndef ARGOS_ROS_EPUCK_H
#define ARGOS_ROS_EPUCK_H

#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/core/utility/datatypes/byte_array.h>
#include <argos3/plugins/robots/e-puck/control_interface/ci_epuck_wheels_actuator.h>
#include <argos3/plugins/robots/e-puck/control_interface/ci_epuck_proximity_sensor.h>
#include <argos3/plugins/robots/e-puck/control_interface/ci_epuck_omnidirectional_camera_sensor.h>
#include <argos3/plugins/robots/e-puck/control_interface/ci_epuck_range_and_bearing_actuator.h>
#include <argos3/plugins/robots/e-puck/control_interface/ci_epuck_range_and_bearing_sensor.h>
#include <argos3/plugins/robots/e-puck/control_interface/ci_epuck_base_leds_actuator.h>
#include <argos3/plugins/robots/e-puck/control_interface/ci_epuck_rgb_leds_actuator.h>
#include <argos3/plugins/robots/e-puck/control_interface/ci_epuck_battery_sensor.h>

#include "RabBuffer.h"

#include "argos_ros_epuck/Proximity.h"
#include "argos_ros_epuck/ProximityList.h"
#include "argos_ros_epuck/Neighbor.h"
#include "argos_ros_epuck/NeighborList.h"
#include "argos_ros_epuck/Leaf.h"
#include "argos_ros_epuck/MerkleLeafList.h"

#include <ros/ros.h>
#include <string>
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "std_msgs/ColorRGBA.h"

using namespace argos;

class CArgosRosEpuck : public CCI_Controller {

public:

  CArgosRosEpuck();
  virtual ~CArgosRosEpuck() {}

  /*
   * This function initializes the controller.
   * The 't_node' variable points to the <parameters> section in the XML
   * file in the <controllers><footbot_ccw_wander_controller> section.
   */
  virtual void Init(TConfigurationNode& t_node);

  /*
   * This function is called once every time step.
   * The length of the time step is set in the XML file.
   */
  virtual void ControlStep();

  /*
   * This function resets the controller to its state right after the
   * Init().
   * It is called when you press the reset button in the GUI.
   * In this example controller there is no need for resetting anything,
   * so the function could have been omitted. It's here just for
   * completeness.
   */
  virtual void Reset() {}

  /*
   * Called to cleanup what done by Init() when the experiment finishes.
   * In this example controller there is no need for clean anything up,
   * so the function could have been omitted. It's here just for
   * completeness.
   */
  virtual void Destroy() {}

  /*
   * The callback method for getting new commanded speed on the cmd_vel topic.
   */
  void cmdVelCallback(const geometry_msgs::Twist& twist);

  /*
   * The callback method for turning on/off the base LEDs
   */
  void BaseLEDsCallback(const std_msgs::Bool& msg);

  /*
   * The callback method for switching the color of the RGB LEDs
   */
  void RGBLEDsCallback(const std_msgs::ColorRGBA& msg);

  /*
   * The callback method for getting the Completed Merkle Tree Object.
   */
  void completedMerkleCallback(const argos_ros_epuck::MerkleLeafList& MerkleLeafList);

  /*
   * Returns the current Color of the RGB robot ring
   */
  inline CColor& GetCurrentRGBColor() {
    return CurrentRGBColor;
  }

  /*
   * Returns the neighbors list
   */
  inline argos_ros_epuck::NeighborList& GetNeighborListForLoopFunction() {
    return neighborList;
  }

  /*
   * Returns the Completed Merkle Tree Object
   */
  inline argos_ros_epuck::MerkleLeafList& GetCompletedMerkleTreeForLoopFunction() {
    return completedMerkleList;
  }

  void SetRangeAndBearingMessages(CCI_EPuckRangeAndBearingSensor::TPackets s_packets);

private:

  CCI_EPuckWheelsActuator* m_pcWheels;
  CCI_EPuckProximitySensor* m_pcProximity;
  CCI_EPuckRangeAndBearingActuator* m_pcRABAct;
  CCI_EPuckRangeAndBearingSensor* m_pcRABSens;
  CCI_EPuckBaseLEDsActuator* m_pcBaseLEDs;
  CCI_EPuckRGBLEDsActuator* m_pcRGBLEDs;
  CCI_EPuckBatterySensor* m_pcBatterySens;

  /*
   * Pointer to the range-and-bearing messages buffer.
   */
  RabBuffer m_pcRabMessageBuffer;

  // The following constant values were copied from the argos source tree from
  // the file src/plugins/robots/foot-bot/simulator/footbot_entity.cpp
  static const Real HALF_BASELINE = 0.07f; // Half the distance between wheels
  static const Real WHEEL_RADIUS = 0.029112741f;

  // This is the Robot ID as an integer, not as a string
  int RobotID;

  /*
   * The following variables are used as parameters for the
   * algorithm. You can set their value in the <parameters> section
   * of the XML configuration file, under the
   * <controllers><argos_ros_epuck_bot_controller> section.
   */

  // The number of time steps from the time step of the last callback
  // after which leftSpeed and rightSpeed will be set to zero.  Useful to
  // shutdown the robot after the controlling code on the ROS side has quit.
  int stopWithoutSubscriberCount;

  // The number of time steps since the last callback.
  int stepsSinceCallback;

  // Most recent left and right wheel speeds, converted from the ROS twist
  // message.
  Real leftSpeed, rightSpeed;

  // Current Color in the RGB Leds Ring
  CColor CurrentRGBColor;

  // ROS
  // Proximity sensor publisher
  ros::Publisher proximityPub;

  // Neighbor list publisher
  ros::Publisher neighborListPub;

  // Battery level publisher and variable
  ros::Publisher batteryPub;
  std_msgs::Float32 battery_level;

  // Subscriber for cmd_vel (Twist message) topic.
  ros::Subscriber cmdVelSub;

  // Subscriber for the base LEDs topic.
  ros::Subscriber BaseLEDsSub;

  // Subscriber for the base LEDs topic.
  ros::Subscriber RGBLEDsSub;

  // Subscriber for Completed Merkle (MerkleLeafList) topic.
  ros::Subscriber CompletedMerkleSub;

  // List of current neighbors
  // This list is a private list since it needs to be
  // accesible from the loop functions
  argos_ros_epuck::NeighborList neighborList;

  // List of Merkle Leafs
  // This list is a private list since it needs to be
  // accesible from the loop functions
  argos_ros_epuck::MerkleLeafList completedMerkleList;

  /*
   * Functions to get Sensor Readings
   */

  /* Get readings from proximity sensor */
  argos_ros_epuck::ProximityList GetProximityList();
  /* Publish Neighbor List */
  argos_ros_epuck::NeighborList GetNeighborList();

};

#endif
