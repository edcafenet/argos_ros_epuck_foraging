/* Include the controller definition */
#include "argos_ros_epuck_foraging.h"
/* Function definitexitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>

#include <iostream>
#include <sstream>
#include <ros/callback_queue.h>

using namespace std;
using namespace argos_ros_epuck;

// Auxiliary functions
UInt8 GetRobotID(std::string ArgosID){
  std::string ID = ArgosID.substr(ArgosID.length()-2);
  return atoi(ID.c_str());
}

/****************************************/
/****************************************/
CArgosRosEpuck::CArgosRosEpuck() :
  m_pcWheels(NULL),
  m_pcProximity(NULL),
  m_pcRABAct(NULL),
  m_pcRABSens(NULL),
  m_pcBaseLEDs(NULL),
  m_pcRGBLEDs(NULL),
  stopWithoutSubscriberCount(5),
  stepsSinceCallback(0),
  leftSpeed(0),
  rightSpeed(0)
{}

void CArgosRosEpuck::Init(TConfigurationNode& t_node) {
  // Initialize RobotID
  RobotID = GetRobotID(GetId());

  int argc = 0;
  char *argv = (char *) "";
  ros::init(argc, &argv, GetId());
  ros::NodeHandle nodeHandle;

  // Create the topics to publish
  stringstream proximityTopic, neighborListTopic, batteryTopic;
  proximityTopic << "/" << GetId() << "/proximity";
  neighborListTopic << "/" << GetId() << "/comm/neighbor_list";
  batteryTopic << "/" << GetId() << "/battery";
  neighborListPub = nodeHandle.advertise<NeighborList>(neighborListTopic.str(), 1);
  proximityPub = nodeHandle.advertise<ProximityList>(proximityTopic.str(), 1);
  batteryPub = nodeHandle.advertise<std_msgs::Float32>(batteryTopic.str(), 1);

  // Create the subscribers
  stringstream cmdVelTopic, BaseLEDsTopic, RGBLEDsTopic, completedMerkleTreeTopic;
  cmdVelTopic << "/" << GetId() << "/cmd_vel";
  BaseLEDsTopic << "/" << GetId() << "/base_leds";
  RGBLEDsTopic << "/" << GetId() << "/rgb_leds";
  completedMerkleTreeTopic  << "/" << GetId() << "/comm/merkle_completed";

  cmdVelSub = nodeHandle.subscribe(cmdVelTopic.str(), 1, &CArgosRosEpuck::cmdVelCallback, this);
  CompletedMerkleSub = nodeHandle.subscribe(completedMerkleTreeTopic.str(), 1, &CArgosRosEpuck::completedMerkleCallback, this);
  BaseLEDsSub = nodeHandle.subscribe(BaseLEDsTopic.str(), 1, &CArgosRosEpuck::BaseLEDsCallback, this);
  RGBLEDsSub = nodeHandle.subscribe(RGBLEDsTopic.str(), 1, &CArgosRosEpuck::RGBLEDsCallback, this);

  // Get sensor/actuator handles
  m_pcWheels = GetActuator<CCI_EPuckWheelsActuator>("epuck_wheels");
  m_pcProximity = GetSensor<CCI_EPuckProximitySensor>("epuck_proximity");
  m_pcRABAct = GetActuator<CCI_EPuckRangeAndBearingActuator>("epuck_range_and_bearing");
  m_pcRABSens = GetSensor<CCI_EPuckRangeAndBearingSensor>("epuck_range_and_bearing" );
  m_pcBaseLEDs = GetActuator<CCI_EPuckBaseLEDsActuator>("epuck_base_leds");
  m_pcRGBLEDs = GetActuator<CCI_EPuckRGBLEDsActuator>("epuck_rgb_leds");

  /* Parse the configuration file */
  GetNodeAttributeOrDefault(t_node, "stopWithoutSubscriberCount",\
			    stopWithoutSubscriberCount,\
			    stopWithoutSubscriberCount);
}

void CArgosRosEpuck::cmdVelCallback(const geometry_msgs::Twist& twist) {
  Real v = twist.linear.x;  // Forward speed
  Real w = twist.angular.z; // Rotational speed

  static const Real INTERWHEEL_DISTANCE        = 0.053f;
  static const Real HALF_INTERWHEEL_DISTANCE   = INTERWHEEL_DISTANCE * 0.5f;
  static const Real WHEEL_RADIUS               = 0.0205f;

  // Use the kinematics of a differential-drive robot
  leftSpeed = (v - HALF_INTERWHEEL_DISTANCE * w) / WHEEL_RADIUS;
  rightSpeed = (v + HALF_INTERWHEEL_DISTANCE * w) / WHEEL_RADIUS;

  stepsSinceCallback = 0;
}

void CArgosRosEpuck::BaseLEDsCallback(const std_msgs::Bool& msg){
  m_pcBaseLEDs->SwitchLEDs(msg.data);
}

void CArgosRosEpuck::RGBLEDsCallback(const std_msgs::ColorRGBA& msg){
  CColor rgb_leds(msg.r, msg.g, msg.b, msg.a);
  m_pcRGBLEDs->SetColors(rgb_leds);
  CurrentRGBColor = rgb_leds;
}

void CArgosRosEpuck::SetRangeAndBearingMessages(CCI_EPuckRangeAndBearingSensor::TPackets s_packets) {
  std::map<UInt8, CCI_EPuckRangeAndBearingSensor::SReceivedPacket*> mapRemainingMessages;
  std::map<UInt8, CCI_EPuckRangeAndBearingSensor::SReceivedPacket*>::iterator mapIt;
  CCI_EPuckRangeAndBearingSensor::TPackets::iterator it;

  for (it = s_packets.begin(); it < s_packets.end(); ++it)
    if ((*it)->Data[1] != RobotID)
      if (!mapRemainingMessages.count((*it)->Data[1])){   // If ID not in map, add message.
	mapRemainingMessages[(*it)->Data[1]] = (*it);
	m_pcRabMessageBuffer.AddMessage((*it));
      }

  m_pcRabMessageBuffer.Update();
}

void CArgosRosEpuck::completedMerkleCallback(const argos_ros_epuck::MerkleLeafList& MerkleLeafList){
  completedMerkleList = MerkleLeafList;
}

ProximityList CArgosRosEpuck::GetProximityList(){
  const CCI_EPuckProximitySensor::TReadings& tProxReads = m_pcProximity->GetReadings();
  ProximityList proxList;
  proxList.n = tProxReads.size();
  for (size_t i = 0; i < proxList.n; ++i) {
    Proximity prox;
    prox.value = tProxReads[i].Value;
    prox.angle = tProxReads[i].Angle.GetValue();
    proxList.proximities.push_back(prox);
  }
  proxList.header.stamp = ros::Time::now();
  return proxList;
}

NeighborList CArgosRosEpuck::GetNeighborList(){
  CCI_EPuckRangeAndBearingSensor::TPackets packets = m_pcRABSens->GetPackets();
  SetRangeAndBearingMessages(packets);
  CCI_EPuckRangeAndBearingSensor::TPackets sLastPackets = m_pcRabMessageBuffer.GetMessages();
  CCI_EPuckRangeAndBearingSensor::TPackets::iterator it;
  NeighborList neighborList;
  std::vector<UInt8> MessageIDs;

  for (it = sLastPackets.begin(); it != sLastPackets.end(); it++){
    if(!std::count(MessageIDs.begin(), MessageIDs.end(), (*it)->Data[1])){
      Neighbor neighbor;
      neighbor.ID = (*it)->Data[1];
      neighbor.range = (*it)->Range;
      neighborList.neighbors.push_back(neighbor);
      neighborList.n += 1;
      MessageIDs.push_back((*it)->Data[1]);
    }
  }
  neighborList.header.stamp = ros::Time::now();
  m_pcRABSens->ClearPackets();
  return neighborList;
}

void CArgosRosEpuck::ControlStep(){

  /* Get readings from proximity sensor */
  ProximityList proxList = GetProximityList();

  /* Publish Proximity Object List */
  proximityPub.publish(proxList);

  /* Get RAB messages from nearby robots */
  neighborList = GetNeighborList();

  /* Publish Battery Level  */
  battery_level.data = m_pcBatterySens->GetBatteryLevel();
  batteryPub.publish(battery_level);

  /* Publish Neighbor List */
  neighborListPub.publish(neighborList);

  /* Prepare Message to send */
  UInt8 data[2];
  data[0] = 0;
  data[1] = RobotID;

  /* Send message */
  m_pcRABAct->SetData(data);

  // Wait for any callbacks to be called.
  ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0));

  // If we haven't heard from the subscriber in a while, set the speed to zero.
  if (stepsSinceCallback > stopWithoutSubscriberCount)
  {
    leftSpeed = 0;
    rightSpeed = 0;
  } else {
    stepsSinceCallback++;
  }
  m_pcWheels->SetLinearVelocity(leftSpeed, rightSpeed);
}

/*
* This statement notifies ARGoS of the existence of the controller.
* It binds the class passed as first argument to the string passed as
* second argument.
* The string is then usable in the configuration file to refer to this
* controller.
 * When ARGoS reads that string in the configuration file, it knows which
 * controller class to instantiate.
 * See also the configuration files for an example of how this is used.
 */
REGISTER_CONTROLLER(CArgosRosEpuck, "argos_ros_epuck_controller")
