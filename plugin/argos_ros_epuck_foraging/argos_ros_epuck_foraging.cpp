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
int GetRobotID(std::string ArgosID){
  std::string s = "epuck_";
  std::string::size_type i = ArgosID.find(s);
  if (i != std::string::npos)
    std::string ID = ArgosID.erase(i, s.length());      
  return  atoi(ArgosID.c_str());      
}

// Compares pucks for sorting purposes.  We sort by angle.
bool puckComparator(Puck a, Puck b) {
  return a.angle < b.angle;
}

// Returns proper color codes for puck types
int GetPuckTypeBasedOnColor(CColor Color){

  int code;
  
  if (Color == CColor::GREEN)      
    code = 1;
  else if (Color == CColor::RED)
    code = 2;
  else if (Color == CColor::BLUE)
    code = 3;
  else if (Color == CColor::YELLOW)
    code = 4;
  else if (Color == CColor::MAGENTA)
    code = 5;
  else if (Color == CColor::CYAN)
    code = 6;    
  else if (Color == CColor::WHITE)
    code = 7;
  else if (Color == CColor::ORANGE)
    code = 8;

  return code;
}

// Initialize ROS node.  There will be only one ROS node no matter how many robots are created in
// ARGoS.  However, we will have one instance of the CArgosRosEpuck class for each ARGoS robot.
ros::NodeHandle* initROS() {
  int argc = 0;
  char *argv = (char *) "";
  ros::init(argc, &argv, "argos_ros_epuck");
  return new ros::NodeHandle();
}

ros::NodeHandle* CArgosRosEpuck::nodeHandle = initROS();

/****************************************/
/****************************************/
CArgosRosEpuck::CArgosRosEpuck() :
  m_pcWheels(NULL),
  m_pcProximity(NULL),
  m_pcOmniCam(NULL),
  m_pcPosition(NULL),
  m_pcRABAct(NULL),
  m_pcRABSens(NULL),
  m_pcBaseLEDs(NULL),
  m_pcRGBLEDs(NULL),
  stopWithoutSubscriberCount(10),
  stepsSinceCallback(0),
  leftSpeed(0),
  rightSpeed(0)
{
}

void CArgosRosEpuck::Init(TConfigurationNode& t_node) {
  // Initialize RobotID
  RobotID = GetRobotID(GetId());
  
  // Create the topics to publish
  stringstream puckListTopic, proximityTopic, positionTopic, groundTopic, neighborListTopic;
  puckListTopic << "/" << GetId() << "/puck_list";
  proximityTopic << "/" << GetId() << "/proximity";
  positionTopic << "/" << GetId() << "/position";
  groundTopic << "/" << GetId() << "/ground";
  neighborListTopic << "/" << GetId() << "/comm/neighbor_list";

  puckListPub = nodeHandle->advertise<PuckList>(puckListTopic.str(), 1);
  neighborListPub = nodeHandle->advertise<NeighborList>(neighborListTopic.str(), 1);
  proximityPub = nodeHandle->advertise<ProximityList>(proximityTopic.str(), 1);
  posePub = nodeHandle->advertise<geometry_msgs::Pose>(positionTopic.str(), 1);
  groundPub = nodeHandle->advertise<GroundSensorPack>(groundTopic.str(), 1);
  
  // Create the subscribers
  stringstream cmdVelTopic, BaseLEDsTopic, RGBLEDsTopic, completedMerkleTreeTopic;
  cmdVelTopic << "/" << GetId() << "/cmd_vel";
  BaseLEDsTopic << "/" << GetId() << "/base_leds";
  RGBLEDsTopic << "/" << GetId() << "/rgb_leds";
  completedMerkleTreeTopic  << "/" << GetId() << "/comm/merkle_completed";

  cmdVelSub = nodeHandle->subscribe(cmdVelTopic.str(), 1, &CArgosRosEpuck::cmdVelCallback, this);
  CompletedMerkleSub = nodeHandle->subscribe(completedMerkleTreeTopic.str(), 1, &CArgosRosEpuck::completedMerkleCallback, this);
  BaseLEDsSub = nodeHandle->subscribe(BaseLEDsTopic.str(), 1, &CArgosRosEpuck::BaseLEDsCallback, this);
  RGBLEDsSub = nodeHandle->subscribe(RGBLEDsTopic.str(), 1, &CArgosRosEpuck::RGBLEDsCallback, this);
  
  // Get sensor/actuator handles
  m_pcWheels = GetActuator<CCI_EPuckWheelsActuator>("epuck_wheels");
  m_pcProximity = GetSensor<CCI_EPuckProximitySensor>("epuck_proximity");
  m_pcOmniCam = GetSensor<CCI_EPuckOmnidirectionalCameraSensor>("epuck_omnidirectional_camera");
  m_pcPosition = GetSensor<CCI_PositioningSensor>("positioning");
  m_pcGround = GetSensor<CCI_EPuckGroundSensor>("epuck_ground");
  m_pcRABAct = GetActuator<CCI_EPuckRangeAndBearingActuator>("epuck_range_and_bearing");
  m_pcRABSens = GetSensor<CCI_EPuckRangeAndBearingSensor>("epuck_range_and_bearing" );
  m_pcBaseLEDs = GetActuator<CCI_EPuckBaseLEDsActuator>("epuck_base_leds");
  m_pcRGBLEDs = GetActuator<CCI_EPuckRGBLEDsActuator>("epuck_rgb_leds");
  m_pcOmniCam->Enable();

  /* Parse the configuration file */
  GetNodeAttributeOrDefault(t_node, "stopWithoutSubscriberCount",\
			    stopWithoutSubscriberCount,\
			    stopWithoutSubscriberCount);
}

void CArgosRosEpuck::cmdVelCallback(const geometry_msgs::Twist& twist) {
  Real v = twist.linear.x;  // Forward speed
  Real w = twist.angular.z; // Rotational speed

  // Use the kinematics of a differential-drive robot 
  leftSpeed = (v - HALF_BASELINE * w) / WHEEL_RADIUS;
  rightSpeed = (v + HALF_BASELINE * w) / WHEEL_RADIUS;

  stepsSinceCallback = 0;
}

void CArgosRosEpuck::BaseLEDsCallback(const std_msgs::Bool& msg){
  m_pcBaseLEDs->SwitchLEDs(msg.data);
}

void CArgosRosEpuck::RGBLEDsCallback(const std_msgs::String& msg){

  if(!msg.data.compare("green"))
    {
      m_pcRGBLEDs->SetColors(CColor::GREEN);
      CurrentRGBColor = CColor::GREEN;
    }
  
  else if(!msg.data.compare("red"))
    {
      m_pcRGBLEDs->SetColors(CColor::RED);
      CurrentRGBColor = CColor::RED;
    }
  
  else if(!msg.data.compare("blue"))
    {
      m_pcRGBLEDs->SetColors(CColor::BLUE);
      CurrentRGBColor = CColor::BLUE;
    }
  
  else if(!msg.data.compare("yellow"))
    {
      m_pcRGBLEDs->SetColors(CColor::YELLOW);
      CurrentRGBColor = CColor::YELLOW;
    }
  
  else if(!msg.data.compare("magenta"))
    {
      m_pcRGBLEDs->SetColors(CColor::MAGENTA);
      CurrentRGBColor = CColor::MAGENTA;
    }
  
  else if(!msg.data.compare("cyan"))
    {
      m_pcRGBLEDs->SetColors(CColor::CYAN);
      CurrentRGBColor = CColor::CYAN;
    }
  
  else if(!msg.data.compare("white"))
    {
      m_pcRGBLEDs->SetColors(CColor::WHITE);
      CurrentRGBColor = CColor::WHITE;
    }
  
  else if(!msg.data.compare("orange"))
    {
      m_pcRGBLEDs->SetColors(CColor::ORANGE);
      CurrentRGBColor = CColor::ORANGE;
    }
  else
    {
      m_pcRGBLEDs->SetColors(CColor::BLACK);
      CurrentRGBColor = CColor::BLACK;
    }
}


void CArgosRosEpuck::completedMerkleCallback(const argos_ros_epuck::MerkleLeafList& MerkleLeafList){
  completedMerkleList = MerkleLeafList;
}

GroundSensorPack CArgosRosEpuck::GetGroundSensorReadings(){
  const CCI_EPuckGroundSensor::SReadings& tGroundReads = m_pcGround->GetReadings();
  GroundSensorPack GroundSensorArray;

  GroundSensorArray.Left.value = tGroundReads[0];
  GroundSensorArray.Center.value = tGroundReads[1];
  GroundSensorArray.Right.value = tGroundReads[2];

  return GroundSensorArray;
}

geometry_msgs::Pose CArgosRosEpuck::GetPositionSensorReadings(){
  const CCI_PositioningSensor::SReading& posReading = m_pcPosition->GetReading();
  geometry_msgs::Pose Pose;

  /* Position and Orientation Readings */
  Pose.position.x = posReading.Position.GetX();
  Pose.position.y = posReading.Position.GetY();
  Pose.position.z = posReading.Position.GetZ();

  Pose.orientation.x = posReading.Orientation.GetX();
  Pose.orientation.y = posReading.Orientation.GetY();
  Pose.orientation.z = posReading.Orientation.GetZ();
  Pose.orientation.w = posReading.Orientation.GetW();
  
  return Pose;
}

PuckList CArgosRosEpuck::GetPuckList(){
  const CCI_EPuckOmnidirectionalCameraSensor::SReadings& camReads = m_pcOmniCam->GetReadings();
  
  PuckList puckList;
  puckList.n = camReads.BlobList.size();
  for (size_t i = 0; i < puckList.n; ++i){
    Puck puck;
    puck.range = camReads.BlobList[i]->Distance;
    // Make the angle of the puck in the range [-PI, PI].  This is useful for
    // tasks such as homing in on a puck using a simple controller based on
    // the sign of this angle.
    puck.angle = camReads.BlobList[i]->Angle.SignedNormalize().GetValue();
    puck.type = GetPuckTypeBasedOnColor(camReads.BlobList[i]->Color);
    puckList.pucks.push_back(puck);
  }

  // Sort the puck list by angle.  This is useful for the purposes of extracting meaning from
  // the local puck configuration (e.g. fitting a lines to the detected pucks).
  sort(puckList.pucks.begin(), puckList.pucks.end(), puckComparator);
  
  return puckList;
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
  return proxList;
}

NeighborList CArgosRosEpuck::GetNeighborList(){
  const CCI_EPuckRangeAndBearingSensor::TPackets& tMsgs = m_pcRABSens->GetPackets();
  NeighborList neighborList;
  /* Go through them to calculate */
  if(!tMsgs.empty()){
    /* A counter for the neighbors */
    neighborList.n = tMsgs.size();
    for(size_t i = 0; i < neighborList.n; ++i){
      Neighbor neighbor;
      neighbor.ID = tMsgs[i]->Data[1];
      neighbor.range = tMsgs[i]->Range;
      neighborList.neighbors.push_back(neighbor);
    } 
  }
  return neighborList;
}

UInt8* CArgosRosEpuck::PrepareMessageToSend(){
/* Prepare message to send */
  UInt8 data[2];
  data[0] = 0;
  data[1] = RobotID;
  return data;
}

void CArgosRosEpuck::ControlStep(){

  /* Get readings from ground sensor */
  GroundSensorPack GroundSensorArray = GetGroundSensorReadings();

  /* Publish Ground Sensor Info */
  groundPub.publish(GroundSensorArray);

  /* Get reading from position sensor */
  geometry_msgs::Pose Pose = GetPositionSensorReadings();

  /* Publish Pose Info */
  posePub.publish(Pose);

  /* Get Pucks around the robot */
  PuckList puckList = GetPuckList();
    
  /* Publish Puck List */
  puckListPub.publish(puckList);

  /* Get readings from proximity sensor */
  ProximityList proxList = GetProximityList();

  /* Publish Proximity Object List */
  proximityPub.publish(proxList);

  /* Get RAB messages from nearby robots */
  /* This list is different from the rest to be accesible from the loop functions */
  neighborList.neighbors.clear();
  neighborList = GetNeighborList();
  
  /* Publish Neighbor List */  
  neighborListPub.publish(neighborList);

  /* Prepare Message to send */
  UInt8* Message = PrepareMessageToSend();

  UInt8 data[2];
  data[0] = 0;
  data[1] = RobotID;
    
  /* Send message */
  m_pcRABAct->SetData(data);
  
  // Wait for any callbacks to be called.
  ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.1));

  // If we haven't heard from the subscriber in a while, set the speed to zero.
  if (stepsSinceCallback > stopWithoutSubscriberCount) {
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
