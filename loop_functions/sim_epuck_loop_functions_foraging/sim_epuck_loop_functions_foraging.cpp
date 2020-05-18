#include "sim_epuck_loop_functions_foraging.h"
#include <argos3/core/simulator/simulator.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/plugins/robots/e-puck/simulator/epuck_entity.h>
#include <argos3/plugins/simulator/entities/cylinder_entity.h>
#include <argos3/core/simulator/entity/floor_entity.h>
#include </home/eddie/catkin_ws/src/argos_ros_epuck_foraging/plugin/argos_ros_epuck_foraging/argos_ros_epuck_foraging.h>
#include <iostream>
#include <sstream>
#include <algorithm>

std::string GetPuckColorBasedOnID(std::string PuckID){
  std::string color = PuckID.substr(0, PuckID.size() - 6);
  return color;
}

int GetPuckTypeBasedOnColor(std::string Color){
  if(!Color.compare("green"))
    return 1;
  else if(!Color.compare("red"))
    return 2;
  else if(!Color.compare("blue"))
    return 3;
  else if(!Color.compare("yellow"))
    return 4;
  else if(!Color.compare("magenta"))
    return 5;
  else if(!Color.compare("cyan"))
    return 6;
  else if(!Color.compare("white"))
    return 7;
  else if(!Color.compare("orange"))
    return 8;
}

/****************************************/
/****************************************/
CSimEpuckLoopFunctionsForaging::CSimEpuckLoopFunctionsForaging():
   m_pcFloor(NULL),
   m_pcRNG(NULL),
   bigRange(0.0f,30000.0f),
   arenaCornerSideX(0.0f, 2.5f),
   arenaCornerSideY(0.0f, 2.5f)
   {}

/****************************************/
/****************************************/
void CSimEpuckLoopFunctionsForaging::Init(TConfigurationNode& t_node) {
   try {
      /* Get a pointer to the floor entity */
      m_pcFloor = &GetSpace().GetFloorEntity();
      /* Create a new RNG */
      m_pcRNG = CRandom::CreateRNG("argos");

      /* Go through the nodes */
      TConfigurationNodeIterator itDistr;
      for(itDistr = itDistr.begin(&t_node);
          itDistr != itDistr.end();
          ++itDistr) {

	  /* Get current node */
          TConfigurationNode& tDistr = *itDistr;

          // General simulation related parameters
          if(itDistr->Value() == "sim"){
	    // Get the number of robots in the experiment
	    GetNodeAttribute(tDistr, "number_of_robots", total_robot_number);
            // Get the ticks per second
            GetNodeAttribute(tDistr, "ticks_per_second", TicksPerSecond);
	    // Get the length of the experiment (timeout threshold)
            GetNodeAttribute(tDistr, "experiment_length", ExperimentLength);
            /* Get the output file name from XML */
            GetNodeAttribute(tDistr, "output", m_strOutput);
            /* Get the output file name from XML */
            GetNodeAttribute(tDistr, "adjacency_matrix", m_strAdjacencyMatrix);
            /* Open the file, erasing its contents */
            m_cOutput.open(m_strOutput.c_str(), std::ios_base::trunc | std::ios_base::out);
            m_cAdjacencyMatrix.open(m_strAdjacencyMatrix.c_str(), std::ios_base::trunc | std::ios_base::out);
            m_cAdjacencyMatrix << "[" << std::endl;
          }
          // Floor Color related parameters
          else if(itDistr->Value() == "floor_color") {
            GetNodeAttribute(tDistr, "num_of_cells", NumberOfCells);
            GetNodeAttribute(tDistr, "cell_dimension", CellDimension);
            GetNodeAttribute(tDistr, "num_of_colors", NumberOfColors);
            GetNodeAttribute(tDistr, "size", ArenaSize);
          }
        }
      }
      catch(CARGoSException& ex) {
        THROW_ARGOSEXCEPTION_NESTED("Error parsing loop functions!", ex);
      }

   //ROS Initialization
   int argc = 0;
   char *argv = (char *) "";
   ros::init(argc, &argv, "loop_function");
   ros::NodeHandle nodeHandle;

   CSpace::TMapPerType& m_cEpucks = GetSpace().GetEntitiesByType("epuck");     
   for(CSpace::TMapPerType::iterator it = m_cEpucks.begin(); it != m_cEpucks.end();++it)
     { 
       CEPuckEntity& cEpuck = *any_cast<CEPuckEntity*>(it->second);
       CArgosRosEpuck& cController = dynamic_cast<CArgosRosEpuck&>(cEpuck.GetControllableEntity().GetController());
       stringstream positionTopic;
       positionTopic << "/epuck_" << cController.GetId().substr(cController.GetId().length()-2) << "/position";
       PosePubVector.push_back(nodeHandle.advertise<geometry_msgs::Pose>(positionTopic.str(), 10));
     }

   // FINISHING CONDITIONS VARIABLES
   robots_with_completed_merkle = 0;

   // PLACING ROBOTS IN A CERTAIN CONFIGURATION
   PlaceEpucksInACircleAtTheArenaCenter();
   
   colorOfCell = new UInt32[NumberOfColors];   
   grid = new UInt32[NumberOfCells];
   
   // Color distribution functions for central nest problems
   grid = FillColorGridWithCentralNest(grid);
   grid = GenerateRandomColorGrid(grid);

   // Move LEDs to the center of color tiles   
   CSpace::TMapPerType& m_pucks = GetSpace().GetEntitiesByType("cylinder");
   for(CSpace::TMapPerType::iterator it = m_pucks.begin(); it != m_pucks.end();++it){
     CCylinderEntity& cPuck = *any_cast<CCylinderEntity*>(it->second);
     std::string color =  GetPuckColorBasedOnID(cPuck.GetId());
     int puck_id = GetPuckTypeBasedOnColor(color);
     UInt32 GridDimensionX = (UInt32)((Real)ArenaSize.GetX()/(Real)CellDimension);
     UInt32 GridDimensionY = (UInt32)((Real)ArenaSize.GetY()/(Real)CellDimension);     

     for(int i = 0; i < (GridDimensionX*GridDimensionY); i++)
       if (grid[i] == puck_id)
	 {
	   Real x = ((i % GridDimensionX ) * CellDimension) + CellDimension/2;
	   Real y = ((i / GridDimensionY ) * CellDimension) + CellDimension/2;
	   CVector3 cNewPosition = CVector3(x,y,0);
	   CQuaternion cNewOrientation = cPuck.GetEmbodiedEntity().GetOriginAnchor().Orientation;
	   cPuck.GetEmbodiedEntity().MoveTo(cNewPosition, cNewOrientation);
	 }
   }
}

UInt32* CSimEpuckLoopFunctionsForaging::GenerateRandomColorPattern(UInt32 *grid){
UInt32 GridDimension = (UInt32)((Real)ArenaSize.GetX()/(Real)CellDimension);
for (int i = 0; i < NumberOfCells; i+=GridDimension+1)
    grid[i] = 1; 
    
 return grid;
}

UInt32* CSimEpuckLoopFunctionsForaging::GenerateRandomColorGrid(UInt32 *grid){
 UInt32 GridDimensionX = (UInt32)((Real)ArenaSize.GetX()/(Real)CellDimension);
 UInt32 GridDimensionY = (UInt32)((Real)ArenaSize.GetY()/(Real)CellDimension);
 UInt32 NumberOfCellsInArena = GridDimensionX * GridDimensionY; 

 CRange<UInt32> GridIds(0,NumberOfCellsInArena-1);
  
  for (int k = 1; k <= NumberOfColors ; k++)
    {
      int j = m_pcRNG->Uniform(GridIds);
    
      while(j == (NumberOfCellsInArena-1)/2 || grid[j] != 7)
	j = m_pcRNG->Uniform(GridIds);

      grid[j] = k;
    }
  
  return grid;
}

UInt32* CSimEpuckLoopFunctionsForaging::FillColorGridWithInitialDistribution(UInt32 *colorOfCell, int NumberOfColors, UInt32 *grid){
  int k = 0;
  /* Generate random color for each cell according with the choosen probabilities*/
  for ( int i = 0; i < NumberOfColors; i++ )
    for( int j = 0; j < colorOfCell[i] ; j++,k++ )
      grid[k] = i;

  return grid;
}

UInt32* CSimEpuckLoopFunctionsForaging::FillColorGridWithCentralNest(UInt32 *grid){

  UInt32 GridDimension = (UInt32)((Real)ArenaSize.GetX()/(Real)CellDimension);
  UInt32 GridCenter = GridDimension/2;
  UInt32 NestDimension = 0;

  colorOfCell[0] = 7; // White
  colorOfCell[1] = 9; // Black
  
  // Initialize array with background color
  std::fill_n(grid,NumberOfCells,colorOfCell[0]);
  
  for (int i = GridCenter-NestDimension; i <= GridCenter+NestDimension; i++)
      for (int j = GridCenter-NestDimension; j <= GridCenter+NestDimension; j++)
  	grid[ i * GridDimension + j ] = colorOfCell[1];
  
  return grid;
}

UInt32* CSimEpuckLoopFunctionsForaging::GenerateEvenColorDistributions(UInt32 *colorOfCell, int NumberOfColors){
  // Even distribution of colors among channels
  for (int i = 0; i < NumberOfColors; i++)
    colorOfCell[i] = NumberOfCells/NumberOfColors;

  return colorOfCell;
}

CColor CSimEpuckLoopFunctionsForaging::GetFloorColor(const CVector2& c_pos_on_floor) {
  UInt32 x,y,i;
  UInt32 GridDimension;

  if ((c_pos_on_floor.GetX() > ArenaSize.GetY()) || (c_pos_on_floor.GetY() > ArenaSize.GetX()))
    return CColor::GREEN;

  x = (UInt32)(((Real)c_pos_on_floor.GetX())/(Real)CellDimension);
  y = (UInt32)(((Real)c_pos_on_floor.GetY())/(Real)CellDimension);

  GridDimension = (UInt32)((Real)ArenaSize.GetX()/(Real)CellDimension);
  i=(UInt32) (y*(GridDimension) + x);

  switch (grid[i])
    {
    case 1:
      return CColor::GREEN;
    case 2:
      return CColor::RED;
    case 3:
      return CColor::BLUE;
    case 4:
      return CColor::YELLOW;
    case 5:
      return CColor::MAGENTA;
    case 6:
      return CColor::CYAN;
    case 7:
      return CColor::WHITE;
    case 8:
      return CColor::ORANGE;
    case 9:
      return CColor::BLACK;
    }
}

void CSimEpuckLoopFunctionsForaging::ReplaceClosePucksToArenaCorner()
{
    double distance_threshold = 1;

    // Get all cylinder entities
    CSpace::TMapPerType& m_pucks = GetSpace().GetEntitiesByType("cylinder");
    for(CSpace::TMapPerType::iterator it = m_pucks.begin(); it != m_pucks.end();++it){
      /* Get handle to entity and controller */
      CCylinderEntity& cPuck = *any_cast<CCylinderEntity*>(it->second);
      Real x1 = cPuck.GetEmbodiedEntity().GetOriginAnchor().Position.GetX();
      Real y1 = cPuck.GetEmbodiedEntity().GetOriginAnchor().Position.GetY();
      
      double x = x1 -(ArenaSize.GetX()/2); 
      double y = y1 -(ArenaSize.GetY()/2);      
      double dist = sqrt(pow(x, 2) + pow(y, 2));       //calculating Euclidean distance

      while(dist < distance_threshold)
	{
	  CVector3 cNewPosition = CVector3(m_pcRNG->Uniform(arenaCornerSideX),m_pcRNG->Uniform(arenaCornerSideY),0);
	  CQuaternion cNewOrientation = cPuck.GetEmbodiedEntity().GetOriginAnchor().Orientation;
	  cPuck.GetEmbodiedEntity().MoveTo(cNewPosition, cNewOrientation);
	  
	  Real x1 = cPuck.GetEmbodiedEntity().GetOriginAnchor().Position.GetX();
	  Real y1 = cPuck.GetEmbodiedEntity().GetOriginAnchor().Position.GetY();
	  double x = x1 -(ArenaSize.GetX()/2); 
	  double y = y1 -(ArenaSize.GetY()/2);      
	  dist = sqrt(pow(x, 2) + pow(y, 2));       //calculating Euclidean distance
	  
	}      
    }
}

/****************************************/
/****************************************/
void CSimEpuckLoopFunctionsForaging::PreStep() {
  // Get all e-puck entities
  CSpace::TMapPerType& m_cEpucks = GetSpace().GetEntitiesByType("epuck");
    
  for(CSpace::TMapPerType::iterator it = m_cEpucks.begin(); it != m_cEpucks.end();++it){
    /* Get handle to entity and controller */
    CEPuckEntity& cEpuck = *any_cast<CEPuckEntity*>(it->second);

    // Populate the Pose message
    geometry_msgs::Pose EpuckPose;
    EpuckPose.position.x = cEpuck.GetEmbodiedEntity().GetOriginAnchor().Position.GetX();
    EpuckPose.position.y = cEpuck.GetEmbodiedEntity().GetOriginAnchor().Position.GetY();
    EpuckPose.position.z = cEpuck.GetEmbodiedEntity().GetOriginAnchor().Position.GetZ();

    EpuckPose.orientation.x = cEpuck.GetEmbodiedEntity().GetOriginAnchor().Orientation.GetX();
    EpuckPose.orientation.y = cEpuck.GetEmbodiedEntity().GetOriginAnchor().Orientation.GetY();
    EpuckPose.orientation.z = cEpuck.GetEmbodiedEntity().GetOriginAnchor().Orientation.GetZ();
    EpuckPose.orientation.w = cEpuck.GetEmbodiedEntity().GetOriginAnchor().Orientation.GetW();
    
    // Publish message for specific epuck
    PosePubVector[std::distance(m_cEpucks.begin(), it)].publish(EpuckPose);
  }
}


/****************************************/
/****************************************/
void CSimEpuckLoopFunctionsForaging::PostStep() 
{
  if (!(GetSpace().GetSimulationClock() % TicksPerSecond)){

    // ROBOT LOOP
    CSpace::TMapPerType& m_cEpucks = GetSpace().GetEntitiesByType("epuck");
    
    // Add the time as a primary key to the JSON object
    adjacency_json["time"] = GetSpace().GetSimulationClock()/TicksPerSecond;

    // Reset the counter of robots with all the leafs
    robots_with_completed_merkle = 0;
    
    for(CSpace::TMapPerType::iterator it = m_cEpucks.begin(); it != m_cEpucks.end();++it){
      /* Get handle to e-pucks entity and controller */
      CEPuckEntity& cEpuck = *any_cast<CEPuckEntity*>(it->second);
      CArgosRosEpuck& cController = dynamic_cast<CArgosRosEpuck&>(cEpuck.GetControllableEntity().GetController());

      /* Get current robot ID */
      int robotID =  std::distance(m_cEpucks.begin(), it);
      
      /* Get neighbor data */
      argos_ros_epuck::NeighborList& neighborList = cController.GetNeighborListForLoopFunction();

      /* Get Merkle Tree data */
      argos_ros_epuck::MerkleLeafList& completedMerkleList = cController.GetCompletedMerkleTreeForLoopFunction();

      /* Get Current RGB Color in the robot's LED ring */
      CColor& CurrentRGBColor = cController.GetCurrentRGBColor();

      // Extract the data to the vectors from the NeighborList message
      std::vector<int> neighborsID;
      std::vector<float> neighborsRanges;

      for (int i=0; i<neighborList.n; i++){
        neighborsID.push_back(neighborList.neighbors[i].ID);
        neighborsRanges.push_back(neighborList.neighbors[i].range);
      }

      std::vector<int> received_leafs_id;  
      std::vector<int> completed_by_current_robot;  
      std::vector<int> received_from_robot;
          
      for (int i = 0; i < completedMerkleList.n; i++)
        if (completedMerkleList.leafs[i].received){
            received_leafs_id.push_back(i);
            received_from_robot.push_back(completedMerkleList.leafs[i].from_robot);
          }
        else
          if (completedMerkleList.leafs[i].completed)
              completed_by_current_robot.push_back(i);

      // Push an object to the json data structure
      // robot network
      adjacency_json["robots"].push_back({{"id", robotID},
                                        // received leafs from other robots
                                        {"mk",
                                        {{"leaf_received", received_leafs_id},
                                        {"from_robot", received_from_robot},
                                        {"by_robot", completed_by_current_robot}}},
                                        
                                        {"neighbors",
                                        {{"id", neighborsID},
                                        {"range", neighborsRanges}}},
                                                                              
                                        });

  
      /* Merkle Tree Checking Conditions */
      // How many leaves did this robot complete?
      int completed_leafs = 0;
      for (int i = 0; i < completedMerkleList.n; i++)
        if (completedMerkleList.leafs[i].completed)
        completed_leafs++;

      // If it completed all we increase the counter
      if(completedMerkleList.n != 0)
        if(completed_leafs == completedMerkleList.n)
          robots_with_completed_merkle++;

      ////////////////
      DisableLEDBasedOnColor(CurrentRGBColor);
    }

    // Dump content of adjacency json file
    // Clear the json object
    if (robots_with_completed_merkle!=total_robot_number && (UInt32)GetSpace().GetSimulationClock() != (ExperimentLength*TicksPerSecond))
      m_cAdjacencyMatrix << adjacency_json.dump(2) << "," << std::endl;
    else
      m_cAdjacencyMatrix << adjacency_json.dump(2) << std::endl;

    if ((UInt32)GetSpace().GetSimulationClock() == (ExperimentLength*TicksPerSecond))
      m_cAdjacencyMatrix << "]" << std::endl;
    
    adjacency_json.clear();

    /* Output stuff to file */
    m_cOutput << GetSpace().GetSimulationClock()/TicksPerSecond << "\t" << robots_with_completed_merkle << std::endl;

  }
 
  ros::spinOnce();  
}

void CSimEpuckLoopFunctionsForaging::DisableLEDBasedOnColor(CColor Color){

  CSpace::TMapPerType& m_leds = GetSpace().GetEntitiesByType("led");
  for(CSpace::TMapPerType::iterator it = m_leds.begin(); it != m_leds.end();++it){
    CLEDEntity& cLed = *any_cast<CLEDEntity*>(it->second);
    if (cLed.GetColor() == Color)
      cLed.SetColor(CColor::BLACK);
  }  
}


void CSimEpuckLoopFunctionsForaging::PlaceEpucksInACircleAtTheArenaCenter()
{

  // Get all epuck entities
  CSpace::TMapPerType& m_cEpucks = GetSpace().GetEntitiesByType("epuck");
    
  int number_of_robots = std::distance(m_cEpucks.begin(), m_cEpucks.end());
  float radius = 0.30;
  double slice = 2 * 3.14159 / number_of_robots;

  for(CSpace::TMapPerType::iterator it = m_cEpucks.begin(); it != m_cEpucks.end(); ++it){
      double angle = slice * std::distance(m_cEpucks.begin(), it);
      float X = (1.25 + radius * cos(angle));
      float Y = (1.25 + radius * sin(angle));
      CVector3 cNewPosition = CVector3(X,Y,0);
      CEPuckEntity& cEpuck = *any_cast<CEPuckEntity*>(it->second);
      CQuaternion cNewOrientation = cEpuck.GetEmbodiedEntity().GetOriginAnchor().Orientation;
      if(std::distance(m_cEpucks.begin(), it) < number_of_robots/2)
	cNewOrientation.SetZ(angle);
      else
	{ angle = angle/(2 * 3.14159);
	  cNewOrientation.SetZ(-angle);
	  cNewOrientation.SetW(angle);
	}
      
      cEpuck.GetEmbodiedEntity().MoveTo(cNewPosition, cNewOrientation);
    }    
}

bool CSimEpuckLoopFunctionsForaging::IsExperimentFinished(){
  
  // If the number of completed merkle trees are the same
  // as the number of total robots in the simulation we finish
  // the experiment. Experiment timeout is handled by the .argos file
  if (robots_with_completed_merkle==total_robot_number){
    // Close the data files
    m_cAdjacencyMatrix << "]" << std::endl;
    m_cAdjacencyMatrix.close();
    m_cOutput.close();
    return true;
  }
  else
    return false;

  return false;
}

/****************************************/
/****************************************/
void CSimEpuckLoopFunctionsForaging::Reset() {
  /* Close the files */
  m_cOutput.close();
  m_cAdjacencyMatrix.close();
  /* Open the file, erasing its contents */
  m_cOutput.open(m_strOutput.c_str(), std::ios_base::trunc | std::ios_base::out);
  m_cAdjacencyMatrix.open(m_strAdjacencyMatrix.c_str(), std::ios_base::trunc | std::ios_base::out);  
}

/****************************************/
/****************************************/
void CSimEpuckLoopFunctionsForaging::Destroy() {
   /* Close the files */
   m_cOutput.close();
   m_cAdjacencyMatrix.close();
}


/****************************************/
/***************************************/
REGISTER_LOOP_FUNCTIONS(CSimEpuckLoopFunctionsForaging, "sim_epuck_loop_functions_foraging")
