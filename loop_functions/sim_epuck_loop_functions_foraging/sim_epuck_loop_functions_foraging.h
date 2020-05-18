#ifndef SIM_EPUCK_LOOP_FUNCTION_FORAGING_H
#define SIM_EPUCK_LOOP_FUNCTION_FORAGING_H

#include </home/eddie/argos3-dist/include/argos3/core/simulator/loop_functions.h>
#include <argos3/core/simulator/entity/floor_entity.h>
#include <argos3/core/utility/math/range.h>
#include <argos3/core/utility/math/rng.h>
#include <nlohmann/json.hpp>
#include <ros/ros.h>

using json = nlohmann::json;
using namespace argos;
using namespace std;

class CSimEpuckLoopFunctionsForaging : public CLoopFunctions {

public:

   CSimEpuckLoopFunctionsForaging();
   virtual ~CSimEpuckLoopFunctionsForaging() {}

   virtual void Init(TConfigurationNode& t_tree);
   virtual void Reset();
   virtual void Destroy();
   virtual void PreStep();
   virtual void PostStep();
   virtual bool IsExperimentFinished();
   virtual void PlaceEpucksInACircleAtTheArenaCenter();
   virtual CColor GetFloorColor(const CVector2& c_pos_on_floor);

   // Generates the color distribution
   UInt32* GenerateEvenColorDistributions(UInt32 *colorOfCell, int NumberOfColors);
   UInt32* FillColorGridWithInitialDistribution(UInt32 *colorOfCell,int NumberOfColors,UInt32 *grid);
   UInt32* FillColorGridWithCentralNest(UInt32 *grid);
   UInt32* GenerateRandomColorGrid(UInt32 *grid);
   UInt32* GenerateRandomColorPattern(UInt32 *grid);

   // Modifies the position of pucks too close to nest
   void ReplaceClosePucksToArenaCorner();

   void DisableLEDBasedOnColor(CColor Color);
   
private:

   UInt32 *colorOfCell, *grid;
   CRange<Real> bigRange;
   CRange<Real> arenaCornerSideX;
   CRange<Real> arenaCornerSideY;

   // Framework related variables
   UInt32 TicksPerSecond, ExperimentLength;

   // Floor related variables
   UInt32 NumberOfCells, NumberOfColors;
   Real CellDimension;

   // Arena relates variables
   CVector3 ArenaSize;  
  
  CFloorEntity* m_pcFloor;
  CRandom::CRNG* m_pcRNG;

  std::string m_strAdjacencyMatrix;
  std::ofstream m_cAdjacencyMatrix;
  json adjacency_json;

  std::string m_strOutput;
  std::ofstream m_cOutput;

  // EXPERIMENT FINISHING VARIABLES
  int robots_with_completed_merkle;
  int total_robot_number;
  
  // ROS related variables
  // Postion publisher
  std::vector<ros::Publisher> PosePubVector;
};

#endif
