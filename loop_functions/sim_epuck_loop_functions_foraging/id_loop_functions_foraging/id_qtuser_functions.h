#ifndef ID_QTUSER_EPUCK_FUNCTIONS_H
#define ID_QTUSER_EPUCK_FUNCTIONS_H

#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_user_functions.h>
#include <argos3/plugins/robots/e-puck/simulator/epuck_entity.h>

using namespace argos;

class CIDQTUserEpuckFunctions : public CQTOpenGLUserFunctions {

public:

   CIDQTUserEpuckFunctions();

   virtual ~CIDQTUserEpuckFunctions() {}

   void Draw(CEPuckEntity& c_entity);
   
};

#endif
