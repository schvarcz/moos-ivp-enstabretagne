/************************************************************/
/*    FILE: Mapping.h
/*    ORGN: ENSTA Bretagne Robotics - moos-ivp-enstabretagne
/*    AUTH: Guilherme Schvarcz Franco
/*    DATE: 2016
/************************************************************/

#ifndef Mapping_HEADER
#define Mapping_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"

#include <vector>
#include <iterator>
#include "MBUtils.h"
#include "ACTable.h"
#include "SonarMapping.h"
#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;

class Mapping : public AppCastingMOOSApp
{
public:
    Mapping();
    ~Mapping();

protected: // Standard MOOSApp functions to overload
    bool OnNewMail(MOOSMSG_LIST &NewMail);
    bool Iterate();
    bool OnConnectToServer();
    bool OnStartUp();

protected: // Standard AppCastingMOOSApp functions to overload
    bool buildReport();
    void registerVariables();

protected: // Mapping functions


private: // Configuration variables


private: // State variables
    std::map<string, SonarMappingScanline> mapSonarsScanline;
    std::map<string, SonarMappingDistance> mapSonarsDistance;

};

#endif 
