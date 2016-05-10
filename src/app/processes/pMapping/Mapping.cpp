/************************************************************/
/*    FILE: Mapping.cpp
/*    ORGN: ENSTA Bretagne Robotics - moos-ivp-enstabretagne
/*    AUTH: Guilherme Schvarcz Franco
/*    DATE: 2016
/************************************************************/

#include "Mapping.h"

Mapping::Mapping()
{
}

Mapping::~Mapping()
{
}

bool Mapping::OnNewMail(MOOSMSG_LIST &NewMail)
{
    AppCastingMOOSApp::OnNewMail(NewMail);

    MOOSMSG_LIST::iterator p;
    for(p = NewMail.begin() ; p != NewMail.end() ; p++)
    {
        CMOOSMsg &msg = *p;

#if 0 // Keep these around just for template
        string comm  = msg.GetCommunity();
        double dval  = msg.GetDouble();
        string sval  = msg.GetString();
        string msrc  = msg.GetSource();
        double mtime = msg.GetTime();
        bool   mdbl  = msg.IsDouble();
        bool   mstr  = msg.IsString();
#endif

        if(msg.GetKey().find("_RAW_DATA") != string::npos)
        {
            double bearing;
            MOOSValFromString(bearing, msg.GetString(), "bearing", true);
            bearing = MOOSDeg2Rad(bearing);

            int nRows, nCols;
            std::vector<double> scanline;
            MOOSVectorFromString(msg.GetString(), scanline, nRows, nCols);

            string key = msg.GetKey().substr(0,msg.GetKey().find("_RAW_DATA"));
            if(mapSonarsScanline.find(key) != mapSonarsScanline.end())
            {
                mapSonarsScanline[key].Update(bearing, scanline);
            }

        }
        else if(msg.GetKey().find("_DISTANCE") != string::npos)
        {
            double bearing, distance;
            MOOSValFromString(bearing, msg.GetString(), "bearing", true);
            MOOSValFromString(distance, msg.GetString(), "distance", true);
            bearing = MOOSDeg2Rad(bearing);


            string key = msg.GetKey().substr(0,msg.GetKey().find("_DISTANCE"));
            if(mapSonarsDistance.find(key) != mapSonarsDistance.end())
            {
                mapSonarsDistance[key].Update(bearing,distance);
            }

        }
        else if(msg.GetKey().find("_CONNECTED") != string::npos)
        {
            string key = msg.GetKey().substr(0,msg.GetKey().find("_CONNECTED"));
            if(mapSonarsScanline.find(key) != mapSonarsScanline.end())
            {
                mapSonarsScanline[key].isConnected = (msg.GetString() == "true");
                mapSonarsDistance[key].isConnected = (msg.GetString() == "true");
            }

        }

        // **************** MAPPING PARAMS ************************
        else if(msg.GetKey().find("_CONTRAST") != string::npos)
        {
            string key = msg.GetKey().substr(0,msg.GetKey().find("_CONTRAST"));
            if(mapSonarsScanline.find(key) != mapSonarsScanline.end())
            {
                mapSonarsScanline[key].mContrast = msg.GetDouble();
            }

        }
        else if(msg.GetKey().find("_SCAN_LENGTH") != string::npos)
        {
            string key = msg.GetKey().substr(0,msg.GetKey().find("_SCAN_LENGTH"));
            if(mapSonarsDistance.find(key) != mapSonarsDistance.end())
            {
                mapSonarsDistance[key].mScanLength = (int)msg.GetDouble();
            }
        }
        else if(msg.GetKey().find("_FOV") != string::npos)
        {
            string key = msg.GetKey().substr(0,msg.GetKey().find("_FOV"));
            if(mapSonarsScanline.find(key) != mapSonarsScanline.end())
            {
                mapSonarsScanline[key].mFOV = MOOSDeg2Rad(msg.GetDouble());
            }
        }


        else if(msg.GetKey() != "APPCAST_REQ") // handle by AppCastingMOOSApp
            reportRunWarning("Unhandled Mail: " + msg.GetKey());
    }

    return true;
}

bool Mapping::OnConnectToServer()
{
    registerVariables();
    return true;
}

bool Mapping::Iterate()
{
    AppCastingMOOSApp::Iterate();

    //Iterate sonars
    for( map<string,SonarMappingScanline>::iterator it = mapSonarsScanline.begin(); it != mapSonarsScanline.end();it++)
    {
        it->second.Iterate();
    }
    for( map<string,SonarMappingDistance>::iterator it = mapSonarsDistance.begin(); it != mapSonarsDistance.end();it++)
    {
        it->second.Iterate();
    }


    for( map<string,SonarMappingScanline>::iterator it = mapSonarsScanline.begin(); it != mapSonarsScanline.end();it++)
    {
        if (!it->second.SonarMap().empty())
        {
            string msgParams = "";
            msgParams +=
                    "WIDTH=" + intToString(it->second.SonarMap().cols) + "," +
                    "HEIGHT=" + intToString(it->second.SonarMap().rows) + "," +
                    "DEPTH=" + intToString(it->second.SonarMap().depth());
            Notify(it->first+"_SCANLINE_MAP_PARAMS", msgParams);
            Notify(it->first+"_SCANLINE_MAP", it->second.SonarMap().data, it->second.SonarMap().channels() * it->second.SonarMap().rows * it->second.SonarMap().cols);
        }
    }
    for( map<string,SonarMappingDistance>::iterator it = mapSonarsDistance.begin(); it != mapSonarsDistance.end();it++)
    {
        if (!it->second.SonarMap().empty())
        {
            string msgParams = "";
            msgParams +=
                    "WIDTH=" + intToString(it->second.SonarMap().cols) + "," +
                    "HEIGHT=" + intToString(it->second.SonarMap().rows) + "," +
                    "DEPTH=" + intToString(it->second.SonarMap().depth());
            Notify(it->first+"_DISTANCE_MAP_PARAMS", msgParams);
            Notify(it->first+"_DISTANCE_MAP", it->second.SonarMap().data, it->second.SonarMap().channels() * it->second.SonarMap().rows * it->second.SonarMap().cols);
        }
    }
    waitKey(3);
    AppCastingMOOSApp::PostReport();
    return true;
}

bool Mapping::OnStartUp()
{
    AppCastingMOOSApp::OnStartUp();

    STRING_LIST sParams;
    m_MissionReader.EnableVerbatimQuoting(false);
    string lastSonar = "";
    string lastImage = "";
    if(!m_MissionReader.GetConfiguration(GetAppName(), sParams))
        reportConfigWarning("No config block found for " + GetAppName());

    STRING_LIST::iterator p;
    sParams.reverse();
    for(p = sParams.begin() ; p != sParams.end() ; p++)
    {
        string orig  = *p;
        string line  = *p;
        string param = toupper(biteStringX(line, '='));
        string value = line;
        bool handled = false;

        if (param == "SONAR_VAR_NAME")
        {
            if (mapSonarsScanline.find(value) == mapSonarsScanline.end())
            {
                mapSonarsScanline.insert ( std::pair<string, SonarMappingScanline> (value,SonarMappingScanline()) );
                mapSonarsDistance.insert ( std::pair<string, SonarMappingDistance> (value,SonarMappingDistance()) );
                lastSonar = value;
                handled = true;
            }
        }

        else if (param == "SONAR_SCAN_LENGTH")
        {
            if (lastSonar != "")
            {
                mapSonarsDistance[lastSonar].mScanLength = atoi((char*)value.c_str());
                handled = true;
            }
        }
        else if (param == "SONAR_FOV")
        {
            if (lastSonar != "")
            {
                mapSonarsScanline[lastSonar].mFOV = MOOSDeg2Rad(atof((char*)value.c_str()));
                handled = true;
            }
        }
        else if (param == "SONAR_CONTRAST")
        {
            if (lastSonar != "")
            {
                mapSonarsScanline[lastSonar].mContrast = atof((char*)value.c_str());
                handled = true;
            }
        }

        if(!handled)
            reportUnhandledConfigWarning(orig);
    }

    registerVariables();
    return true;
}

void Mapping::registerVariables()
{
    AppCastingMOOSApp::RegisterVariables();

    for( map<string,SonarMappingScanline>::iterator it = mapSonarsScanline.begin(); it != mapSonarsScanline.end();it++)
    {
        Register(it->first+"_RAW_DATA", 0);
        Register(it->first+"_DISTANCE", 0);
        Register(it->first+"_CONNECTED", 0);
        Register(it->first+"_CONTRAST", 0);
        Register(it->first+"_FOV", 0);
        Register(it->first+"_SCAN_LENGTH", 0);
    }
}

bool Mapping::buildReport() 
{
#if 0 // Keep these around just for template
    ACTable actab(4);
    actab << "Alpha | Bravo | Charlie | Delta";
    actab.addHeaderLines();
    actab << "one" << "two" << "three" << "four";
    m_msgs << actab.getFormattedString();
#endif

    m_msgs << std::endl;
    for( map<string,SonarMappingScanline>::iterator it = mapSonarsScanline.begin(); it != mapSonarsScanline.end();it++)
    {
        m_msgs <<  it->first << std::endl;
        m_msgs <<  it->second.toString() << std::endl << std::endl;
    }
    m_msgs << std::endl;
    for( map<string,SonarMappingDistance>::iterator it = mapSonarsDistance.begin(); it != mapSonarsDistance.end();it++)
    {
        m_msgs <<  it->first << std::endl;
        m_msgs <<  it->second.toString() << std::endl << std::endl;
    }

    return true;
}
