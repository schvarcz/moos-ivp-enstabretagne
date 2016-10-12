#--------------------------------------------------------
# The CMakeLists.txt for:                      uSimMarine
# Author(s):                                Mike Benjamin
#--------------------------------------------------------

# Set System Specific Libraries
if (${WIN32})
  SET(SYSTEM_LIBS
    wsock32)
else (${WIN32})
  SET(SYSTEM_LIBS
    m
    pthread)
endif (${WIN32})

SET(SRC
   UBM_MOOSApp.cpp
   UBM_Model.cpp
   UBM_Info.cpp
   SimEngineBoat.cpp
   ThrustMap.cpp
   main.cpp
)

ADD_EXECUTABLE(uSimMarineBoat ${SRC})

TARGET_LINK_LIBRARIES(uSimMarineBoat 
  ${MOOS_LIBRARIES}
  ${MOOSGeodesy_LIBRARIES}
  contacts
  geometry 
  apputil
  mbutil
  ${SYSTEM_LIBS})



