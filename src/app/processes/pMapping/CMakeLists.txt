#--------------------------------------------------------
# The CMakeLists.txt for: pMapping
# ENSTA Bretagne Robotics - moos-ivp-enstabretagne
# Author(s): 
#--------------------------------------------------------

SET(SRC
  Mapping.cpp
  SonarMapping.cpp
  main.cpp
)

ADD_EXECUTABLE(pMapping ${SRC})

TARGET_LINK_LIBRARIES(pMapping
   ${MOOS_LIBRARIES}
   xml-documentation
   apputil
   mbutil
   m
   pthread
   opencv_core
   opencv_imgproc
   opencv_highgui)
