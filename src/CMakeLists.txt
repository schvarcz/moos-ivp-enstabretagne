##############################################################################
# FILE:        moos-ivp-enstabretagne/src/CMakeLists.txt
# DATE:        2015
# DESCRIPTION: CMakeLists.txt file for the moos-ivp-enstabretagne source directory
##############################################################################

FILE(GLOB LOCAL_LIBRARY_DIRS ./lib)
INCLUDE_DIRECTORIES(${LOCAL_LIBRARY_DIRS})
ADD_SUBDIRECTORY(app)
ADD_SUBDIRECTORY(lib)

##############################################################################
#                           END of CMakeLists.txt
##############################################################################
