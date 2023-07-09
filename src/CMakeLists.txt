##############################################################################
# FILE:        moos-ivp-rlargaes/src/CMakeLists.txt
# DATE:        2022/02/13
# DESCRIPTION: CMakeLists.txt file for the moos-ivp-rlargaes source directory
##############################################################################


#============================================================================
# Add the libraries in the current directory to the include path
#============================================================================
FILE(GLOB LOCAL_LIBRARY_DIRS ./lib_*)
INCLUDE_DIRECTORIES(${LOCAL_LIBRARY_DIRS})

#============================================================================
# List the subdirectories to build...
#============================================================================
# ADD_SUBDIRECTORY(lib_behaviors-test)
ADD_SUBDIRECTORY(pNullPlanner)
# ADD_SUBDIRECTORY(pLPAStar)
ADD_SUBDIRECTORY(pDStarLite)
ADD_SUBDIRECTORY(pGCS2D)
ADD_SUBDIRECTORY(pIRIS2D)
ADD_SUBDIRECTORY(uEvalPlanner)
ADD_SUBDIRECTORY(uFldObsMonteCarloSim)

##############################################################################
#                           END of CMakeLists.txt
##############################################################################

