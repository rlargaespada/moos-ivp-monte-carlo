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
ADD_SUBDIRECTORY(pLPAStar)
# ADD_SUBDIRECTORY(pExampleApp)
# ADD_SUBDIRECTORY(pXRelayTest)
# ADD_SUBDIRECTORY(pOdometry)
# ADD_SUBDIRECTORY(pPrimeFactor)
# ADD_SUBDIRECTORY(pPointAssign)
# ADD_SUBDIRECTORY(pGenPath)
# ADD_SUBDIRECTORY(pGenRescue)

##############################################################################
#                           END of CMakeLists.txt
##############################################################################

