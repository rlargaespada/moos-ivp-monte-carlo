#=======================================================================
# FILE:  moos-ivp-extend/CMakeLists.txt
# DATE:  2012/07/24
# INFO:  Top-level CMakeLists.txt file for the moos-ivp-extend project
# NAME:  Maintained by Mike Benjamin - Original setup by Christian Convey
#        Chris Gagner, and tips borrowed from Dave Billin
#=======================================================================

CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
PROJECT( IVP_EXTEND )

set (CMAKE_CXX_STANDARD 11)

#=======================================================================
# Set the output directories for the binary and library files
#=======================================================================

GET_FILENAME_COMPONENT(IVP_EXTEND_BIN_DIR "${CMAKE_SOURCE_DIR}/bin"  ABSOLUTE )
GET_FILENAME_COMPONENT(IVP_EXTEND_LIB_DIR "${CMAKE_SOURCE_DIR}/lib"  ABSOLUTE )

SET( LIBRARY_OUTPUT_PATH      "${IVP_EXTEND_LIB_DIR}" CACHE PATH "" )
SET( ARCHIVE_OUTPUT_DIRECTORY "${IVP_EXTEND_LIB_DIR}" CACHE PATH "" )
SET( LIBRARY_OUTPUT_DIRECTORY "${IVP_EXTEND_LIB_DIR}" CACHE PATH "" )

SET( EXECUTABLE_OUTPUT_PATH    "${IVP_EXTEND_BIN_DIR}" CACHE PATH "" )
SET( RUNTIME_OUTPUT_DIRECTORY "${IVP_EXTEND_BIN_DIR}"  CACHE PATH "" )

#=======================================================================
# Find MOOS
#=======================================================================
find_package(MOOS 10.0)
INCLUDE_DIRECTORIES(${MOOS_INCLUDE_DIRS})

#=======================================================================
# Find the "moos-ivp" base directory
#=======================================================================

# Search for the moos-ivp folder
find_path( MOOSIVP_SOURCE_TREE_BASE
           NAMES build-ivp.sh build-moos.sh configure-ivp.sh
           PATHS "../moos-ivp" "../../moos-ivp" "../../moos-ivp/trunk/" "../moos-ivp/trunk/"
           DOC "Base directory of the MOOS-IvP source tree"
           NO_DEFAULT_PATH
)

if (NOT MOOSIVP_SOURCE_TREE_BASE)
    message("Please set MOOSIVP_SOURCE_TREE_BASE to  ")
    message("the location of the \"moos-ivp\" folder ")
    message( FATAL_ERROR "CMake will exit." )
endif()

#======================================================================
# Specify where to find IvP's headers and libraries...
#======================================================================

FILE(GLOB IVP_INCLUDE_DIRS ${MOOSIVP_SOURCE_TREE_BASE}/ivp/src/lib_* )
INCLUDE_DIRECTORIES(${IVP_INCLUDE_DIRS})

FILE(GLOB IVP_LIBRARY_DIRS ${MOOSIVP_SOURCE_TREE_BASE}/lib )
LINK_DIRECTORIES(${IVP_LIBRARY_DIRS})

#======================================================================
# Find External Libraries
#======================================================================

unset(GCS_MISSING_COMPONENTS)  # requires Eigen3, cddlib, mosek, mosek fusion
message(STATUS "Finding external libraries...")
list(APPEND CMAKE_MESSAGE_INDENT "  ")


# find Eigen3 using find_package
find_package(Eigen3 3.4 NO_MODULE)
if (TARGET Eigen3::Eigen)
   message(STATUS "Eigen3 include directory: ${EIGEN3_INCLUDE_DIR}")
else()
   message(WARNING "Cannot find Eigen3 package version 3.4!")
   list(APPEND GCS_MISSING_COMPONENTS Eigen3)
endif()


# find cdd library and include dir
find_library(CDD_LIBRARY NAMES cdd)
if(CDD_LIBRARY)
   message(STATUS "cdd library: ${CDD_LIBRARY}")
else()
   message(WARNING "Could NOT find cdd")
   list(APPEND GCS_MISSING_COMPONENTS "cddlib")
endif()

find_path(CDD_INCLUDE_DIR NAMES cdd/cdd.h)
if(CDD_INCLUDE_DIR)
   message(STATUS "cdd include directory: ${CDD_INCLUDE_DIR}")
else()
   message(WARNING "Could NOT find cdd/cdd.h")
   list(APPEND GCS_MISSING_COMPONENTS "cddlib headers")
endif()


# find mosek and mosek fusion libraries
# look for mosek include directory
FIND_PATH(MOSEK_INCLUDE_DIR mosek.h
  PATHS /usr/local/mosek/10.0/tools/platform/linux64x86/h/
        $ENV{HOME}/mosek/10.0/tools/platform/linux64x86/h/
)
SET(MOSEK_SEARCH_PATHS "${MOSEK_INCLUDE_DIR}" "${MOSEK_INCLUDE_DIR}/../bin" "${MOSEK_INCLUDE_DIR}/lib")
if(MOSEK_INCLUDE_DIR)
   message(STATUS "MOSEK include directory: ${MOSEK_INCLUDE_DIR}")
else()
   message(WARNING "Could NOT find MOSEK include directory!")
   list(APPEND GCS_MISSING_COMPONENTS "MOSEK include directory")
endif()

# look for mosek c library
FIND_LIBRARY(MOSEK_C_LIB  NAMES mosek64 PATHS ${MOSEK_SEARCH_PATHS} NO_DEFAULT_PATH DPATH_SUFFIXES a lib dylib)
if(MOSEK_C_LIB)
   message(STATUS "MOSEK C library: ${MOSEK_C_LIB}")
else()
   message(WARNING "Could NOT find MOSEK C library!")
   list(APPEND GCS_MISSING_COMPONENTS "MOSEK C library")
endif()

# look for mosek fusion library
FIND_LIBRARY(MOSEK_FUSION_LIB  NAMES fusion64 PATHS ${MOSEK_SEARCH_PATHS} NO_DEFAULT_PATH DPATH_SUFFIXES a lib dylib)
if(MOSEK_FUSION_LIB)
   message(STATUS "MOSEK fusion library: ${MOSEK_FUSION_LIB}")
else()
   message(WARNING "Could NOT find MOSEK fusion library!")
   list(APPEND GCS_MISSING_COMPONENTS "MOSEK fusion library")
endif()


# report final status
list(POP_BACK CMAKE_MESSAGE_INDENT)
if(GCS_MISSING_COMPONENTS)
   message(WARNING "cannot build pGCS2D or pIRIS2D due to the following missing dependencies:")
   message(WARNING "missing components: ${GCS_MISSING_COMPONENTS}")
else()
   message(STATUS "all components found")
endif()


#======================================================================
# Specify Compiler Flags
#======================================================================
IF( ${WIN32} )
   #---------------------------------------------
   # Windows Compiler Flags
   #---------------------------------------------
   IF(MSVC)
      # Flags for Microsoft Visual Studio
      SET( WALL_ON OFF CACHE BOOL 
         "tell me about all compiler warnings (-Wall) ")
      IF(WALL_ON)
         SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
      ENDIF(WALL_ON)
   ELSE(MSVC)
      # Other Windows compilers go here
   ENDIF(MSVC)

ELSE( ${WIN32} )
   #---------------------------------------------
   # Linux and Apple Compiler Flags
   #---------------------------------------------
   # Force -fPIC because gcc complains when we don't use it with x86_64 code.
   # Note sure why: -fPIC should only be needed for shared objects, and
   # AFAIK, CMake gets that right when building shared objects. -CJC
   SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -g -Wdeprecated-declarations")
   IF(CMAKE_COMPILER_IS_GNUCXX)
      # Flags for the GNU C++ Compiler
      SET( WALL_ON OFF CACHE BOOL 
         "tell me about all compiler warnings (-Wall) ")
      IF(WALL_ON)
         SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall" -C++11)
      ENDIF( WALL_ON)
   ELSE(CMAKE_COMPILER_IS_GNUCXX)
    
   ENDIF(CMAKE_COMPILER_IS_GNUCXX)

ENDIF( ${WIN32} )

#=======================================================================
# Add Subdirectories
#=======================================================================
ADD_SUBDIRECTORY( src )
