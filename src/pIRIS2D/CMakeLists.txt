#--------------------------------------------------------
# The CMakeLists.txt for:                       pIRIS2D
# Author(s):                              Raul Largaespada
#--------------------------------------------------------

SET(SRC
  IRISPolygon.cpp
  IRISEllipse.cpp
  iris_mosek.cpp
  IRISProblem.cpp
  IRIS2D.cpp
  IRIS2D_Info.cpp
  main.cpp
)

# find dependencies
# find Eigen3 using find_package
find_package(Eigen3 3.4 REQUIRED NO_MODULE)

# search usual places for cdd library and include dir
find_library(CDD_LIBRARY NAMES cdd)
if(NOT CDD_LIBRARY)
  message(FATAL_ERROR "Could NOT find cdd")
else()
  message(STATUS "cdd library: ${CDD_LIBRARY}")
endif()

find_path(CDD_INCLUDE_DIR NAMES cdd/cdd.h)
if(NOT CDD_INCLUDE_DIR)
  message(FATAL_ERROR "Could NOT find cdd/cdd.h")
else()
  message(STATUS "cdd include directory: ${CDD_INCLUDE_DIR}")
endif()


# find mosek
# look for mosek include directory
FIND_PATH(MOSEK_INCLUDE_DIR mosek.h
  PATHS /usr/local/mosek/10.0/tools/platform/linux64x86/h/
        $ENV{HOME}/mosek/10.0/tools/platform/linux64x86/h/
)

# look for mosek shared library
SET(SEARCH_PATHS "${MOSEK_INCLUDE_DIR}" "${MOSEK_INCLUDE_DIR}/../bin" "${MOSEK_INCLUDE_DIR}/lib")
FIND_LIBRARY(MOSEK_LIBRARIES  NAMES mosek64 PATHS ${SEARCH_PATHS} NO_DEFAULT_PATH DPATH_SUFFIXES a lib dylib)

# report status
if(MOSEK_LIBRARIES AND MOSEK_INCLUDE_DIR)
  message(STATUS "mosek library: ${MOSEK_LIBRARIES}")
  message(STATUS "mosek include directory: ${MOSEK_INCLUDE_DIR}")
else()
  message(FATAL_ERROR "Could NOT find mosek")
endif()


# add pIRIS2D and link libraries, include dirs
ADD_EXECUTABLE(pIRIS2D ${SRC})

TARGET_LINK_LIBRARIES(pIRIS2D
   ${MOOS_LIBRARIES}
   geometry
   apputil
   mbutil
   m
   pthread
   Eigen3::Eigen
   ${CDD_LIBRARY}
   ${MOSEK_LIBRARIES})

TARGET_INCLUDE_DIRECTORIES(pIRIS2D PUBLIC
  ${CDD_INCLUDE_DIR}
  ${MOSEK_INCLUDE_DIR})
