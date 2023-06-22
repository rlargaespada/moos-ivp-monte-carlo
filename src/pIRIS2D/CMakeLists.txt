#--------------------------------------------------------
# The CMakeLists.txt for:                       pIRIS2D
# Author(s):                              Raul Largaespada
#--------------------------------------------------------

SET(SRC
  IRIS2D.cpp
  IRIS2D_Info.cpp
  main.cpp
)

find_package(Eigen3 REQUIRED NO_MODULE)

find_library(CDD_LIBRARY NAMES cdd)

if(NOT CDD_LIBRARY)
  message(FATAL_ERROR "Could NOT find cdd")
endif()

find_path(CDD_INCLUDE_DIR NAMES cdd/cdd.h)

if(NOT CDD_INCLUDE_DIR)
  message(FATAL_ERROR "Could NOT find cdd/cdd.h")
endif()


ADD_EXECUTABLE(pIRIS2D ${SRC})

TARGET_LINK_LIBRARIES(pIRIS2D
   ${MOOS_LIBRARIES}
   geometry
   apputil
   mbutil
   m
   pthread
   Eigen3::Eigen)

