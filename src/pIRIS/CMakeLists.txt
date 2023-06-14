#--------------------------------------------------------
# The CMakeLists.txt for:                       pIRIS
# Author(s):                              Raul Largaespada
#--------------------------------------------------------

SET(SRC
  IRIS.cpp
  IRIS_Info.cpp
  main.cpp
)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

ADD_EXECUTABLE(pIRIS ${SRC})

TARGET_LINK_LIBRARIES(pIRIS
   ${MOOS_LIBRARIES}
   geometry
   apputil
   mbutil
   m
   pthread
   Eigen3::Eigen)

