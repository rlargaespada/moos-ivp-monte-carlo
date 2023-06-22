#--------------------------------------------------------
# The CMakeLists.txt for:                       pIRIS
# Author(s):                              Raul Largaespada
#--------------------------------------------------------

SET(SRC
  IRIS2D.cpp
  IRIS2D_Info.cpp
  main.cpp
)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

ADD_EXECUTABLE(pIRIS2D ${SRC})

TARGET_LINK_LIBRARIES(pIRIS2D
   ${MOOS_LIBRARIES}
   geometry
   apputil
   mbutil
   m
   pthread
   Eigen3::Eigen)

