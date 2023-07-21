#--------------------------------------------------------
# The CMakeLists.txt for:                       pIRIS2D
# Author(s):                              Raul Largaespada
#--------------------------------------------------------

# if any components are missing, fatal error
if(GCS_MISSING_COMPONENTS)
  message(FATAL_ERROR "Cannot build pIRIS2D due to missing dependencies: ${GCS_MISSING_COMPONENTS}")
endif()

SET(SRC
  IRISPolygon.cpp
  IRISEllipse.cpp
  iris_mosek.cpp
  iris_ops.cpp
  IRISProblem.cpp
  IRIS2D.cpp
  IRIS2D_Info.cpp
  main.cpp
)

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
  ${MOSEK_C_LIB})

TARGET_INCLUDE_DIRECTORIES(pIRIS2D PUBLIC
  ${CDD_INCLUDE_DIR}
  ${MOSEK_INCLUDE_DIR})
