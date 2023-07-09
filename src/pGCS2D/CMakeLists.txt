#--------------------------------------------------------
# The CMakeLists.txt for:                       pGCS2D
# Author(s):                              Raul Largaespada
#--------------------------------------------------------

# if any components are missing, fatal error
if(GCS_MISSING_COMPONENTS)
  message(FATAL_ERROR "Cannot build pGCS2D due to missing dependencies: ${GCS_MISSING_COMPONENTS}")
endif()

SET(SRC
  GCS2D.cpp
  GCS2D_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(pGCS2D ${SRC})

TARGET_LINK_LIBRARIES(pGCS2D
   ${MOOS_LIBRARIES}
   geometry
   apputil
   mbutil
   m
   pthread
   Eigen3::Eigen
   ${CDD_LIBRARY}
   ${MOSEK_C_LIB}
   ${MOSEK_FUSION_LIB})

TARGET_INCLUDE_DIRECTORIES(pGCS2D PUBLIC
  ${CDD_INCLUDE_DIR}
  ${MOSEK_INCLUDE_DIR})
