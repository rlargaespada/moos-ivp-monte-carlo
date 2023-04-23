#--------------------------------------------------------
# The CMakeLists.txt for:                       uFldObsMonteCarloSim
# Author(s):                              Raul Largaespada
#--------------------------------------------------------

SET(SRC
  ObsMonteCarloSim.cpp
  ObsMonteCarloSim_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(uFldObsMonteCarloSim ${SRC})

TARGET_LINK_LIBRARIES(uFldObsMonteCarloSim
   ${MOOS_LIBRARIES}
   apputil
   obstacles
   contacts
   geometry
   mbutil
   m
   pthread)

