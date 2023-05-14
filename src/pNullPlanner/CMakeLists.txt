#--------------------------------------------------------
# The CMakeLists.txt for:                       pNullPlanner
# Author(s):                              Raul Largaespada
#--------------------------------------------------------

SET(SRC
  NullPlanner.cpp
  NullPlanner_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(pNullPlanner ${SRC})

TARGET_LINK_LIBRARIES(pNullPlanner
   ${MOOS_LIBRARIES}
   geometry
   apputil
   mbutil
   m
   pthread)

