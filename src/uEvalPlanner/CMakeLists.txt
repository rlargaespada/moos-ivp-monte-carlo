#--------------------------------------------------------
# The CMakeLists.txt for:                       uEvalPlanner
# Author(s):                              Raul Largaespada
#--------------------------------------------------------

SET(SRC
  EvalPlanner.cpp
  EvalPlanner_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(uEvalPlanner ${SRC})

TARGET_LINK_LIBRARIES(uEvalPlanner
   ${MOOS_LIBRARIES}
   geometry
   apputil
   mbutil
   m
   pthread)

