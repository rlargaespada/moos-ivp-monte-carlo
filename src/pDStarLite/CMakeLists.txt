#--------------------------------------------------------
# The CMakeLists.txt for:                       pDStarLite
# Author(s):                              Raul Largaespada
#--------------------------------------------------------

SET(SRC
  DStarLite.cpp
  DStarLite_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(pDStarLite ${SRC})

TARGET_LINK_LIBRARIES(pDStarLite
   ${MOOS_LIBRARIES}
   geometry
   apputil
   mbutil
   m
   pthread)

