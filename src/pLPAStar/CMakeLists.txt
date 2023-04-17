#--------------------------------------------------------
# The CMakeLists.txt for:                       pLPAStar
# Author(s):                              
#--------------------------------------------------------

SET(SRC
  LPAStar.cpp
  LPAStar_Info.cpp
  main.cpp
)

ADD_EXECUTABLE(pLPAStar ${SRC})

TARGET_LINK_LIBRARIES(pLPAStar
   ${MOOS_LIBRARIES}
   geometry
   apputil
   mbutil
   m
   pthread)

