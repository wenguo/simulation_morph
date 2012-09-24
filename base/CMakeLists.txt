find_package( FLTK REQUIRED )
IF( FLTK_FOUND )
  MESSAGE( STATUS ${INDENT} "FLTK_LIBRARIES=${FLTK_LIBRARIES}" )
  MESSAGE( STATUS ${INDENT} "FLTK_INCLUDE_DIR=${FLTK_INCLUDE_DIR}" )
ELSE( FLTK_FOUND )
  MESSAGE( FATAL_ERROR "FLTK not found, aborting" )
ENDIF( FLTK_FOUND )

find_package( OpenGL REQUIRED )
IF(OPENGL_GLU_FOUND)
    MESSAGE( STATUS ${INDENT} "OPENGL_LIBRARIES=${OPENGL_LIBRARIES}" )
    MESSAGE( STATUS ${INDENT} "OPENGL_INCLUDE_DIR=${OPENGL_INCLUDE_DIR}" )
ELSE(OPENGL_GLU_FOUND )
  MESSAGE( FATAL_ERROR "OpenGL GLU not found, aborting" )
ENDIF(OPENGL_GLU_FOUND )

INCLUDE_DIRECTORIES(${FLTK_INCLUDE_DIRS} ${OPENGL_INCLUDE_DIR})
LINK_DIRECTORIES(${FLTK_LIBRARY_DIRS} ${OPENGL_LIBRARY_DIRS})


ADD_LIBRARY(${ROBOT_BASE} STATIC
   robot.cc
   communication.cc
   simulation_manager.cc
   history.cc
   support.cc
   og/organism.cc
   og/organism_sample.cc
   og/organism_draw.cc
   og/organism_sequence.cc
   utils/worldfile.cc
   utils/ipc.cc
   utils/lolmsg.c
   utils/bytequeue.c
   )

TARGET_LINK_LIBRARIES(${ROBOT_BASE} ${FLTK_LIBRARIES} ${OPENGL_LIBRARIES} -lipc)
INSTALL(TARGETS ${ROBOT_BASE} DESTINATION /usr/local/lib)
