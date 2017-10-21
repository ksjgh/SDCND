PROJECT(Planner CXX C)

CMAKE_MINIMUM_REQUIRED(VERSION 2.4)
IF(COMMAND cmake_policy)
	CMAKE_POLICY(SET CMP0003 OLD)
	CMAKE_POLICY(SET CMP0005 OLD)
ENDIF(COMMAND cmake_policy)

SET(EXECUTABLE_OUTPUT_PATH "${PROJECT_BINARY_DIR}/bin")

#############################################################################
#Compiler flags for normal (debug) and optimized (release) builds
#
#
IF(NOT MSVC_IDE)
  SET(CMAKE_CXX_FLAGS_DEBUG "-g -O0" CACHE STRING  
      "Flags used by the C++ compiler during debug builds." FORCE)
  SET(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG" 
      CACHE STRING "Flags used by the C++ compiler during release builds." FORCE) 
  SET(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O3 -g" CACHE STRING
      "Flags used by the C++ compiler during release builds with debug information." FORCE) 

  SET(CMAKE_C_FLAGS_DEBUG "-g -O0" CACHE STRING
      "Flags used by the C compiler during debug builds." FORCE)
  SET(CMAKE_C_FLAGS_RELEASE "-O3 -DNDEBUG" CACHE STRING
      "Flags used by the C compiler during release builds." FORCE)
  SET(CMAKE_C_FLAGS_RELWITHDEBINFO "-O3 -g" CACHE STRING
      "Flags used by the C compiler during release builds with debug information." FORCE)

ENDIF(NOT MSVC_IDE)

IF(APPLE)
	LIST(APPEND CMAKE_SHARED_LINKER_FLAGS "-single_module")
ENDIF(APPLE)


#############################################################################
#Find OpenGL and GLUT
#
INCLUDE(FindOpenGL)
IF(NOT OPENGL_INCLUDE_DIR)
  SET(OPENGL_INCLUDE_DIR GL CACHE STRING "OpenGL directory for header file" FORCE)
ENDIF(NOT OPENGL_INCLUDE_DIR)

INCLUDE(FindGLUT)
IF(NOT GLUT_INCLUDE_DIR)
  SET(GLUT_INCLUDE_DIR GL CACHE STRING "Glut header file" FORCE)
ENDIF(NOT GLUT_INCLUDE_DIR)
IF (NOT GLUT_glut_LIBRARY)
  IF (WIN32 AND NOT CYGWIN)
    SET(GLUT_glut_LIBRARY glut32 CACHE STRING "Glut library for win32" FORCE)
  ENDIF(WIN32 AND NOT CYGWIN)
ENDIF (NOT GLUT_glut_LIBRARY)

INCLUDE_DIRECTORIES(${OPENGL_INCLUDE_DIR} ${GLUT_INCLUDE_DIR})


IF(WIN32)
  SET(MsgOpenGL  "cannot determine if gl library exists")
  SET(MsgOpenGLU "cannot determine if glu library exists")
  SET(MsgGLUT    "cannot determine if glut library exists")
ELSE(WIN32)
  IF(GLUT_glut_LIBRARY)
    SET(MsgGLUT "[ok]")
  ELSE(GLUT_glut_LIBRARY)
    SET(MsgGLUT "[missing]")
  ENDIF(GLUT_glut_LIBRARY)

  IF(OPENGL_gl_LIBRARY)
    SET(MsgOpenGL "[ok]")
  ELSE(OPENGL_gl_LIBRARY)
    SET(MsgOpenGL "[missing]")
  ENDIF(OPENGL_gl_LIBRARY)

  IF(OPENGL_glu_LIBRARY)
    SET(MsgOpenGLU "[ok]")
  ELSE(OPENGL_glu_LIBRARY)
    SET(MsgOpenGLU "[missing]")
  ENDIF(OPENGL_glu_LIBRARY)
ENDIF(WIN32)

IF(GLUT_glut_LIBRARY)
  SET(INTERACTIVE_LIBS ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY} ${GLUT_glut_LIBRARY})
ELSE (GLUT_glut_LIBRARY)
  SET(INTERACTIVE_LIBS ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY})
ENDIF(GLUT_glut_LIBRARY)

MESSAGE("
*****************************************************************************
* REQUIRED EXTERNAL LIBRARIES
*   GL   : ${MsgOpenGL} ${OPENGL_gl_LIBRARY}
*   GLU  : ${MsgOpenGLU} ${OPENGL_glu_LIBRARY}
*   GLUT : ${MsgGLUT} ${GLUT_glut_LIBRARY}
*****************************************************************************
")

#############################################################################
INCLUDE_DIRECTORIES(src)

AUX_SOURCE_DIRECTORY(src SRC_FILES)

ADD_EXECUTABLE(Planner ${SRC_FILES})
TARGET_LINK_LIBRARIES(Planner ${INTERACTIVE_LIBS})
