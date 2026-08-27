# --- Robot Target Configuration -----------------------------------------------
set(ROBOT_TARGET "ALL" CACHE STRING
    "Which robot(s) to build for: \"LANCE1\", \"LANCE2\", \"ALL\"")
set_property(CACHE ROBOT_TARGET PROPERTY STRINGS LANCE1 LANCE2 ALL)

set(BUILD_LANCE1 FALSE)
set(BUILD_LANCE2 FALSE)
if(ROBOT_TARGET STREQUAL "LANCE1" OR ROBOT_TARGET STREQUAL "ALL")
    set(BUILD_LANCE1 TRUE)
endif()
if(ROBOT_TARGET STREQUAL "LANCE2" OR ROBOT_TARGET STREQUAL "ALL")
    set(BUILD_LANCE2 TRUE)
endif()

message(STATUS
    "Robot target is \"${ROBOT_TARGET}\";\n"
    "\tLANCE-1 enabled: ${BUILD_LANCE1}\n"
    "\tLANCE-2 enabled: ${BUILD_LANCE2}")
