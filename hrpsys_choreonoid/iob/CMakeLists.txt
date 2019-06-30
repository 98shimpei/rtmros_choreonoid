add_definitions(-DHRPSYS_PACKAGE_VERSION=\"\\"${CPACK_PACKAGE_VERSION_MAJOR}.${CPACK_PACKAGE_VERSION_MINOR}.${CPACK_PACKAGE_VERSION_PATCH}\\"\" -DROBOT_IOB_VERSION=4)

# https://github.com/start-jsk/rtmros_choreonoid/issues/296
# catkin bt --cmake-args -DCMAKE_BUILD_TYPE={Release, Debug}
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# hrpIo
add_library(hrpIo_choreonoid SHARED iob.cpp)
#add_dependencies(hrpIo_choreonoid hrpsys_choreonoid_msgs_gencpp)
include_directories(${ROBOTHARDWARE_SOURCE})

if (NOT APPLE AND NOT QNXNTO)
   target_link_libraries(hrpIo_choreonoid rt)
endif()

target_link_libraries(hrpIo_choreonoid roscpp rostime)

# RobotHardware
#set(comp_source  ${ROBOTHARDWARE_SOURCE}/robot.cpp ${ROBOTHARDWARE_SOURCE}/RobotHardware.cpp ${ROBOTHARDWARE_SOURCE}/RobotHardwareService_impl.cpp)
message ("RobotHardware source directory: ${ROBOTHARDWARE_SOURCE}")
set(comp_source  ${ROBOTHARDWARE_SOURCE}/robot.cpp RobotHardware_choreonoid.cpp ${ROBOTHARDWARE_SOURCE}/RobotHardware.cpp ${ROBOTHARDWARE_SOURCE}/RobotHardwareService_impl.cpp)

#set(libs)
foreach(hrplib hrpModel-3.1 hrpCollision-3.1 hrpUtil-3.1)
  find_library(libpath NAMES ${hrplib} PATHS ${openhrp3_LIBRARY_DIRS} NO_DEFAULT_PATH)
  list(APPEND libs ${libpath})
endforeach(hrplib)
find_library(hrpStub NAMES hrpsysBaseStub PATHS ${hrpsys_LIBRARY_DIRS} NO_DEFAULT_PATH)

add_library(RobotHardware_choreonoid SHARED ${comp_source})

target_link_libraries(RobotHardware_choreonoid ${libs} ${hrpStub} hrpIo_choreonoid)
set_target_properties(RobotHardware_choreonoid PROPERTIES PREFIX "")
set_target_properties(RobotHardware_choreonoid PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set_target_properties(RobotHardware_choreonoid PROPERTIES OUTPUT_NAME RobotHardware_choreonoid)

# add_executable(RobotHardwareComp_choreonoid ${ROBOTHARDWARE_SOURCE}/RobotHardwareComp.cpp ${comp_source})
# target_link_libraries(RobotHardwareComp_choreonoid ${libs} ${omniorb_LIBRARIES} ${omnidynamic_LIBRARIES} RTC coil dl pthread)
# set_target_properties(RobotHardwareComp_choreonoid PROPERTIES OUTPUT_NAME RobotHardwareComp)

# add_library(hrpEC_choreonoid SHARED ${HRPEC_SOURCE}/hrpEC.cpp ${HRPEC_SOURCE}/hrpEC-common.cpp)
# target_link_libraries(hrpEC_choreonoid hrpIo_choreonoid hrpsysBaseStub)
# set_target_properties(hrpEC_choreonoid PROPERTIES PREFIX "")
# set_target_properties(hrpEC_choreonoid PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
# set_target_properties(hrpEC_choreonoid PROPERTIES OUTPUT_NAME hrpEC)
# set_target_properties(hrpEC_choreonoid PROPERTIES COMPILE_DEFINITIONS "OPENRTM_VERSION110")

# hrpIo
add_library(hrpIo_choreonoid_hand SHARED iob_hand.cpp)
#add_dependencies(hrpIo_choreonoid hrpsys_choreonoid_msgs_gencpp)
include_directories(${ROBOTHARDWARE_SOURCE})

if (NOT APPLE AND NOT QNXNTO)
   target_link_libraries(hrpIo_choreonoid_hand rt)
endif()

target_link_libraries(hrpIo_choreonoid_hand roscpp rostime)

find_package(jsk_hrp2_ros_bridge)
list(GET jsk_hrp2_ros_bridge_INCLUDE_DIRS 0 jsk_hrp2_ros_bridge_first_incdir)
list(APPEND jsk_hrp2_ros_bridge_INCLUDE_DIRS ${jsk_hrp2_ros_bridge_first_incdir}/jsk_hrp2_ros_bridge)
list(APPEND jsk_hrp2_ros_bridge_INCLUDE_DIRS ${jsk_hrp2_ros_bridge_first_incdir}/jsk_hrp2_ros_bridge/idl)
set(HRP3HANDCONTROLLER_SOURCE ${jsk_hrp2_ros_bridge_SOURCE_PREFIX}/rtc/HRP3HandController)
set(comp_sources_hand HRP3HandController_choreonoid.cpp handcontrol.cpp ${HRP3HANDCONTROLLER_SOURCE}/HRP3HandController.cpp ${HRP3HANDCONTROLLER_SOURCE}/HRP3HandControllerService_impl.cpp ${SEQUENCEPLAYER_SOURCE}/interpolator.cpp)
set(HRP2NO "-DHRPNO=17")
set(handcontrol_compile_flags "${HRP2NO}")
include_directories(${jsk_hrp2_ros_bridge_INCLUDE_DIRS} ${HRP3HANDCONTROLLER_SOURCE} ${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${SEQUENCEPLAYER_SOURCE} )
link_directories(${jsk_hrp2_ros_bridge_PREFIX}/lib)
add_definitions(-std=c++14)
add_library(HRP3HandController_choreonoid SHARED ${comp_sources_hand})
target_link_libraries(HRP3HandController_choreonoid ${libs} ${hrpStub} hrpIo_choreonoid_hand HRP3HandControllerServiceStub)
set_target_properties(HRP3HandController_choreonoid PROPERTIES PREFIX "")
set_target_properties(HRP3HandController_choreonoid PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set_target_properties(HRP3HandController_choreonoid PROPERTIES COMPILE_FLAGS ${handcontrol_compile_flags})
set_target_properties(HRP3HandController_choreonoid PROPERTIES OUTPUT_NAME HRP3HandController_choreonoid)

install(TARGETS hrpIo_choreonoid hrpIo_choreonoid_hand
  LIBRARY DESTINATION lib CONFIGURATIONS Release Debug RelWithDebInfo
)
##install(TARGETS RobotHardwareComp_choreonoid RobotHardware_choreonoid hrpEC_choreonoid
install(TARGETS RobotHardware_choreonoid HRP3HandController_choreonoid
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} CONFIGURATIONS Release Debug RelWithDebInfo
  LIBRARY DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/lib CONFIGURATIONS Release Debug RelWithDebInfo
)
