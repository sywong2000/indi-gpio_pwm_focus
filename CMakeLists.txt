cmake_minimum_required(VERSION 2.7)
PROJECT(indi_gpio_pwm_focus C CXX)

LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/")
LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../cmake_modules/")
#LIST(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../indi-3rdparty/cmake_modules/")
include(GNUInstallDirs)

set (GPIO_PWM_VERSION_MAJOR 2)
set (GPIO_PWM_VERSION_MINOR 1)

find_package(INDI REQUIRED)
find_package(Threads REQUIRED)

find_library(pigpiod_if2 REQUIRED)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/config.h.cmake ${CMAKE_CURRENT_BINARY_DIR}/config.h)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/indi_gpio_pwm_focus.xml.cmake ${CMAKE_CURRENT_BINARY_DIR}/indi_gpio_pwm_focus.xml)

include_directories( ${CMAKE_CURRENT_BINARY_DIR})
include_directories( ${CMAKE_CURRENT_SOURCE_DIR})
include_directories( ${INDI_INCLUDE_DIR})

include(CMakeCommon)

########### GPIO PWM Focuser ###########
set(indigpio_pwm_SRCS
   ${CMAKE_CURRENT_SOURCE_DIR}/GPIO_PWM_Focuser.cpp
   )

IF (UNITY_BUILD)
    ENABLE_UNITY_BUILD(indigpio_pwm_focus indigpio_pwm_SRCS 10 cpp)
ENDIF ()

add_executable(indi_gpio_pwm_focus ${indigpio_pwm_SRCS})

target_link_libraries(indi_gpio_pwm_focus ${INDI_DRIVER_LIBRARIES} pigpiod_if2 pthread)

install(TARGETS indi_gpio_pwm_focus RUNTIME DESTINATION bin )

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/indi_gpio_pwm_focus.xml DESTINATION ${INDI_DATA_DIR})
