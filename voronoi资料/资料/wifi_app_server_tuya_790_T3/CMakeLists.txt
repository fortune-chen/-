# cmake版本 
CMAKE_MINIMUM_REQUIRED(VERSION 3.10)

# 交叉编译方式
SET(CMAKE_SYSTEM_NAME Linux)
SET(CMAKE_SYSTEM_PROCESSOR arm)

# 指定C交叉编译器
SET(CMAKE_C_COMPILER "linux-gcc")

# 指定C++交叉编译器
SET(CMAKE_CXX_COMPILER "linux-g++")

# 指定AR、RANLIB命令
set(CMAKE_AR "linux-ar")
set(CMAKE_RANLIB "linux-ranlib")

# 指定strip命令
SET(CMAKE_STRIP "linux-strip")

# 项目的名称
PROJECT(wifi_app_server)

# 根据APP类型确定编译代码目录、宏定义以及编译选项
ADD_DEFINITIONS(-DAPP_TUYA)
INCLUDE_DIRECTORIES(./src/include)
INCLUDE_DIRECTORIES(./src/tuya ./src/tuya/include)
INCLUDE_DIRECTORIES(./src/tuya/include/hal ./src/tuya/include/hal/driver ./src/tuya/include/hal/system)
INCLUDE_DIRECTORIES(./src/smartconfig/ssv)
INCLUDE_DIRECTORIES(./src/triplet)

AUX_SOURCE_DIRECTORY(./src/tuya DIR_SRCS)
AUX_SOURCE_DIRECTORY(./src/debug DIR_SRCS)
AUX_SOURCE_DIRECTORY(./src/smartconfig/ssv DIR_SRCS)
AUX_SOURCE_DIRECTORY(./src/triplet SRC_MAIN)

LINK_DIRECTORIES(./src/tuya/libs ./src/libs ./src/debug ./src/triplet)
LINK_LIBRARIES(tuya_iot robot_api ulog config)

# 编译选项
IF (CMAKE_BUILD_TYPE STREQUAL Debug)
    SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -pthread -lm -lrt -Os -g -rdynamic -funwind-tables -ffunction-sections")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -lm -lrt -Os -g -rdynamic -funwind-tables -ffunction-sections")
ELSEIF (CMAKE_BUILD_TYPE STREQUAL Release)
    SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -pthread -lm -lrt -Os -s")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -lm -lrt -Os -s")
ELSE ()
    SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -pthread -lm -lrt -Os -s")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -lm -lrt -Os -s")
ENDIF()

# 链接目标
ADD_EXECUTABLE(${PROJECT_NAME} ${DIR_SRCS})

