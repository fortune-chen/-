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
PROJECT(robot_api)

# 根据APP类型确定编译代码目录、宏定义以及编译选项
AUX_SOURCE_DIRECTORY(. DIR_SRCS)
INCLUDE_DIRECTORIES(./../src/include)

# 编译选项
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -pthread -lm -lrt -Os -g -rdynamic -funwind-tables -ffunction-sections")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -lm -lrt -Os -g -rdynamic -funwind-tables -ffunction-sections")
SET(LIBRARY_OUTPUT_PATH "./../../src/libs")

# 生成静态库
ADD_LIBRARY(${PROJECT_NAME} STATIC ${DIR_SRCS})

