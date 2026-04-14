cmake_minimum_required(VERSION 3.10)

project(flipper-tesla-fsd)
file(GLOB SRC_FILES
        scenes/*.cpp scenes/*.h scenes/*.c
        my/*.cpp my/*.h my/*.c
        scenes_config/*.cpp scenes_config/*.h scenes_config/*.c
        libraries/*.cpp libraries/*.h libraries/*.c
        fsd_logic/*.cpp fsd_logic/*.h fsd_logic/*.c
        *.cpp *.h *.c
)

INCLUDE_DIRECTORIES(my)

add_library(flipper-tesla-fsd STATIC ${SRC_FILES})