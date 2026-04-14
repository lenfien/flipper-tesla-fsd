cmake_minimum_required(VERSION 3.10)

file(GLOB SRC_FILES
        include/*/*h
)

add_executable(evan-open-can-tools ${SRC_FILES})