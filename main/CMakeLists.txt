set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(app_sources "main.cpp" "mycache.cpp" )

idf_component_register(SRCS ${app_sources} REQUIRES epdiy2)
