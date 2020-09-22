cmake_minimum_required(VERSION 3.0.0)
project(cdds-util VERSION 0.1.0)

include(CTest)
enable_testing()

include_directories(./include /usr/local/include)
link_directories(/usr/local/lib)
add_library(cdds-util SHARED src/cdds_blob.c)

target_link_libraries(cdds-util ddsc)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
