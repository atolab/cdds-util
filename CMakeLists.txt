cmake_minimum_required(VERSION 3.0.0)
project(cdds-util VERSION 0.1.0)

include(CTest)
enable_testing()

include_directories(./include /usr/local/include)
link_directories(/usr/local/lib)
if (BUILD_SHARED_LIBS OR NOT DEFINED BUILD_SHARED_LIBS)
    add_library(cdds-util SHARED src/cdds_blob.c)
else()
    add_library(cdds-util src/cdds_blob.c)
endif()

target_link_libraries(cdds-util ddsc)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)

install(TARGETS cdds-util DESTINATION lib)
install(DIRECTORY include/cdds DESTINATION include)
