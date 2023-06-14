cmake_minimum_required(VERSION 3.13.1)
# This application is based on the smp_svr zephyr sample for use
# with mcumgr. Additional functionality has been added in
# support of contact tracing applications on BT510 hardware.

# Standard Zephyr application boilerplate.
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(bt510)

FILE(GLOB app_sources src/*.c)
target_sources(app PRIVATE
    ${app_sources}
)

include_directories(${CMAKE_SOURCE_DIR}/include)

zephyr_link_libraries_ifdef(CONFIG_FILE_SYSTEM_NFFS NFFS)
target_include_directories(app PRIVATE ${ZEPHYR_BASE}/subsys/fs/nvs)

