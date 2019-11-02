cmake_minimum_required(VERSION 3.8)
project(Aquisition)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_FLAGS "-Wall -Wextra -O3")

find_package(PkgConfig REQUIRED)

pkg_check_modules(USB REQUIRED
	libusb-1.0 hidapi-libusb)

set(MCCUSB_DIR "${CMAKE_SOURCE_DIR}/Linux_Drivers/USB/mcc-libusb")

include_directories("${MCCUSB_DIR}")
include_directories(${USB_INCLUDE_DIRS})

add_executable(aquire main.c "${MCCUSB_DIR}/pmd.c" "${MCCUSB_DIR}/nist.c" "${MCCUSB_DIR}/usb-20X.c")
target_link_libraries(aquire ${USB_LIBRARIES} m)
