# Register the app as an IDF component
idf_component_register(
    SRCS "main.c"
    PRIV_INCLUDE_DIRS "."
)

# Get target and arch
idf_build_get_property(target IDF_TARGET)
idf_build_get_property(arch IDF_TARGET_ARCH)

# Ensure we are on RISC-V (ESP32-C6)
if(NOT "${arch}" STREQUAL "riscv")
    message(WARNING "This Swift configuration is optimized for RISC-V (ESP32-C6). Current arch: ${arch}")
endif()

# Extract flags
string(REGEX MATCH "-march=[^ ]+" march_flag "${CMAKE_C_FLAGS}")
string(REGEX REPLACE "_x[^ ]*" "" march_flag "${march_flag}")

string(REGEX MATCH "-mabi=[^ ]+" mabi_flag "${CMAKE_C_FLAGS}")
if("${mabi_flag}" STREQUAL "")
    set(mabi_flag "-mabi=ilp32")
endif()

# Clear default compile options for Swift
get_target_property(var ${COMPONENT_LIB} COMPILE_OPTIONS)
set_target_properties(${COMPONENT_LIB} PROPERTIES COMPILE_OPTIONS "")

# Compute -Xcc flags for includes
set(SWIFT_INCLUDES)
foreach(dir ${CMAKE_C_IMPLICIT_INCLUDE_DIRECTORIES})
    string(CONCAT SWIFT_INCLUDES ${SWIFT_INCLUDES} "-Xcc ")
    string(CONCAT SWIFT_INCLUDES ${SWIFT_INCLUDES} "-I${dir} ")
endforeach()
foreach(dir ${CMAKE_CXX_IMPLICIT_INCLUDE_DIRECTORIES})
    string(CONCAT SWIFT_INCLUDES ${SWIFT_INCLUDES} "-Xcc ")
    string(CONCAT SWIFT_INCLUDES ${SWIFT_INCLUDES} "-I${dir} ")
endforeach()

# Swift compiler flags
target_compile_options(${COMPONENT_LIB} PUBLIC "$<$<COMPILE_LANGUAGE:Swift>:SHELL:
        -target riscv32-none-none-eabi
        -Xfrontend -function-sections -enable-experimental-feature Embedded -wmo -parse-as-library -Osize
        -Xcc ${march_flag} -Xcc ${mabi_flag} -Xcc -fno-pic -Xcc -fno-pie
        
        -pch-output-dir /tmp
        -Xfrontend -enable-single-module-llvm-emission
        
        ${SWIFT_INCLUDES}

        -import-bridging-header ${CMAKE_CURRENT_LIST_DIR}/BridgingHeader.h
    >")

# Enable Swift
set(CMAKE_Swift_COMPILER_WORKS YES)
set(CMAKE_Swift_COMPILATION_MODE_DEFAULT wholemodule)
set(CMAKE_Swift_COMPILATION_MODE wholemodule)
enable_language(Swift)

# Sources
target_sources(${COMPONENT_LIB}
    PRIVATE
    GatewayApp.swift
)

# Remove .swift_modhash section
add_custom_command(
    TARGET ${COMPONENT_LIB}
    POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} --remove-section .swift_modhash
            $<TARGET_FILE:${COMPONENT_LIB}> $<TARGET_FILE:${COMPONENT_LIB}>
)
