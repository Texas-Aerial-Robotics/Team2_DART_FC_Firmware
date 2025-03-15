cmake_minimum_required(VERSION 3.22)
enable_language(C ASM)
set(MX_Defines_Syms
    USE_PWR_LDO_SUPPLY
    USE_HAL_DRIVER
    STM32H743xx
    $<$<CONFIG:Debug>:DEBUG>
)
set(MX_Include_Dirs
    ${CMAKE_SOURCE_DIR}/Core/Inc
    ${CMAKE_SOURCE_DIR}/Drivers/STM32H7xx_HAL_Driver/Inc
    ${CMAKE_SOURCE_DIR}/Drivers/STM32H7xx_HAL_Driver/Inc/Legacy
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Device/ST/STM32H7xx/Include
    ${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Include
)
file(GLOB_RECURSE CORE_SOURCES
    "${CMAKE_SOURCE_DIR}/Core/Src/*.[cCsS]"
    "${CMAKE_SOURCE_DIR}/Core/Src/*.cpp"
)
file(GLOB_RECURSE DRIVER_SOURCES
    "${CMAKE_SOURCE_DIR}/Drivers/**/*.[cCsS]"
    "${CMAKE_SOURCE_DIR}/Drivers/**/*.cpp"
)
add_library(stm32cubemx INTERFACE)
target_include_directories(stm32cubemx INTERFACE ${MX_Include_Dirs})
target_compile_definitions(stm32cubemx INTERFACE ${MX_Defines_Syms})
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    ${CORE_SOURCES}
    ${DRIVER_SOURCES}
)
target_link_libraries(${CMAKE_PROJECT_NAME} stm32cubemx)
set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES ADDITIONAL_CLEAN_FILES "${CMAKE_PROJECT_NAME}.map")
if((CMAKE_C_STANDARD EQUAL 90) OR (CMAKE_C_STANDARD EQUAL 99))
    message(FATAL_ERROR "Generated code requires C11 or higher")
endif()
