cmake_minimum_required(VERSION 3.16)
project(MarsLander VERSION 1.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- Настройка SFML ---

set(SFML_DIR "C:/SFML-3.0.2/lib/cmake/SFML") 

find_package(SFML 3 COMPONENTS Graphics Window System REQUIRED)

# Файлы проекта

include_directories(include libs)

# Исходный код 
set(SOURCES
    src/main.cpp
    src/TerrainGenerator.cpp
    src/PhysicsEngine.cpp
    src/LandingController.cpp
    src/Visualizer.cpp
    src/LandingSiteDetector.cpp
    src/RadarTypes.cpp
)

set(HEADERS
    include/Config.h
    include/TerrainGenerator.h
    include/PhysicsEngine.h
    include/LandingController.h
    include/Visualizer.h
    include/LandingSiteDetector.h
    include/RadarTypes.h
)


add_executable(MarsLander ${SOURCES} ${HEADERS})


target_link_libraries(MarsLander PRIVATE SFML::Graphics SFML::Window SFML::System)


if(WIN32)
    add_custom_command(TARGET MarsLander POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        $<TARGET_FILE:SFML::Graphics>
        $<TARGET_FILE:SFML::Window>
        $<TARGET_FILE:SFML::System>
        $<TARGET_FILE_DIR:MarsLander>
        COMMENT "Copying SFML DLLs to executable directory"
    )
endif()