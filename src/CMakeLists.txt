set(BOX2D_SOURCE_FILES
    Arbiter.h
    Arbiter.cpp
    Body.h
    Body.cpp
    Joint.h
    Joint.cpp
    MathUtils.h
    World.h
    World.cpp
    Collide.cpp)

add_library(box2d-lite STATIC ${BOX2D_SOURCE_FILES})
target_include_directories(box2d-lite PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
