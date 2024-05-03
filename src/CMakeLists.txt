set(SOURCES
  "frames/frames.cpp"
  "environment/atmosisa.cpp"
  "environment/geodetic.cpp"
  "projectile/projectile.cpp"
  "equations/pointMass.cpp"
  "mission.cpp"
  "json.hpp"
  "utils/logger.hpp"
)


set(LIB_NAME ${PROJECT_NAME}_lib)

add_library(${LIB_NAME} STATIC ${SOURCES})

set_target_properties(${LIB_NAME} PROPERTIES OUTPUT_NAME ${PROJECT_NAME})

target_include_directories(${LIB_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} frames environment equations)

add_executable(${PROJECT_NAME} main.cpp)

target_link_libraries(${PROJECT_NAME} PRIVATE ${LIB_NAME})

