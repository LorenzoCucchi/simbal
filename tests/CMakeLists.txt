set(SOURCES
  "test_frames.cpp"
  "test_atmosisa.cpp"
)


set(LIB_NAME ${PROJECT_NAME}_lib)
set(TEST_NAME tests)

add_executable(${TEST_NAME} test_main.cpp ${SOURCES})

target_link_libraries(${TEST_NAME} PRIVATE ${LIB_NAME} Catch2::Catch2WithMain)

if (MSVC)
  target_compile_options(${TEST_NAME} PRIVATE "/IW4")
  target_compile_options(${TEST_NAME} PRIVATE "/WX")
else()
  target_compile_options(${TEST_NAME} PRIVATE "-Wall")
  target_compile_options(${TEST_NAME} PRIVATE "-Wextra")
  target_compile_options(${TEST_NAME} PRIVATE "-Werror")
  target_compile_options(${TEST_NAME} PRIVATE "-pedantic")
endif()

