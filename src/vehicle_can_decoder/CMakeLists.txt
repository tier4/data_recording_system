cmake_minimum_required(VERSION 3.14)
project(vehicle_can_decoder VERSION 0.1.0 LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ── ROS2 build system ────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(can_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# ── Message generation ───────────────────────────────────────────────────────
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Signal.msg"
  "msg/SignalEntry.msg"
  "msg/SignalGroup.msg"
  "msg/SignalDiagnostic.msg"
  "msg/VehicleSchema.msg"
  DEPENDENCIES std_msgs
)

# ── Third-party: dbcppp ──────────────────────────────────────────────────────
include(FetchContent)

FetchContent_Declare(
  dbcppp
  GIT_REPOSITORY https://github.com/xR3b0rn/dbcppp.git
  # Pin to a release tag for reproducible builds.
  # Update intentionally when a new release is vetted.
  GIT_TAG        v3.2.6
  GIT_SHALLOW    TRUE
)
# dbcppp uses its own CMakeLists; set options before MakeAvailable
set(dbcppp_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(dbcppp_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(dbcppp)

# dbcppp v3.x only exports INSTALL_INTERFACE include paths on the library
# target. Patch the BUILD_INTERFACE so dependents can find <dbcppp/Network.h>.
target_include_directories(libdbcppp PUBLIC
  $<BUILD_INTERFACE:${dbcppp_SOURCE_DIR}/include>
)

# ── Third-party: exprtk (header-only) ────────────────────────────────────────
FetchContent_Declare(
  exprtk
  GIT_REPOSITORY https://github.com/ArashPartow/exprtk.git
  # Pin to a specific commit for reproducible builds.
  GIT_TAG        0.0.3
  GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(exprtk)

add_library(exprtk INTERFACE)
target_include_directories(exprtk INTERFACE ${exprtk_SOURCE_DIR})

# ── Library target ────────────────────────────────────────────────────────────
add_library(${PROJECT_NAME}_lib
  src/dbc_decoder.cpp
  src/signal_transformer.cpp
  src/signal_router.cpp
  src/timeout_monitor.cpp
  src/can_reader.cpp
  src/vehicle_can_node.cpp
)

target_include_directories(${PROJECT_NAME}_lib
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# exprtk is large; compile it in its own TU to keep build times manageable
target_compile_options(${PROJECT_NAME}_lib PRIVATE
  $<$<CXX_COMPILER_ID:GNU>:-O2>
)

target_link_libraries(${PROJECT_NAME}_lib
  libdbcppp
  exprtk
)

ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  std_msgs
  can_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(${PROJECT_NAME}_lib "${cpp_typesupport_target}")

# ── Node executable ───────────────────────────────────────────────────────────
add_executable(vehicle_can_node src/main.cpp)
target_link_libraries(vehicle_can_node ${PROJECT_NAME}_lib)

# ── Tests ─────────────────────────────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_dbc_decoder
    test/test_dbc_decoder.cpp
  )
  target_link_libraries(test_dbc_decoder ${PROJECT_NAME}_lib)

  ament_add_gtest(test_signal_transformer
    test/test_signal_transformer.cpp
  )
  target_link_libraries(test_signal_transformer ${PROJECT_NAME}_lib)

  ament_add_gtest(test_signal_router
    test/test_signal_router.cpp
  )
  target_link_libraries(test_signal_router ${PROJECT_NAME}_lib)

  ament_add_gtest(test_timeout_monitor
    test/test_timeout_monitor.cpp
  )
  target_link_libraries(test_timeout_monitor ${PROJECT_NAME}_lib)

  ament_add_gtest(test_integration
    test/test_integration.cpp
  )
  target_link_libraries(test_integration ${PROJECT_NAME}_lib)

  ament_add_gtest(test_schema_pre_population
    test/test_schema_pre_population.cpp
  )
  target_link_libraries(test_schema_pre_population ${PROJECT_NAME}_lib)

  # Make test data (DBC file, YAML config) accessible to tests
  set(TEST_DATA_DIR "${CMAKE_CURRENT_SOURCE_DIR}/test/test_data")
  target_compile_definitions(test_dbc_decoder           PRIVATE TEST_DATA_DIR="${TEST_DATA_DIR}")
  target_compile_definitions(test_signal_router         PRIVATE TEST_DATA_DIR="${TEST_DATA_DIR}")
  target_compile_definitions(test_integration           PRIVATE TEST_DATA_DIR="${TEST_DATA_DIR}")
  target_compile_definitions(test_schema_pre_population PRIVATE TEST_DATA_DIR="${TEST_DATA_DIR}")
endif()

# ── Install ───────────────────────────────────────────────────────────────────
install(TARGETS ${PROJECT_NAME}_lib
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

install(TARGETS vehicle_can_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
