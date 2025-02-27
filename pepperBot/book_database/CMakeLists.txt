cmake_minimum_required(VERSION 3.8)
project(book_database)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Add the executable
add_executable(book_database src/book_database.cpp)

# Link the libraries
ament_target_dependencies(book_database
  rclcpp
  std_msgs
  geometry_msgs
)

# Install the executable
install(TARGETS
  book_database
  DESTINATION lib/${PROJECT_NAME}
)

# Mark the package as an ament package
ament_package()
