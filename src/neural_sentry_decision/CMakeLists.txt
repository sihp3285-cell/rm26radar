cmake_minimum_required(VERSION 3.16)
project(neural_sentry_decision)
find_package(ament_cmake REQUIRED)
# Python/ORT is loaded exclusively by this separate executable, not the C++ radar library.
install(DIRECTORY neural_sentry DESTINATION lib/${PROJECT_NAME}
        PATTERN "__pycache__" EXCLUDE)
install(PROGRAMS scripts/neural_sentry_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
ament_package()
