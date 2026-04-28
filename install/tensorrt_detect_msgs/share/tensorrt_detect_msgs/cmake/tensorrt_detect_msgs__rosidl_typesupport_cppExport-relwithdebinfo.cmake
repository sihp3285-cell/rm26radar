#----------------------------------------------------------------
# Generated CMake target import file for configuration "RelWithDebInfo".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "tensorrt_detect_msgs::tensorrt_detect_msgs__rosidl_typesupport_cpp" for configuration "RelWithDebInfo"
set_property(TARGET tensorrt_detect_msgs::tensorrt_detect_msgs__rosidl_typesupport_cpp APPEND PROPERTY IMPORTED_CONFIGURATIONS RELWITHDEBINFO)
set_target_properties(tensorrt_detect_msgs::tensorrt_detect_msgs__rosidl_typesupport_cpp PROPERTIES
  IMPORTED_LINK_DEPENDENT_LIBRARIES_RELWITHDEBINFO "rosidl_runtime_c::rosidl_runtime_c;rosidl_typesupport_cpp::rosidl_typesupport_cpp;rosidl_typesupport_c::rosidl_typesupport_c"
  IMPORTED_LOCATION_RELWITHDEBINFO "${_IMPORT_PREFIX}/lib/libtensorrt_detect_msgs__rosidl_typesupport_cpp.so"
  IMPORTED_SONAME_RELWITHDEBINFO "libtensorrt_detect_msgs__rosidl_typesupport_cpp.so"
  )

list(APPEND _cmake_import_check_targets tensorrt_detect_msgs::tensorrt_detect_msgs__rosidl_typesupport_cpp )
list(APPEND _cmake_import_check_files_for_tensorrt_detect_msgs::tensorrt_detect_msgs__rosidl_typesupport_cpp "${_IMPORT_PREFIX}/lib/libtensorrt_detect_msgs__rosidl_typesupport_cpp.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
