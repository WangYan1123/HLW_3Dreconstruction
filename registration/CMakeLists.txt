add_definitions(-DREG_DATA_PATH="${PROJECT_SOURCE_DIR}/registration/registrationData/")
file(GLOB reg_SOURCES "registration.h" "registration.cpp")

add_library(registration  SHARED ${reg_SOURCES})

target_link_libraries(registration
  PRIVATE 
  Open3D::Open3D
  ${PCL_LIBRARIES}
)

target_compile_definitions(registration  PRIVATE REG_LIBRARY)

add_executable(registrationTest ${reg_SOURCES})
target_link_libraries(registrationTest   
  PRIVATE 
  ${OpenCV_LIBRARIES}
  Open3D::Open3D
  ${PCL_LIBRARIES}
)


