file(GLOB reg_SOURCES "ColeProcess.h" "ColeProcess.cpp")


add_library(cole SHARED ${reg_SOURCES})

target_link_libraries(cole 
  PRIVATE 
  ${OpenCV_LIBRARIES}
  Open3D::Open3D
  ${PCL_LIBRARIES}

)

target_compile_definitions(cole PRIVATE REG_LIBRARY)

add_executable(coleTest ${reg_SOURCES})
target_link_libraries(coleTest   
  PRIVATE 
  ${OpenCV_LIBRARIES}
  Open3D::Open3D
  ${PCL_LIBRARIES}
)

