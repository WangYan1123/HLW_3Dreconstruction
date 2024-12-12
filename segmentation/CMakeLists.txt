
file(GLOB reg_SOURCES 
"codelibrary/*.h" 
"codelibrary/*.cpp" 
"regiongrow.h" "regiongrow.cpp" "vccs_knn_supervoxel.h""vccs_supervoxel.h" "supervoxel.cpp""meshsample.cpp""supervoxelseg.cpp""PPF.cpp""HPR.cpp""HPR.h""PPF.h"
)

add_library(segmentation SHARED ${reg_SOURCES})

target_link_libraries(segmentation 
  PRIVATE 
  ${OpenCV_LIBRARIES}
  Open3D::Open3D
  ${PCL_LIBRARIES}

)

target_compile_definitions(segmentation PRIVATE REG_LIBRARY)

add_executable(segmentationTest ${reg_SOURCES})
target_link_libraries(segmentationTest   
  PRIVATE 
  ${OpenCV_LIBRARIES}
  Open3D::Open3D
  ${PCL_LIBRARIES}

)

