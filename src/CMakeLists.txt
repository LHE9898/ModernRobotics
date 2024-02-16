# Source files
file(GLOB SRC_FILES "*.cpp")

# Create library
add_library(mylib ${SRC_FILES})
# Specify include directories
target_include_directories(mylib PUBLIC ${CMAKE_SOURCE_DIR}/inc)

# 외부 라이브러리 Linking
find_package(Eigen3 REQUIRED)
target_link_libraries(mylib Eigen3::Eigen)

# ... 외부 라이브러리 사용시 추가