cmake_minimum_required(VERSION 3.5)
project(adaptive_clustering)

//CMake 최소 버전은 3.5 이상 필요
//이 프로젝트(패키지)의 이름은 adaptive_clustering

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

//GCC 또는 Clang 컴파일러를 쓴다면, 경고를 최대한 많이 보여주도록 옵션 설정
	-Wall: 기본 경고
	-Wextra: 추가 경고
	-Wpedantic: 표준에서 벗어난 문법 경고

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(adaptive_clustering_msgs REQUIRED)

//ROS2에서 사용하는 모든 패키지를 불러옴
	adaptive_clustering_msgs는 너가 따로 만든 custom message 패키지야 (e.g. ClusterArray 메시지)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

//PCL (Point Cloud Library)를 쓰기 위해 헤더 경로와 라이브러리 경로, 정의들 추가

# Executable
add_executable(${PROJECT_NAME} src/adaptive_clustering.cpp)

//src/adaptive_clustering.cpp를 컴파일해서 실행파일 adaptive_clustering 생성

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  sensor_msgs
  visualization_msgs
  geometry_msgs
  pcl_conversions
  pcl_msgs
  PCL
  adaptive_clustering_msgs
)

//위 실행파일이 의존하는 ROS2 및 PCL 패키지들을 연결

target_link_libraries(adaptive_clustering ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_SEARCH_LIBRARIES} ${PCL_SEGMENTATION_LIBRARIES})

//PCL의 각 기능별 라이브러리 연결

클러스터링: segmentation

KdTree 검색: search

파일 I/O 등 //

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)
//실행파일을 빌드 후 설치할 때 어디로 보낼지 지정

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)
//launch/ 디렉토리도 함께 설치 (launch 파일도 패키지 안에 포함되도록)

ament_package()
//이 패키지가 ament 빌드 시스템을 사용한다고 선언