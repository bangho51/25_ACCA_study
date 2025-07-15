cmake_minimum_required(VERSION 3.8)   # cmake 최소 버전 요구 (여기선 3.8 이상)
project(adaptive_clustering_msgs)   # 프로젝트 이름을 'adaptive_clustering_msgs'로 설정

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")  # 만약 GNU 또는 Clang 컴파일러라면,
  add_compile_options(-Wall -Wextra -Wpedantic)  #   모든 경고(-Wall), 추가 경고(-Wextra), 엄격한 경고(-Wpedantic) 옵션 추가
endif()

# find dependencies
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)   # (주석) 추가 의존성 패키지를 필요에 따라 수동으로 추가할 때 사용


find_package(ament_cmake REQUIRED)  # 'ament_cmake' 패키지 필수, 빌드 시스템 기반 도구
/* ament_cmake란?
ament_cmake는 **ROS 2(로봇 운영 체제 2)**에서 표준적으로 사용하는 빌드 시스템 도구입니다. ROS 1에서 사용하던 catkin의 후속으로, ROS 2의 빌드·패키지 관리 표준을 제공합니다.

주요 역할과 특징
빌드 자동화
CMake를 기반으로, ROS 2 패키지의 소스 코드, 메시지, 서비스, 액션 등을 쉽게 빌드하고 설치할 수 있도록 관리합니다.

의존성 관리
각 패키지간의 의존성을 쉽게 선언하고, 자동으로 관련 패키지를 찾고 빌드 순서를 조정합니다.

확장성과 플러그인 구조
다양한 기능(예: 메시지 생성, 서비스 생성 등)과 테스트 툴, 린트 자동화 등을 플러그인 형태로 손쉽게 추가할 수 있습니다.

빌드 규약 강제
ROS 2 소프트웨어가 플랫폼에 상관없이 동일한 규약으로 빌드되어 호환성을 높입니다.

자동 설치 및 export
패키지를 설치하면서 자동으로 라이브러리, 실행 파일, 메시지 등의 export를 지원하여 다른 패키지에서 쓸 수 있게 해줍니다.
실제 이코드의 역할:
1. 빌드 시 컴파일 옵션/설정 적용
2. 메시지/서비스/액션 파일 자동 생성
3. 의존성 패키지 자동 설치 및 연결 */  
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)


rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ClusterArray.msg"
  DEPENDENCIES std_msgs sensor_msgs
 )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()




ament_package()
