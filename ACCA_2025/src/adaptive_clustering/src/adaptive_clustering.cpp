// Copyright (C) 2018  Zhi Yan and Li Sun

// This program is free software: you can redistribute it and/or modify it
// under the terms of the GNU General Public License as published by the Free
// Software Foundation, either version 3 of the License, or (at your option)
// any later version.

// This program is distributed in the hope that it will be useful, but WITHOUT
// ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
// FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
// more details.

// You should have received a copy of the GNU General Public License along
// with this program.  If not, see <http://www.gnu.org/licenses/>.

// ROS 2
#include <rclcpp/rclcpp.hpp> 
// ROS2 노드 기본 API
#include <std_msgs/msg/header.hpp>
// /points_no_ground 같은 PointCloud2 데이터를 받을 때 사용
#include <sensor_msgs/msg/point_cloud2.hpp>
// PointCloud2 메시지 타입을 사용하기 위한 헤더 파일
#include <geometry_msgs/msg/pose_array.hpp>
// PoseArray 메시지 타입을 사용하기 위한 헤더 파일, centroid 출력을 위한 메시지
#include <visualization_msgs/msg/marker_array.hpp>
// MarkerArray 메시지 타입을 사용하기 위한 헤더 파일, bounding box 시각화를 위한 메시지
#include "adaptive_clustering_msgs/msg/cluster_array.hpp"
// ClusterArray 메시지 타입을 사용하기 위한 헤더 파일, 클러스터링 결과를 출력하기 위한 메시지,사용자 정의 클러스터 배열 메시지


// PCL
#include <pcl_conversions/pcl_conversions.h>
// PCL <<->> ROS 변환을 위한 헤더 파일
#include <pcl/filters/voxel_grid.h>
// VoxelGrid 필터를 사용하여 포인트 클라우드를 다운샘플링하기 위한 헤더 파일
#include <pcl/segmentation/extract_clusters.h>
// EuclideanClusterExtraction 클래스를 사용하여 클러스터링을 수행하기 위한 헤더 파일, 너무 높은 클라우드를 3D 격자(큐브) 형태로 나눠서 대표 점만 남기는 기능
#include <pcl/common/common.h>
// PCL의 공통 기능을 사용하기 위한 헤더 파일, 클러스터의 최소/최대 범위를 계산하는 데 사용,  bounding box 계산 등에 필요한 공통 유틸리티 함수들을 포함하고 있음
#include <pcl/common/centroid.h>
// PCL의 중심 계산 기능을 사용하기 위한 헤더 파일, centroid 계산 >> 각 클러스터들의 Pose 계산을 하기 편하게, 가까운 centroid끼리 묶어서 중복제거 등


//#define LOG

const int region_max_ = 10; 
// Change this value to match how far you want to detect., 포인트 클라우드를 최대 10개의 원형 구간으로 분할


// 노드 클래스 정의, daptiveClustering: ROS 2에서 사용하는 노드 클래스
class AdaptiveClustering : public rclcpp::Node {
  // 노드 클래스는 rclcpp::Node를 상속받아야 하며, 노드 이름은 "adaptive_clustering"으로 설정
  // 이 클래스는 포인트 클라우드를 받아서 클러스터링을 수행하고
  // 클러스터링 결과를 다양한 메시지 타입으로 pub & sub 하는 기능
  // 또한, 파라미터를 통해 센서 모델, 필터링 옵션 등을 설정할 수 있음 
public:
  AdaptiveClustering()
    : Node("adaptive_clustering"), // 생성자, 노드 이름을 "adaptive_clustering"으로 설정
        print_fps_(false), // FPS 출력 여부
        leaf_(1),  // 다운샘플링을 위한 리프 크기, 포인트 클라우드를 얼마나 다운샘플링할지 결정, 1은 다운샘플링을 하지 않겠다는 의미
        z_axis_min_(-1.0), // Z축 최소값, 포인트 클라우드에서 제거할 최소 높이
        z_axis_max_(1.0), // Z축 최대값, 포인트 클라우드에서 제거할 최대 높이
        cluster_size_min_(20), // 클러스터링 시 최소 클러스터 크기, 클러스터가 이 크기보다 작으면 무시
        cluster_size_max_(100), // 클러스터링 시 최대 클러스터 크기, 클러스터가 이 크기보다 크면 무시
        x_threshold_(0.5), // centroid 병합 기준 (x축)
        y_threshold_(0.5), // centroid 병합 기준 (y축)
        cone_position_z_(-0.6), // cone 위치의 Z축 오프셋, 클러스터링 결과에서 cone 위치를 조정하기 위한 값(표현용)
        frames_(0), // 프레임 카운터, FPS 계산을 위한 변수
        reset_(true) // 리셋 플래그, FPS 계산을 초기화하기 위한 변수
        { 
        // 노드 생성 시 파라미터를 선언하고 초기화
        this->declare_parameter<std::string>("sensor_model", "VLP-32"); // 센서 모델 파라미터, VLP-32, VLP-16, HDL-32E, HDL-64E 중 하나를 선택, 센서 모델에 따라 포인트 클라우드의 분할 방식이 달라짐
        // 예를 들어, VLP-16은 16개의 레이저를 사용, HDL-32E는 32개의 레이저를 사용
        // 이 파라미터는 노드가 시작될 때 기본값으로 "VLP-32"로 설정
        this->declare_parameter<bool>("print_fps", false); // FPS 출력 여부 파라미터, true로 설정하면 FPS를 출력
        this->declare_parameter<int>("leaf", 1);  // 다운샘플링을 위한 리프 크기 파라미터, 1로 설정하면 다운샘플링을 하지 않음
        this->declare_parameter<float>("z_axis_min", -1.);    // Z축 최소값 파라미터, -1.0으로 설정
        this->declare_parameter<float>("z_axis_max", 2.0);   // Z축 최대값 파라미터, 2.0으로 설정
        this->declare_parameter<float>("x_threshold", 0.5);  // centroid 병합 기준 (x축) 파라미터, 0.5로 설정
        this->declare_parameter<float>("y_threshold", 0.5);   // centroid 병합 기준 (y축) 파라미터, 0.5로 설정
        this->declare_parameter<float>("cone_position_z", -0.6);  // cone 위치의 Z축 오프셋 파라미터, -0.6으로 설정
        this->declare_parameter<int>("cluster_size_min", 3);  // 클러스터링 시 최소 클러스터 크기 파라미터, 3으로 설정
        this->declare_parameter<int>("cluster_size_max", 500);   // 클러스터링 시 최대 클러스터 크기 파라미터, 500으로 설정
        // 위에서 다 선언했는데 왜 다시 선언하나여??

        // print_fps_(false), 
        // leaf_(1),
        // z_axis_min_(-1.2),
        // z_axis_max_(1.0),
        // cluster_size_min_(20),
        // cluster_size_max_(100),
        // x_threshold_(0.5),
        // y_threshold_(0.5),
        // cone_position_z_(-0.6),
        // frames_(0),
        // reset_(true) {
    
        // this->declare_parameter<std::string>("sensor_model", "VLP-32");
        // this->declare_parameter<bool>("print_fps", false);
        // this->declare_parameter<int>("leaf", 1);
        // this->declare_parameter<float>("z_axis_min", -0.8);
        // this->declare_parameter<float>("z_axis_max", 2.0);
        // this->declare_parameter<float>("x_threshold", 0.5);
        // this->declare_parameter<float>("y_threshold", 0.5);
        // this->declare_parameter<float>("cone_position_z", -0.6);
        // this->declare_parameter<int>("cluster_size_min", 3);
        // this->declare_parameter<int>("cluster_size_max", 2200000);
      
        this->get_parameter("sensor_model", sensor_model_); // 센서 모델 파라미터를 가져옴. get_parameter는 노드의 파라미터를 가져오는 함수, 선언된 파라미터의 값을 가져와서 멤버 변수에 저장
        this->get_parameter("print_fps", print_fps_); // FPS 출력 여부 파라미터를 가져옴. print_fps_는 FPS를 출력할지 여부를 결정하는 변수, true로 설정되면 FPS를 출력
        this->get_parameter("leaf", leaf_); // 다운샘플링을 위한 리프 크기 파라미터를 가져옴. leaf_는 포인트 클라우드를 얼마나 다운샘플링할지 결정하는 변수, 1로 설정되면 다운샘플링을 하지 않음
        this->get_parameter("z_axis_min", z_axis_min_); // Z축 최소값 파라미터를 가져옴. z_axis_min_는 포인트 클라우드에서 제거할 최소 높이를 결정하는 변수, -1.0으로 설정
        this->get_parameter("z_axis_max", z_axis_max_); // Z축 최대값 파라미터를 가져옴. z_axis_max_는 포인트 클라우드에서 제거할 최대 높이를 결정하는 변수, 2.0으로 설정
        this->get_parameter("x_threshold", x_threshold_); // centroid 병합 기준 (x축) 파라미터를 가져옴. x_threshold_는 centroid 병합 시 x축 기준을 결정하는 변수, 0.5로 설정
        this->get_parameter("y_threshold", y_threshold_);
        this->get_parameter("cone_position_z", cone_position_z_);
        this->get_parameter("cluster_size_min", cluster_size_min_);
        this->get_parameter("cluster_size_max", cluster_size_max_);
      
    cluster_array_pub_ = this->create_publisher<adaptive_clustering_msgs::msg::ClusterArray>("clusters", 100); // 클러스터링 결과를 출력하는 퍼블리셔, adaptive_clustering_msgs::msg::ClusterArray 타입의 메시지를 퍼블리시
    // "clusters" 토픽으로 퍼블리시, 큐 사이즈는 100으로 설정
    // 이 퍼블리셔는 클러스터링된 포인트 클라우드 데이터를 구독하는 다른 노드에게 전달
    // adaptive_clustering_msgs::msg::ClusterArray는 사용자 정의 메시지 타입으로,
    // 클러스터링된 포인트 클라우드 데이터를 포함하는 메시지 타입
    // 이 메시지는 클러스터링된 포인트 클라우드 데이터를 포함하는 배열
    cloud_filtered_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("cloud_filtered", 100);
    // 필터링된 포인트 클라우드를 퍼블리시하는 퍼블리셔, sensor_msgs::msg::PointCloud2 타입의 메시지를 퍼블리시
    pose_array_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("cone_poses", 100);
    // 클러스터의 중심점을 PoseArray로 퍼블리시하는 퍼블리셔, geometry_msgs::msg::PoseArray 타입의 메시지를 퍼블리시
    marker_array_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("markers", 100);
    // 클러스터의 경계 박스를 시각화하기 위한 퍼블리셔, visualization_msgs::msg::MarkerArray 타입의 메시지를 퍼블리시
    
    point_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>( 
      "points_no_ground", 1, std::bind(&AdaptiveClustering::pointCloudCallback, this, std::placeholders::_1)); 
      // 포인트 클라우드를 구독하는 서브스크립션, "points_no_ground" 토픽에서 sensor_msgs::msg::PointCloud2 타입의 메시지를 구독
    // 이 서브스크립션은 포인트 클라우드 데이터를 받아서 클러스터링을 수행하는 콜백 함수를 등록
    // 콜백 함수는 AdaptiveClustering 클래스의 pointCloudCallback 메서드로, 
    // 포인트 클라우드 데이터를 받아서 클러스터링을 수행하고 결과를 퍼블리시하는 역할을 함
    // std::bind를 사용하여 콜백 함수를 바인딩, std::placeholders::_1은 콜백 함수가 호출될 때 전달되는 인자를 나타냄
    
    if (sensor_model_ == "VLP-16") {
      regions_ = {2, 3, 3, 3, 3, 3, 3, 2, 3, 3, 3, 3, 3, 3}; // VLP-16 센서 모델에 대한 원형 구간 설정, 16개의 레이저를 사용
    } 
    else if (sensor_model_ == "HDL-32E") {
      regions_ = {4, 5, 4, 5, 4, 5, 5, 4, 5, 4, 5, 5, 4, 5}; // HDL-32E 센서 모델에 대한 원형 구간 설정, 32개의 레이저를 사용
    } 
    else if (sensor_model_ == "HDL-64E") {
      regions_ = {14, 14, 14, 15, 14}; // HDL-64E 센서 모델에 대한 원형 구간 설정, 64개의 레이저를 사용
    } 
    else if (sensor_model_ == "VLP-32") {
      regions_ = {4, 5, 4, 5, 4, 5, 5, 4, 5, 4, 5, 5, 4, 5}; // VLP-32 센서 모델에 대한 원형 구간 설정, 32개의 레이저를 사용
    }
    else {
      RCLCPP_FATAL(this->get_logger(), "Unknown sensor model!"); // 알 수 없는 센서 모델에 대한 오류 메시지 출력
      rclcpp::shutdown(); // 노드 종료
    }
  }

private: 
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstPtr& ros_pc2_in) { 
    if (print_fps_) {
      if (reset_) {
        frames_ = 0;
        start_time_ = clock();
        reset_ = false;
      }
    }
    // 포인트 클라우드 콜백 함수, 포인트 클라우드 데이터를 받아서 클러스터링을 수행하고 결과를 퍼블리시하는 역할
    // ros_pc2_in은 sensor_msgs::msg::PointCloud2 타입의 포인트 클라우드 데이터 포인터
    // 이 콜백 함수는 포인트 클라우드 데이터가 수신될 때마다 호출됨
    // 포인트 클라우드 데이터를 받아서 클러스터링을 수행하고 결과를 퍼블리시하는 역할을 함
    
    /*** Convert ROS message to PCL ***/
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_pc_in(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*ros_pc2_in, *pcl_pc_in);
    // ROS 메시지를 PCL 포인트 클라우드로 변환,
    // sensor_msgs::msg::PointCloud2 타입의 메시지를 pcl::PointCloud<pcl::PointXYZI> 타입의 포인트 클라우드로 변환
    // pcl_pc_in은 pcl::PointCloud<pcl::PointXYZI> 타입의 포인터로,
    // 포인트 클라우드 데이터를 저장하는 데 사용됨, pcl::PointXYZI는 PCL에서 제공하는 포인트 타입으로, 
    // x, y, z 좌표와 intensity 값을 포함, intensity는 포인트의 강도를 나타내는 값

    /*** Downsampling + ground & ceiling removal ***/
    pcl::IndicesPtr pc_indices(new std::vector<int>);
    for (size_t i = 0; i < pcl_pc_in->size(); ++i) {
      if (i % leaf_ == 0) {
        if (pcl_pc_in->points[i].z >= z_axis_min_ && pcl_pc_in->points[i].z <= z_axis_max_) {
          pc_indices->push_back(i);
        }
      }
    }
    // 다운샘플링 및 지면과 천장 제거,
    // pcl_pc_in에서 포인트를 다운샘플링하고, z축 범위에 해당하는 포인트만 선택
    // leaf_는 다운샘플링을 위한 리프 크기로, 
    // 포인트 클라우드의 포인트를 leaf_ 크기 단위로 그룹화하여 다운샘플링
    // pc_indices는 선택된 포인트의 인덱스를 저장하는 std::vector<int> 타입의 포인터로
    // pcl_pc_in에서 z축 범위에 해당하는 포인트의 인덱스를 저장
    // pcl_pc_in->points[i].z는 포인트의 z 좌표
    // pcl_pc_in->size()는 포인트 클라우드의 포인트 개수를 가져옴
    // pcl_pc_in->points[i].z >= z_axis_min_ && pcl_pc_in->points[i].z <= z_axis_max_는
    // 포인트의 z 좌표가 z_axis_min_와 z_axis_max_ 사이에 있는지 확인하는 조건

    /*** Divide the point cloud into nested circular regions ***/
    std::array<std::vector<int>, region_max_> indices_array;
    for (size_t i = 0; i < pc_indices->size(); i++) {
      float range = 0.0;
      for (int j = 0; j < region_max_; j++) {
        float d2 = pcl_pc_in->points[(*pc_indices)[i]].x * pcl_pc_in->points[(*pc_indices)[i]].x +
                   pcl_pc_in->points[(*pc_indices)[i]].y * pcl_pc_in->points[(*pc_indices)[i]].y +
                   pcl_pc_in->points[(*pc_indices)[i]].z * pcl_pc_in->points[(*pc_indices)[i]].z;
        if (d2 > range * range && d2 <= (range + regions_[j]) * (range + regions_[j])) {
          indices_array[j].push_back((*pc_indices)[i]);
          break;
        }
        range += regions_[j];
      }
    }
    // 포인트 클라우드를 원형 구간으로 나누기,
    // pc_indices에서 선택된 포인트의 인덱스를 사용하여 pcl_pc_in의 포인트를 순회
    // indices_array는 region_max_ 크기의 std::array<std::vector<int>, region_max_> 타입의 배열로,
    // 각 원형 구간에 해당하는 포인트의 인덱스를 저장
    // pcl_pc_in->points[(*pc_indices)[i]].x, 
    // pcl_pc_in->points[(*pc_indices)[i]].y, 
    // pcl_pc_in->points[(*pc_indices)[i]].z는
    // 선택된 포인트의 x, y, z 좌표를 가져옴
    // d2는 선택된 포인트의 3D 거리 제곱
    // range는 현재 원형 구간의 반지름을 나타내며, regions_ 배열을 사용하여 각 원형 구간의 크기를 정의
    // regions_ 배열은 각 원형 구간의 크기를 정의하는 std::vector<int> 타입의 배열로
    // VLP-16, HDL-32E, HDL-64E, VLP-32 센서 모델에 따라 다르게 설정됨
    // d2 > range * range는 현재 포인트의 거리 제곱이 현재 원형 구간의 반지름 제곱보다 큰지 확인
    // d2 <= (range + regions_[j]) * (range + regions_[j])는 현재 포인트의 거리 제곱이 다음 원형 구간의 반지름 제곱보다 작거나 같은지 확인
    // 이 조건을 만족하는 경우, 현재 포인트는 해당 원형 구간에 속한다고 판단하고
    // indices_array[j].push_back((*pc_indices)[i])를 통해 해당 원형 구간의 인덱스 배열에 포인트 인덱스를 추가
    // break;를 사용하여 현재 포인트에 대한 처리를 종료하고 다음 포인트로 넘어감  

    /*** Euclidean clustering ***/
    float tolerance = 0.0;
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr, Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZI>::Ptr>> clusters;
    // 클러스터링을 위한 tolerance 초기화, tolerance는 클러스터링 시 사용되는 거리 임계값
    // tolerance는 클러스터링 시 포인트 간의 거리가 tolerance 이하인 경우
    // 같은 클러스터로 간주되도록 설정, tolerance는 클러스터링의 정확도에 영향을 미침
    // tolerance는 클러스터링 시 포인트 간의 거리를 기준으로 클러스터를 형성하는 데 사용
    // tolerance가 작을수록 클러스터가 더 작고, tolerance가 클수록 클러스터가 더 크게 형성됨

    for (int i = 0; i < region_max_; i++) {
      tolerance += 0.1; // tolerance를 0.1씩 증가시킴, tolerance는 클러스터링 시 사용되는 거리 임계값 
      if (indices_array[i].size() > cluster_size_min_) // 클러스터링을 수행할 원형 구간의 포인트 개수가 최소 클러스터 크기보다 큰 경우
      {
        std::shared_ptr<const std::vector<int>> indices_array_ptr = std::make_shared<const std::vector<int>>(indices_array[i]); // indices_array[i]를 const std::vector<int> 타입의 포인터로 변환
        pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>); // KdTree를 사용하여 포인트 클라우드의 공간 검색을 위한 트리를 생성
        tree->setInputCloud(pcl_pc_in, indices_array_ptr); // KdTree에 입력 포인트 클라우드와 인덱스 배열을 설정
        
        std::vector<pcl::PointIndices> cluster_indices; // 클러스터링 결과를 저장할 벡터, pcl::PointIndices는 클러스터의 포인트 인덱스를 저장하는 구조체
        pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec; // EuclideanClusterExtraction 클래스를 사용하여 클러스터링을 수행 
        ec.setClusterTolerance(tolerance); // 클러스터링 시 사용되는 거리 임계값을 설정
        ec.setMinClusterSize(cluster_size_min_); // 최소 클러스터 크기를 설정
        ec.setMaxClusterSize(cluster_size_max_); // 최대 클러스터 크기를 설정
        ec.setSearchMethod(tree); // KdTree를 클러스터링 검색 방법으로 설정
        ec.setInputCloud(pcl_pc_in); // 클러스터링을 수행할 입력 포인트 클라우드를 설정
        ec.setIndices(indices_array_ptr); // 클러스터링을 수행할 인덱스 배열을 설정
        ec.extract(cluster_indices); // 클러스터링을 수행하여 클러스터 인덱스를 추출
        
        for (const auto& it : cluster_indices) {
          pcl::PointCloud<pcl::PointXYZI>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZI>); // 클러스터를 저장할 포인트 클라우드 생성
          for (const auto& pit : it.indices) { // 클러스터의 각 포인트 인덱스를 순회
            cluster->points.push_back(pcl_pc_in->points[pit]); // 포인트 클라우드에서 해당 인덱스의 포인트를 클러스터에 추가
          } // 클러스터에 포인트를 추가
          cluster->width = cluster->size(); // 클러스터의 너비를 클러스터의 포인트 개수로 설정
          cluster->height = 1; // 클러스터의 높이를 1로 설정, 클러스터는 2D 평면에서 표현되므로 높이는 1로 설정
          cluster->is_dense = true; // 클러스터가 밀집되어 있다고 가정
          clusters.push_back(cluster); // 클러스터를 클러스터 벡터에 추가
        } // 클러스터링된 포인트 클라우드를 clusters 벡터에 추가
      } // 클러스터링을 수행할 원형 구간의 포인트 개수가 최소 클러스터 크기보다 큰 경우에만 클러스터링 수행
    } // region_max_ 만큼 반복하여 각 원형 구간에 대해 클러스터링 수행 
    
    /*** Output ***/
    if (cloud_filtered_pub_->get_subscription_count() > 0) {
      pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_pc_out(new pcl::PointCloud<pcl::PointXYZI>); // 필터링된 포인트 클라우드를 저장할 PCL 포인트 클라우드 생성
      sensor_msgs::msg::PointCloud2 ros_pc2_out; // 필터링된 포인트 클라우드를 ROS 메시지로 변환하기 위한 변수
      pcl::copyPointCloud(*pcl_pc_in, *pc_indices, *pcl_pc_out); // pcl_pc_in에서 pc_indices에 해당하는 포인트를 필터링하여 pcl_pc_out에 저장
      pcl::toROSMsg(*pcl_pc_out, ros_pc2_out); // PCL 포인트 클라우드를 ROS 메시지로 변환
      cloud_filtered_pub_->publish(ros_pc2_out); // 필터링된 포인트 클라우드를 퍼블리시
    } // 필터링된 포인트 클라우드를 퍼블리시, cloud_filtered_pub_ 퍼블리셔가 구독자를 가지고 있는 경우에만 퍼블리시
    
    adaptive_clustering_msgs::msg::ClusterArray cluster_array; // 클러스터링 결과를 저장할 사용자 정의 메시지 타입인 ClusterArray 생성
    geometry_msgs::msg::PoseArray pose_array;  // 클러스터의 중심점을 PoseArray로 저장하기 위한 변수 생성
    visualization_msgs::msg::MarkerArray marker_array; // 클러스터의 경계 박스를 시각화하기 위한 MarkerArray 생성
    
    for (size_t i = 0; i < clusters.size(); i++) {
      if (cluster_array_pub_->get_subscription_count() > 0) {
        sensor_msgs::msg::PointCloud2 ros_pc2_out; // 클러스터링된 포인트 클라우드를 ROS 메시지로 변환하기 위한 변수
        pcl::toROSMsg(*clusters[i], ros_pc2_out); // PCL 포인트 클라우드를 ROS 메시지로 변환
        cluster_array.clusters.push_back(ros_pc2_out); // 클러스터링된 포인트 클라우드를 ClusterArray 메시지에 추가
      } // 클러스터링 결과를 퍼블리시하는 퍼블리셔가 구독자를 가지고 있는 경우에만 퍼블리시
      
      if (pose_array_pub_->get_subscription_count() > 0) {
        std::vector<Eigen::Vector4f> centroids; // 클러스터의 중심점을 저장하기 위한 벡터, Eigen::Vector4f는 4차원 벡터로, x, y, z 좌표와 w(회전) 값을 포함
        for (const auto& cluster : clusters) {
            Eigen::Vector4f centroid; // 클러스터의 중심점을 저장할 변수
            pcl::compute3DCentroid(*cluster, centroid); // 클러스터의 중심점을 계산
            centroids.push_back(centroid); // 각 클러스터의 중심점을 계산하여 centroids 벡터에 저장
        } // 각 클러스터의 중심점을 계산하여 centroids 벡터에 저장

        std::vector<bool> used(clusters.size(), false); // 사용된 클러스터를 추적하기 위한 벡터, false로 초기화

        for (size_t i = 0; i < clusters.size(); ++i) {
            if (used[i]) continue; // 이미 사용된 클러스터는 건너뜀
            used[i] = true; // 현재 클러스터를 사용된 것으로 표시

            Eigen::Vector4f avg_centroid = centroids[i]; // 현재 클러스터의 중심점을 평균 중심점으로 초기화
            int count = 1; // 현재 클러스터를 포함한 중심점의 개수

            for (size_t j = i + 1; j < clusters.size(); ++j) {
                if (used[j]) continue; 
                if (std::abs(centroids[i][0] - centroids[j][0]) <= x_threshold_ && 
                    std::abs(centroids[i][1] - centroids[j][1]) <= y_threshold_) // x, y 좌표 차이가 기준 이하인 경우
                    {
                    avg_centroid += centroids[j]; // 평균 중심점에 현재 클러스터의 중심점을 추가
                    count++; // 중심점의 개수를 증가
                    used[j] = true; // 현재 클러스터를 사용된 것으로 표시
                } // 중심점의 x, y 좌표 차이가 기준 이하인 경우, centroid 병합
            } // centroid 병합

            avg_centroid /= count; // 평균 중심점을 계산, count로 나누어 평균값을 구함
            // centroid 병합 후 평균 중심점 계산

            geometry_msgs::msg::Pose pose; // Pose 메시지 생성, centroid 병합 후 평균 중심점을 Pose로 변환
            pose.position.x = avg_centroid[0]; // 평균 중심점의 x 좌표
            pose.position.y = avg_centroid[1]; // 평균 중심점의 y 좌표
            pose.position.z = avg_centroid[2]; // 평균 중심점의 z 좌표
            pose.orientation.w = 1; // 회전 정보는 단순히 w=1로 설정, 회전 정보는 필요하지 않으므로 단순화
            pose_array.poses.push_back(pose); // PoseArray에 평균 중심점을 추가
        } // centroid 병합 후 평균 중심점 계산
        
#ifdef LOG // 디버깅용 로그 출력, 클러스터의 최소/최대 범위를 출력
        Eigen::Vector4f min, max; // 클러스터의 최소/최대 범위를 저장할 변수
        pcl::getMinMax3D(*clusters[i], min, max); // 클러스터의 최소/최대 범위를 계산
        std::cerr << ros_pc2_in->header().frame_id << " " // 프레임 ID
                  << ros_pc2_in->header().stamp.sec << " " // 타임스탬프 초
                  << min[0] << " " // 클러스터의 최소 x 좌표
                  << min[1] << " " // 클러스터의 최소 y 좌표
                  << min[2] << " " // 클러스터의 최소 z 좌표
                  << max[0] << " " // 클러스터의 최대 x 좌표
                  << max[1] << " " // 클러스터의 최대 y 좌표
                  << max[2] << " " // 클러스터의 최대 z 좌표
                  << std::endl; // 클러스터의 최소/최대 범위를 출력
#endif 
      } // centroid 병합 후 평균 중심점 계산
      
      if (marker_array_pub_->get_subscription_count() > 0) {
        Eigen::Vector4f min, max; // 클러스터의 최소/최대 범위를 저장할 변수
        // 클러스터의 경계 박스를 시각화하기 위한 MarkerArray 생성
        // marker_array는 visualization_msgs::msg::MarkerArray 타입의 메시지로, 클러스터의 경계 박스를 시각화하기 위한 마커를 포함
        // 각 클러스터에 대해 경계 박스를 계산하고 시각화하기 위한 마커를 생성
        // 클러스터의 경계 박스를 계산하여 시각화하기 위한 MarkerArray
        // 클러스터의 최소/최대 범위를 계산하여 min, max 변수에 저장
        // pcl::getMinMax3D는 PCL에서 제공하는 함수로,
        // 포인트 클라우드의 최소/최대 범위를 계산하여 min, max 변수에 저장
        // min은 클러스터의 최소 범위를 나타내는 4차원 벡터로, 
        pcl::getMinMax3D(*clusters[i], min, max); // 클러스터의 최소/최대 범위를 계산하여 min, max 변수에 저장
  
        
        visualization_msgs::msg::Marker marker; // Marker 메시지 생성, 클러스터의 경계 박스를 시각화하기 위한 마커
        marker.header = ros_pc2_in->header; // 마커의 헤더를 포인트 클라우드의 헤더로 설정
        marker.ns = "adaptive_clustering"; // 마커의 네임스페이스를 "adaptive_clustering"으로 설정
        marker.id = i; // 마커의 ID를 클러스터 인덱스로 설정, 각 클러스터에 대해 고유한 ID를 부여
        marker.type = visualization_msgs::msg::Marker::LINE_LIST; // 마커의 타입을 LINE_LIST로 설정, 클러스터의 경계 박스를 선으로 시각화
        
        geometry_msgs::msg::Point p[24]; // 클러스터의 경계 박스를 구성하는 24개의 점을 저장하기 위한 배열
        p[0].x = max[0];  p[0].y = max[1];  p[0].z = max[2]; // 클러스터의 최대 x, y, z 좌표를 점으로 설정
        p[1].x = min[0];  p[1].y = max[1];  p[1].z = max[2]; // 클러스터의 최소 x, 최대 y, 최대 z 좌표를 점으로 설정
        p[2].x = max[0];  p[2].y = max[1];  p[2].z = max[2]; // 클러스터의 최대 x, y, z 좌표를 점으로 설정
        p[3].x = max[0];  p[3].y = min[1];  p[3].z = max[2]; 
        p[4].x = max[0];  p[4].y = max[1];  p[4].z = max[2]; 
        p[5].x = max[0];  p[5].y = max[1];  p[5].z = min[2];
        p[6].x = min[0];  p[6].y = min[1];  p[6].z = min[2];
        p[7].x = max[0];  p[7].y = min[1];  p[7].z = min[2];
        p[8].x = min[0];  p[8].y = min[1];  p[8].z = min[2];
        p[9].x = min[0];  p[9].y = max[1];  p[9].z = min[2];
        p[10].x = min[0]; p[10].y = min[1]; p[10].z = min[2];
        p[11].x = min[0]; p[11].y = min[1]; p[11].z = max[2];
        p[12].x = min[0]; p[12].y = max[1]; p[12].z = max[2];
        p[13].x = min[0]; p[13].y = max[1]; p[13].z = min[2];
        p[14].x = min[0]; p[14].y = max[1]; p[14].z = max[2];
        p[15].x = min[0]; p[15].y = min[1]; p[15].z = max[2];
        p[16].x = max[0]; p[16].y = min[1]; p[16].z = max[2];
        p[17].x = max[0]; p[17].y = min[1]; p[17].z = min[2];
        p[18].x = max[0]; p[18].y = min[1]; p[18].z = max[2];
        p[19].x = min[0]; p[19].y = min[1]; p[19].z = max[2];
        p[20].x = max[0]; p[20].y = max[1]; p[20].z = min[2];
        p[21].x = min[0]; p[21].y = max[1]; p[21].z = min[2];
        p[22].x = max[0]; p[22].y = max[1]; p[22].z = min[2];
        p[23].x = max[0]; p[23].y = min[1]; p[23].z = min[2];
        for (int i = 0; i < 24; i++) {
          marker.points.push_back(p[i]); // 클러스터의 경계 박스를 구성하는 24개의 점을 마커에 추가
        } // 클러스터의 경계 박스를 구성하는 24개의 점을 마커에 추가
        
        marker.scale.x = 0.02; // 마커의 선 두께를 0.02로 설정, 클러스터의 경계 박스를 시각화하기 위한 선의 두께
        marker.color.a = 1.0; // 마커의 투명도를 1.0으로 설정, 완전히 불투명하게 설정
        marker.color.r = 0.0; // 마커의 색상을 빨간색으로 설정
        marker.color.g = 1.0; // 마커의 색상을 녹색으로 설정 
        marker.color.b = 0.5;   // 마커의 색상을 파란색으로 설정, 클러스터의 경계 박스를 시각화하기 위한 색상 설정
        marker.lifetime = rclcpp::Duration(0, 300000000); // 마커의 생명 주기를 0.3초로 설정, 마커가 0.3초 후에 사라지도록 설정
        marker_array.markers.push_back(marker); // 클러스터의 경계 박스를 시각화하기 위한 MarkerArray에 클러스터의 경계 박스 마커를 추가
      } // 클러스터의 경계 박스를 시각화하기 위한 MarkerArray에 클러스터의 경계 박스 마커를 추가
    } // 클러스터링 결과를 퍼블리시하는 퍼블리셔가 구독자를 가지고 있는 경우에만 퍼블리시
    
    if (!cluster_array.clusters.empty()) {
      cluster_array.header = ros_pc2_in->header; // 클러스터링 결과의 헤더를 포인트 클라우드의 헤더로 설정
      cluster_array_pub_->publish(cluster_array); // 클러스터링 결과를 퍼블리시
    } // 클러스터링 결과를 퍼블리시하는 퍼블리셔가 구독자를 가지고 있는 경우에만 퍼블리시
    
    if (!pose_array.poses.empty()) {
      pose_array.header = ros_pc2_in->header; // PoseArray의 헤더를 포인트 클라우드의 헤더로 설정
      pose_array_pub_->publish(pose_array); // PoseArray를 퍼블리시
    } // PoseArray를 퍼블리시하는 퍼블리셔가 구독자를 가지고 있는 경우에만 퍼블리시
    
    if (!marker_array.markers.empty()) {
      marker_array_pub_->publish(marker_array);   // MarkerArray를 퍼블리시
    } // MarkerArray를 퍼블리시하는 퍼블리셔가 구독자를 가지고 있는 경우에만 퍼블리시
    
    if (print_fps_) {
      if (++frames_ > 10) {
        RCLCPP_INFO(this->get_logger(), "[adaptive_clustering] fps = %f, timestamp = %ld", 
                    float(frames_) / (float(clock() - start_time_) / CLOCKS_PER_SEC), clock() / CLOCKS_PER_SEC); // FPS를 출력
        reset_ = true; // FPS를 출력하고 프레임 수를 초기화
      } // 10 프레임마다 FPS를 출력
    } // FPS를 출력하는 경우, 프레임 수를 증가시키고 10 프레임마다 FPS를 출력
  } // 포인트 클라우드 콜백 함수, 포인트 클라우드 데이터를 받아서 클러스터링을 수행하고 결과를 퍼블리시하는 역할
  
  rclcpp::Publisher<adaptive_clustering_msgs::msg::ClusterArray>::SharedPtr cluster_array_pub_; // 클러스터링 결과를 퍼블리시하는 퍼블리셔, adaptive_clustering_msgs::msg::ClusterArray 타입의 메시지를 퍼블리시
  // adaptive_clustering_msgs::msg::ClusterArray는 사용자 정의 메시지 타입으로,
  // 클러스터링된 포인트 클라우드 데이터를 포함하는 메시지 타입
  // 이 메시지는 클러스터링된 포인트 클라우드 데이터를 포함하는 배열
  // 클러스터링된 포인트 클라우드 데이터를 구독하는 다른 노드에게 전달
  // rclcpp::Publisher는 ROS2에서 퍼블리셔를 생성
  // rclcpp::Publisher는 ROS2에서 퍼블리셔를 생성하는 클래스
  // SharedPtr는 스마트 포인터로, 퍼블리셔의 메모리를 자동으로 관리
  // rclcpp::Publisher<adaptive_clustering_msgs::msg::ClusterArray>::SharedPtr는 
  // adaptive_clustering_msgs::msg::ClusterArray 타입의 메시지를 퍼블리시하는 퍼블리셔의 스마트 포인터 타입
  // adaptive_clustering_msgs::msg::ClusterArray는 사용자 정의 메시지 타입으로,
  // 클러스터링된 포인트 클라우드 데이터를 포함하는 메시지 타입
  // 이 퍼블리셔는 클러스터링된 포인트 클라우드 데이터를
  // 구독하는 다른 노드에게 전달
  // adaptive_clustering_msgs::msg::ClusterArray는 사용자 정의 메시지 타입으로,
  // 클러스터링된 포인트 클라우드 데이터를 포함하는 메시지 타입
  // 이 퍼블리셔는 클러스터링된 포인트 클라우드 데이터를
  // 구독하는 다른 노드에게 전달
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_filtered_pub_; // 필터링된 포인트 클라우드를 퍼블리시하는 퍼블리셔, sensor_msgs::msg::PointCloud2 타입의 메시지를 퍼블리시
  // sensor_msgs::msg::PointCloud2는 ROS2에서 제공하는 표준 포인트 클라우드 메시지 타입
  // 이 퍼블리셔는 필터링된 포인트 클라우드 데이터를 구독하는 다른 노드에게 전달
  // rclcpp::Publisher는 ROS2에서 퍼블리셔를 생성하는 클래스
  // SharedPtr는 스마트 포인터로, 퍼블리셔의 메모리를 자동으로 관리
  // rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr는 
  //  sensor_msgs::msg::PointCloud2 타입의 메시지를 퍼블리시하는 퍼블리셔의 스마트 포인터 타입
  // sensor_msgs::msg::PointCloud2는 ROS2에서 제공하는 표준 포인트 클라우드 메시지 타입
  // 이 퍼블리셔는 필터링된 포인트 클라우드 데이터를 구독하는 다른 노드에게 전달
  // sensor_msgs::msg::PointCloud2는 ROS2에서 제공하는 표준 포인트 클라우드 메시지
  // 이 퍼블리셔는 필터링된 포인트 클라우드 데이터를 구독하는 다른 노드에게 전달
  // sensor_msgs::msg::PointCloud2는 ROS2에서 제공하는 표준 포인트 클라우드 메시지 타입
  // 이 퍼블리셔는 필터링된 포인트 클라우드 데이터를 구독하는 다른 노드에게
  // 전달
  // sensor_msgs::msg::PointCloud2는 ROS2에서 제공하는 표준 포인트 클라우드 메시지 타입
  // 이 퍼블리셔는 필터링된 포인트 클라우드 데이터를 구독하는 다른 노드에게 전달
  // sensor_msgs::msg::PointCloud2는 ROS2에서 제공하는 표준 포인트 클라우드 메시지     
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_array_pub_; // 클러스터의 중심점을 PoseArray로 퍼블리시하는 퍼블리셔, geometry_msgs::msg::PoseArray 타입의 메시지를 퍼블리시
  // geometry_msgs::msg::PoseArray는 ROS2에서 제공하는 표준 PoseArray 메시지 타입
  // 이 퍼블리셔는 클러스터의 중심점을 구독     
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_pub_; // 클러스터의 경계 박스를 시각화하기 위한 퍼블리셔, visualization_msgs::msg::MarkerArray 타입의 메시지를 퍼블리시
  // visualization_msgs::msg::MarkerArray는 ROS2에서 제공하는 표준 MarkerArray 메시지 타입
  // 이 퍼블리셔는 클러스터의 경계 박스를 구독하는 다른 노드에게 전달
  // visualization_msgs::msg::MarkerArray는 ROS2에서 제공하는 표준 MarkerArray 메시지 타입
  // 이 퍼블리셔는 클러스터의 경계 박스를 구독하는 다른 노드에게 전달
  // visualization_msgs::msg::MarkerArray는 ROS2에서 제공하는 표준 MarkerArray 메시지 타입
  // 이 퍼블리셔는 클러스터의 경계 박스를 구독하는 다른 노드에게 전달
  // visualization_msgs::msg::MarkerArray는 ROS2에서 제공하는 표준 MarkerArray 메시지 타입
  // 이 퍼블리셔는 클러스터의 경계 박스를 구독하는 다른 노드에게 전달 
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_; // 포인트 클라우드 데이터를 구독하는 퍼블리셔, sensor_msgs::msg::PointCloud2 타입의 메시지를 구독
  // sensor_msgs::msg::PointCloud2는 ROS2에서 제공하는 표준 포인트 클라우드 메시지 타입
  // 이 퍼블리셔는 포인트 클라우드 데이터를 구독하는 다른 노드에게 전달
  // rclcpp::Subscription는 ROS2에서 구독자를 생성하는 클래스
  // SharedPtr는 스마트 포인터로, 구독자의 메모리를 자동으로 관리
  // rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr는 
  // sensor_msgs::msg::PointCloud2 타입의 메시지를 구독하는 퍼블리셔의 스마트 포인터 타입
  // sensor_msgs::msg::PointCloud2는 ROS2에서 제공하는 표준 포인트 클라우드 메시지 타입
  // 이 퍼블리셔는 포인트 클라우드 데이터를 구독하는 다른 노드에게 전달
  // sensor_msgs::msg::PointCloud2는 ROS2에서 제공하는 표준 포인트 클라우드 메시지 타입
  // 이 퍼블리셔는 포인트 클라우드 데이터를 구독하는 다른 노드에게 전달

  
  bool print_fps_; // FPS를 출력할지 여부를 나타내는 플래그, true이면 FPS를 출력, false이면 출력하지 않음
  int leaf_; // 다운샘플링을 위한 리프 크기, 포인트 클라우드의 포인트를 leaf_ 크기 단위로 그룹화하여 다운샘플링
  float z_axis_min_; // z축 최소값, 포인트 클라우드에서 z축 범위를 설정하기 위한 변수
  float z_axis_max_; // z축 최대값, 포인트 클라우드에서 z축 범위를 설정하기 위한 변수
  float x_threshold_;  // 클러스터링 시 x 좌표 차이의 임계값, 클러스터링 시 포인트 간의 x 좌표 차이가 x_threshold_ 이하인 경우 같은 클러스터로 간주
  float y_threshold_; // 클러스터링 시 y 좌표 차이의 임계값, 클러스터링 시 포인트 간의 y 좌표 차이가 y_threshold_ 이하인 경우 같은 클러스터로 간주
  float cone_position_z_; // 원뿔 위치의 z 좌표, 클러스터링 시 원뿔 위치의 z 좌표를 설정하기 위한 변수
  int cluster_size_min_; // 클러스터링 시 최소 클러스터 크기, 클러스터링 시 포인트 개수가 cluster_size_min_ 이상인 경우에만 클러스터로 간주
  int cluster_size_max_; // 클러스터링 시 최대 클러스터 크기, 클러스터링 시 포인트 개수가 cluster_size_max_ 이하인 경우에만 클러스터로 간주
  
  std::vector<int> regions_; // 원형 구간의 크기를 정의하는 벡터, 각 원형 구간의 크기를 정의하는 std::vector<int> 타입의 벡터
  
  int frames_; // 프레임 수를 카운트하는 변수, FPS를 계산하기 위한 변수
  clock_t start_time_; // FPS 계산을 위한 시작 시간, clock() 함수를 사용하여 현재 시간을 가져옴
  bool reset_; // FPS를 초기화할지 여부를 나타내는 플래그, true이면 FPS를 초기화, false이면 초기화하지 않음
  std::string sensor_model_; // 센서 모델을 나타내는 문자열, VLP-16, HDL-32E, HDL-64E, VLP-32 중 하나로 설정
}; 

int main(int argc, char **argv) {
  rclcpp::init(argc, argv); // ROS2 초기화, rclcpp::init은 ROS2 노드를 초기화하는 함수
  rclcpp::spin(std::make_shared<AdaptiveClustering>()); // AdaptiveClustering 노드를 생성하고 실행, rclcpp::spin은 노드를 실행하는 함수
  rclcpp::shutdown(); // ROS2 종료, rclcpp::shutdown은 ROS2 노드를 종료하는 함수
  return 0; // 프로그램 종료, main 함수는 프로그램의 진입점으로, rclcpp::init을 통해 ROS2를 초기화하고
  // rclcpp::spin을 통해 AdaptiveClustering 노드를 실행한 후, rclcpp::shutdown을 통해 ROS2를 종료
  // 프로그램이 정상적으로 종료되었음을 나타내기 위해 0을 반환
} // main 함수, ROS2 노드를 초기화하고 실행하는 역할
