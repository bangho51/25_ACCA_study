import os # ROS 2 런치 관련 모듈 


from launch import LaunchDescription # 런치파일 속 노드를 어떤 설정으로 실행 할 지 정한 순서를 리스트로 하고  [LaunchDescription] 안에 넣어 런치 시스템에 전
from launch.actions import DeclareLaunchArgument # 런치파일 속 노드의 파라미터를 선언하는 모듈
from launch.substitutions import LaunchConfiguration # 런치파일 속 노드의 파라미터를 선언할 때 사용되는 모듈
from launch_ros.actions import Node # ROS 2 런치파일 속 노드를 실행하기 위한 모듈

def generate_launch_description(): # 런치파일을 생성하는 함수
    return LaunchDescription([
        Node(
            package='adaptive_clustering', # adaptive_clustering 패키지를 실행하기 위한 설정
            executable='adaptive_clustering', # 실행할 노드의 패키지와 실행 파일을 지정
            name='adaptive_clustering', # 노드의 이름을 'adaptive_clustering'으로 설정
            parameters=[{'print_fps': True}], # adaptive_clustering 노드의 파라미터 설정
        ), # adaptive_clustering 노드를 실행하기 위한 설정
    ] ) # 런치파일에 노드를 추가하는 부분

if __name__ == '__main__': # 런치파일이 직접 실행될 때의 진입점S
    generate_launch_description() # 이 파일은 ROS 2 런치 시스템에서 adaptive_clustering 노드를 실행하기 위한 설정을 포함하고 있습니다.
    # 이 파일을 실행하면 adaptive_clustering 노드가 실행되고, 해당 노드의 파라미터로 'print_fps'가 True로 설정됩니다.
    # 이 노드는 adaptive_clustering 패키지에 속하며, 노드의 이름은 'adaptive_clustering'입니다.
    # 이 파일은 ROS 2 런치 시스템에서 adaptive_clustering 노드를 실행하기 위한 설정을 포함하고 있습니다.
    # 이 파일을 실행하면 adaptive_clustering 노드가 실행되고, 해당 노드의 파라미터로 'print_fps'가 True로 설정됩니다.
    # 이 노드는 adaptive_clustering 패키지에 속하며, 노드의 이름은 'adaptive_clustering'입니다.
