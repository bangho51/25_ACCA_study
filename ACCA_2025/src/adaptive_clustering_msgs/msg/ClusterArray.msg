std_msgs/Header header
sensor_msgs/PointCloud2[] clusters

//std_msgs/Header header
의미: 메시지의 메타데이터(시간, 프레임 등)를 담는 헤더 정보입니다.

주요 필드:

stamp: 메시지가 생성된 시간(타임스탬프)

frame_id: 기준 좌표계(예: "map", "base_link")의 이름

용도: 모든 ROS 메시지에서 시간 동기화, 프레임 일관성 보장을 위해 공통적으로 사용합니다.

2. sensor_msgs/PointCloud2[] clusters
의미: 여러 개의 포인트 클라우드(3D 점 데이터) 데이터를 배열로 묶은 변수입니다.

타입 설명:

sensor_msgs/PointCloud2: 3차원 공간의 점들의 집합(예: 라이다, 3D 카메라 등에서 추출된 데이터)

용도: 보통 "클러스터링" 작업에서 사용할 때, 각 클러스터(개별 객체 등)를 하나의 포인트클라우드로 저장해 배열에 담을 때 사용합니다.

요약
header는 메시지의 시간, 공간적 정보를 담는 메타데이터입니다.

clusters는 여러 개의 포인트 클라우드 데이터를 담는 배열입니다.

이 구조는 여러 3D 객체 혹은 영역(클러스터) 각각을 별도의 포인트클라우드로 관리하는 ROS 메시지 포맷에서 흔히 활용됩니다. //
