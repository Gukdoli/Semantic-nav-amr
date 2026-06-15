# Semantic Navigation AMR

자연어 명령("소화기 옆으로 가")으로 객체 기반 목표를 지정하는 ROS 2 시맨틱 내비게이션 시스템.
전체 설계는 @docs/SPEC.md 참조. 마일스톤 단위로 구현하며, 현재 진행 상황은 docs/SPEC.md의 체크박스 기준.

## 환경

- ROS 2 Humble / Ubuntu 22.04. Python 3.10+, rclpy 기반 (C++ 노드는 성능 이슈가 있을 때만).
- 로봇: AgileX **Scout v2** (skid-steering 4륜), TurtleBot3 아님. 패키지 `agilex_scout` / `scout_nav2`.
- **Gazebo = Ignition Gazebo 6 = Fortress (gz sim 계열, Classic 아님).** 스폰 `ros_gz_sim create`,
  토픽 브릿지 `ros_gz_bridge parameter_bridge` + `ignition.msgs.*`. 렌더 센서(카메라/lidar)는 gz sim
  방식만 사용 — **`libgazebo_ros_camera.so`(Classic) 절대 금지.**
- RGB-D 카메라(Intel RealSense **D435i**)는 M2에서 통합 완료 → 아래 "카메라" 섹션.
- Nav2, tf2_ros, cv_bridge 사용.

## 실제 프레임/토픽 (코드 분석으로 확인 — 표준과 다름, 주의)

- **로봇 본체 프레임은 `mobile_robot_base_link`** (표준 `base_link` 아님).
  TF 체인: `world → map`(static_tf) → `odom → base_footprint`(gz odometry-publisher, `/scout/tf`)
  → `base_link`(0-offset 더미) → `mobile_robot_base_link` → 센서들.
  - Nav2 costmap `robot_base_frame: mobile_robot_base_link`, AMCL `base_frame_id: base_footprint`.
  - 카메라 부모 링크는 **`front_mount`** (전방 상단, mobile_robot_base_link 기준 xyz=0.325 0 0.1127).
- 시뮬 토픽 (ros_gz_bridge로 노출): `/cmd_vel`, `/odometry`, `/laser_scan`(2D), `/points`(3D
  PointCloud2), `/imu`, `/camera/...`(아래), `/tf`, `/tf_static`, `/clock`. (`/scan` 아님 → `/laser_scan`.)
- 월드: AWS RoboMaker Small Warehouse (`no_roof_small_warehouse.launch.py` 재사용). 탐지 대상 객체
  (fire_extinguisher/chair)는 `sim.launch.py`가 `ros_gz_sim create`로 스폰(인자 `spawn_objects`).
- **통합 기동: `semantic_nav_bringup/sim.launch.py`** — Gazebo+Scout + Nav2 + RViz를 묶고 Nav2는 지연 시작.
  - **Nav2 백엔드 `nav2` 인자** (기본 `bringup`): `bringup`=표준 nav2_bringup + 맵
    `semantic_nav_bringup/maps/test.yaml`(주행 확인됨, `map` 인자로 교체). `scout`=비권장(맵 절대경로 깨짐).
    `none`=Nav2 없이 sim+RViz.
  - odom이 ground_truth라 static `map→odom`(인자 `publish_map_odom_tf`, 기본 on)으로 TF 트리 완성.
  - **이 머신은 환경이 까다로워 직접 launch 대신 `./run_sim.sh` 사용.** 상세 `docs/TROUBLESHOOTING.md`.

## 워크스페이스 구조

```
ros2_ws/src/
├── semantic_nav_msgs/      # 커스텀 msg/srv 정의 (CMake, rosidl)
├── object_detector/        # 객체 탐지 + 2D→3D 투영
├── semantic_map/           # 시맨틱 객체 DB + RViz 마커
├── language_goal/          # 명령 파서 + 목표 포즈 생성 + Nav2 클라이언트
└── semantic_nav_bringup/   # launch 파일, 파라미터 yaml
```

## 빌드 / 실행 / 테스트

- 빌드: `colcon build --symlink-install` (워크스페이스 루트). 단일: `--packages-select <pkg>`.
- 환경: `source install/setup.bash`. 테스트: `colcon test --packages-select <pkg> && colcon test-result --verbose`.
- **시뮬 실행: `./run_sim.sh`** (환경 자동 설정 — 직접 launch 시 DDS/EGL/좀비로 안 뜸). 상세 `docs/TROUBLESHOOTING.md`.

## 카메라 (M2 완료 — M3 인식이 의존하는 불변 사실)

- RGB-D = Intel RealSense **D435i** (`realsense2_description`). `./run_sim.sh` 기동 시 토픽
  `/camera/color/image_raw`(rgb8 640×480) · `/camera/depth/image_raw` · `/camera/color/camera_info` 발행.
- 모든 카메라 메시지 frame_id = **`camera_color_optical_frame`**. depth는 color와 정렬(align)된
  동일 가상카메라 → 같은 픽셀·같은 프레임. **depth 인코딩 = `32FC1`(m)**, 그대로 미터로 사용.
- **camera_info(K) 신뢰 가능** — fx=fy≈462.3, cx=320, cy=240(이미지와 일치). FOV 재계산 불필요.
- 구현 상세(프레임 트리·intrinsics·객체 모델)는 `docs/NOTES.md`, 환경 이슈는 `docs/TROUBLESHOOTING.md`.

## 인식·시맨틱맵 (M3 완료 — M4가 의존하는 불변 사실)

- 토픽/서비스: `/semantic_nav/detections`(DetectedObject3DArray, **header 없음** — 각 객체가
  frame_id=map인 자기 header) · `/semantic_nav/find_object`(FindObject) ·
  `/semantic_nav/object_markers`(MarkerArray) · `/semantic_nav/debug_image`(디버그 오버레이).
- **FindObject는 배열 응답** `DetectedObject3D[] matches` (같은 라벨 다중 인스턴스, confidence 내림차순,
  미확정 제외). 단일 응답 아님 — M4 goal_commander는 matches를 순회/선택해야 함.
- **탐지기 = YOLOE**(open-vocab). 기본 `models/yoloe-11l-seg.pt`(gitignore), **GPU(torch cu128)
  ~33ms@imgsz1280, min_confidence 0.25, inference_rate 8Hz**. `target_classes`로 어휘 런타임 설정.
  실배포(Jetson Orin NX)는 `.engine`(TensorRT FP16, 어휘 export 시 고정) — SPEC 2.2.
  GPU torch는 cu128(2.11.0) — 설치 함정은 `docs/TROUBLESHOOTING.md` 8장.
- **confirmation**: `min_observations`(기본 3) 미만 관측 객체는 **미확정** → find_object/초록 마커 제외,
  회색 반투명 마커로만 표시. (conf 0.25로 낮춰 늘어난 단발 오탐의 자연 필터.)
- semantic_map은 **객체를 삭제하지 않는다**(정적 랜드마크 가정). 데이터 어소시에이션: 같은 라벨 &
  거리<`merge_distance`(0.5m)면 EMA 병합, 아니면 신규. 움직이는 객체 추종/추적(tracking)은 M4/M5 보류.

## 규칙

- 모든 노드는 파라미터를 declare_parameter로 선언하고 yaml로 관리한다. 하드코딩 금지.
- 좌표 변환은 반드시 tf2 lookup_transform 사용. 수동 행렬 계산 금지.
- 본체 기준 프레임은 `base_link`가 아니라 **`mobile_robot_base_link`** (TF lookup·costmap·목표 포즈 계산).
  Nav2 `robot_base_frame`도 이 프레임.
- **3D 투영 TF lookup은 이미지 header.frame_id(`camera_color_optical_frame`) 기준으로 한다.
  `camera_link`로 lookup 금지** (광학 좌표계 ≠ 본체 좌표계).
- 시뮬 토픽 이름 고정: 2D 스캔 **`/laser_scan`**, 오도메트리 **`/odometry`**, 3D 포인트클라우드 `/points`.
- 커스텀 인터페이스 변경 시 semantic_nav_msgs만 수정하고 의존 패키지 재빌드. 토픽/서비스 이름은 SPEC.md 표 준수.
- 카메라 센서는 기존 로봇 URDF를 직접 수정하지 말고 별도 `camera.xacro` 모듈로 include.
- 카메라 토픽은 `/camera/color/image_raw`, `/camera/depth/image_raw`, `/camera/color/camera_info`로 통일.
- 새 기능은 해당 마일스톤 범위만 구현. 무거운 추론(YOLO 등)은 콜백 안 직접 X → 별도 스레드/큐.
- 커밋 메시지는 영어, conventional commits (feat:, fix:, docs:).

## 자주 하는 실수 (하지 말 것)

- Gazebo Classic(libgazebo_ros_camera.so)과 gz sim(rgbd_camera + ros_gz_bridge) 카메라 플러그인 혼동.
- gz(Ignition) 토픽은 `ros_gz_bridge`(`config/ros2_gz_bridge_config.yaml`)에 등록해야 ROS에 보인다.
  누락 시 에러 없이 조용히 안 나옴(`ros2 topic list`에 없음).
- depth 인코딩(16UC1 mm vs 32FC1 m) 혼동 — 이 워크스페이스는 **32FC1(m)**.
- TF lookup 시 `rclpy.time.Time()`(latest) 대신 이미지 timestamp 사용.
- Nav2 액션 호출 전 `wait_for_server` 누락.
- **AMCL 등 localizer 사용 시 `publish_map_odom_tf:=false` 필수** (안 그러면 map→odom 이중 발행 충돌).
