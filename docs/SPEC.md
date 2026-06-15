# SPEC — Semantic Navigation AMR

자연어/객체 기반 목표 지정이 가능한 AMR. 인식 파이프라인이 백그라운드에서 객체를 맵 좌표로 누적하고,
명령 파이프라인이 사용자 명령을 파싱해 Nav2 목표로 변환한다.

## 1. 시스템 구성

```
[인식 파이프라인 - 상시 동작]
RGB-D 카메라 → 객체 탐지 노드(YOLO-World) → 3D 위치 추정(depth+TF) → 시맨틱 객체 DB

[명령 파이프라인 - 명령 입력 시]
자연어 명령 → 명령 파서 → 목표 포즈 생성(DB 조회 + 오프셋) → Nav2 NavigateToPose
```

## 2. 패키지별 명세

### 2.0 카메라 센서 통합 (선행 작업 — 현재 로봇에 비전 없음)

기존 AMR URDF/xacro에 시뮬레이션 RGB-D 카메라를 추가한다.

> **확인된 환경 (코드 분석):** 이 워크스페이스는 **Ignition Gazebo 6 (Fortress) / gz sim** +
> AgileX **Scout v2** 로봇이다. 아래 "Gazebo 버전별 분기"는 **gz sim 경로만 적용**한다
> (Classic 경로는 무시). 로봇 URDF 진입점은 `agilex_scout/urdf/robot.urdf.xacro`이며 그 안에서
> `mobile_robot/scout_v2.urdf.xacro`(본체) → `scout_v2.gazebo`(센서/플러그인)를 include한다.
> 새 `camera.xacro`는 `robot.urdf.xacro`에 include 한 줄을 추가하는 방식으로 연결한다.

- `camera.xacro` 모듈 신규 작성 (기존 로봇 파일은 include 한 줄만 추가)
  - camera_link + 고정 joint. **부모 링크는 `front_mount`로 확정** (전방 상단 마운트,
    mobile_robot_base_link 기준 xyz=0.325 0 0.1127). 카메라 자체 오프셋/방향은 파라미터화.
    (이 워크스페이스에는 표준 `base_link`가 0-offset 더미로만 존재하고 실제 본체 프레임은
    `mobile_robot_base_link`이므로 base_link에 직접 붙이지 않는다.)
  - 광학 프레임 포함 (REP-103/105 광학 좌표 규약 준수). **구현은 Intel RealSense D435i**
    (`realsense2_description`)를 채택 → 광학 프레임은 `camera_color_optical_frame` /
    `camera_depth_optical_frame`(D435i 표준). 실하드웨어와 프레임 동일.
- Gazebo 센서 (gz sim / Fortress 전용 — Classic 분기 없음):
  - `<sensor type="rgbd_camera">`(또는 depth_camera) 를 camera_link에 정의. 렌더링은 이미
    `scout_v2.gazebo`에 로드된 `ignition-gazebo-sensors-system` 플러그인이 처리.
  - gz → ROS 토픽 노출은 **`ros_gz_bridge` 등록 필수**: `agilex_scout/config/ros2_gz_bridge_config.yaml`에
    color image / depth image / camera_info 항목을 기존 패턴대로 추가 (브릿지 누락 시 토픽이
    조용히 안 나옴).
- 최종 토픽 (remap으로 통일):
  - `/camera/color/image_raw` (sensor_msgs/Image)
  - `/camera/depth/image_raw` (sensor_msgs/Image)
  - `/camera/color/camera_info` (sensor_msgs/CameraInfo)
- 검증 기준: RViz에서 컬러/depth 이미지 표시, `ros2 run tf2_tools view_frames`에서
  map → ... → camera_link 체인 연결 확인, depth 인코딩 확인 후 CLAUDE.md에 기록

### 2.1 semantic_nav_msgs

커스텀 인터페이스 정의 전용 패키지.

- `msg/DetectedObject3D.msg`
  ```
  std_msgs/Header header        # frame_id: map
  string label
  float32 confidence
  geometry_msgs/Point position  # map 좌표
  ```
- `msg/DetectedObject3DArray.msg`: `DetectedObject3D[] objects`
- `srv/FindObject.srv` (라벨로 조회, **확정된 모든 인스턴스를 배열로 반환** — 같은 라벨
  다중 객체 대응. 비면 not found. 각 match는 DetectedObject3D로 위치/confidence/라벨/
  last_seen(header.stamp)을 담고 confidence 내림차순 정렬.)
  ```
  string label
  ---
  DetectedObject3D[] matches
  ```
- `srv/NavigateToObject.srv`
  ```
  string command            # 원문 자연어 명령
  ---
  bool accepted
  string message            # 실패 사유 등
  ```

### 2.2 object_detector

- 노드: `object_detector_node`
- 구독: `/camera/color/image_raw`, `/camera/depth/image_raw`, `/camera/color/camera_info`
  - color(~14Hz)와 depth(~5Hz)는 **주기가 다르다** → `message_filters.ApproximateTimeSynchronizer`로
    동기화(정확 stamp 매칭인 `TimeSynchronizer`는 거의 안 맞음). `slop` 파라미터(기본 0.1s)로 허용 오차 지정.
    camera_info는 동기화 대상 아님 — 최신값 캐시해서 사용(intrinsic은 거의 불변).
- 발행: `/semantic_nav/detections` (DetectedObject3DArray)
- 처리 흐름:
  1. YOLOE로 open-vocabulary 탐지. **`ultralytics`의 YOLOE 사용**(YOLO-World 후속, 동일 ultralytics API).
     small 변형 **`yoloe-11s-seg.pt`**(YOLO11 기반 text/visual-prompt 모델; prompt-free 변형은
     `yoloe-11s-seg-pf.pt`)를 채택. `set_classes(target_classes)`로 탐지 어휘를 런타임 설정
     (YOLOE는 텍스트 임베딩을 함께 넘기는 `set_classes(names, get_text_pe(names))` 형태 —
     detector 래퍼가 흡수). 모델 가중치는 `model_path` 파라미터로 관리(파일 있으면 로드, 없으면 최초
     1회만 다운로드 — 콜백/추론 루프 안에서 받지 않는다).
  2. 바운딩 박스 중심부 depth **중앙값**으로 카메라 좌표 3D 점 계산 (camera_info의 intrinsic 사용).
     **중앙값 계산 전 NaN/0 픽셀 제외** — depth는 32FC1이라 측정 실패가 `NaN`(0도 무효로 취급).
     유효 픽셀이 없으면 해당 탐지는 스킵.
  3. tf2로 camera frame → map 변환 (이미지 timestamp 기준). **lookup의 source 프레임은 임의 고정값이 아니라
     이미지 메시지의 `header.frame_id`(= `camera_color_optical_frame`)를 그대로 쓴다** (`camera_link` 금지).
  4. confidence < `min_confidence` (기본 0.5) 필터링
- 파라미터: `target_classes`, `model_path`, `min_confidence`(기본 0.5), `slop`(기본 0.1),
  `inference_rate_hz` (기본 5)
- **실배포 노트 (YOLOE 배포):** 타겟 보드는 **NVIDIA Jetson Orin NX**. 실배포는
  **TensorRT FP16 엔진**으로 한다. **TensorRT export 시 텍스트 어휘가 엔진에 고정**되므로
  (offline vocabulary), `target_classes`를 바꾸면 **엔진을 재export** 해야 한다. 개발머신(CPU)은
  PyTorch 추론을 그대로 쓰며 **코드는 동일**(model_path만 `.pt`↔`.engine`로 교체). 따라서
  런타임 `set_classes`는 PyTorch 경로에서만 유효하고, TensorRT 엔진에서는 export 시점 어휘가 박힌다.

### 2.3 semantic_map

- 노드: `semantic_map_node`
- 구독: `/semantic_nav/detections`
- 서비스 제공: `/semantic_nav/find_object` (FindObject)
- 발행: `/semantic_nav/object_markers` (visualization_msgs/MarkerArray, RViz용)
- 데이터 어소시에이션: 같은 label이고 거리 < `merge_distance` (기본 0.5m)면 기존 항목에 병합
  (위치는 지수이동평균, last_seen 갱신, 관측 count 증가). 아니면 새 항목 추가.
- 확정(confirmation): `min_observations`(기본 3)회 미만 관측된 객체는 **미확정**으로 보아
  find_object 응답과 (초록) 마커에서 제외하고, 디버깅용 **회색 반투명 마커**로 따로 표시한다.
  recall 향상을 위해 detector `min_confidence`를 낮추면(0.25) 단발성 오탐 유입이 늘어나는데,
  스친 오탐은 관측 count를 못 채우므로 confirmation이 자연 필터가 된다.
- 저장: 1차는 in-memory dict. 마일스톤 4에서 종료 시 JSON 저장/로드 추가.
  객체는 한 번 들어오면 **삭제하지 않는다**(정적 랜드마크 가정).

### 2.4 language_goal

- 노드: `goal_commander_node`
- 서비스 제공: `/semantic_nav/navigate_to_object` (NavigateToObject)
- 처리 흐름:
  1. 명령 파싱 → {target_label, relation}. 1차는 키워드 매칭, 마일스톤 4에서 LLM 파서로 교체
  2. find_object 서비스로 객체 위치 조회
  3. 목표 포즈 계산: 객체에서 로봇 방향으로 `approach_distance` (기본 0.7m) 떨어진 점,
     방향(yaw)은 객체를 바라보도록 설정
  4. 글로벌 코스트맵 조회해서 목표 지점 점유 여부 확인, 점유 시 객체 주변 8방향 후보 탐색
  5. Nav2 NavigateToPose 액션 호출, 결과를 서비스 응답으로 반환

### 2.5 semantic_nav_bringup

- `sim.launch.py`: Gazebo(gz sim, AWS warehouse 월드) + AgileX Scout v2 + Nav2 + 위 노드 전체.
  기존 `agilex_scout/launch/simulate_control_gazebo.launch.py`(로봇+Gazebo)와
  `scout_nav2/launch/nav2.launch.py`(Nav2)를 include로 재사용해 통합.
- `params/` 아래 노드별 yaml

## 3. 마일스톤

- [x] M1: 기존 AMR 패키지 + Nav2 기동 확인. RViz에서 2D Goal Pose로 주행 확인.
      `agilex_scout/launch/simulate_control_gazebo.launch.py`(Gazebo+Scout)와
      `scout_nav2/launch/nav2.launch.py`(Nav2)를 include로 묶는 통합
      `semantic_nav_bringup/sim.launch.py`를 새로 작성하고, 이 한 줄로 전체 기동 + 주행 확인.
      (기존 launch는 재사용만, 내부 수정 최소화.)
- [x] M2: RGB-D 카메라 통합 (섹션 2.0). 완료/검증됨:
      - `agilex_scout/urdf/sensors/camera.xacro` 신규(`<xacro:camera parent="front_mount">`,
        `robot.urdf.xacro`에 include 한 줄). **Intel RealSense D435i**(`realsense2_description`의
        `sensor_d435i`) 메시+프레임 트리 + gz `rgbd_camera` 센서(camera_link에 부착). 실하드웨어와
        프레임/토픽 동일. 이미지 frame_id = `camera_color_optical_frame`(REP-103/105).
      - 토픽 노출 확인: `/camera/color/image_raw`(rgb8 640×480, ~14Hz),
        `/camera/depth/image_raw`(**32FC1, m**, ~5Hz), `/camera/color/camera_info`(~14Hz).
        ros_gz_bridge에 3항목 추가(`agilex_scout/config/ros2_gz_bridge_config.yaml`).
      - TF 체인 확인: `... → mobile_robot_base_link → front_mount → camera_bottom_screw_frame →
        camera_link → camera_color_optical_frame`(+ depth/infra1·2/accel/gyro 프레임).
        D435i 메시 렌더링엔 `IGN_GAZEBO_RESOURCE_PATH`에 `/opt/ros/humble/share` 필요(run_sim.sh에 추가됨).
      - camera_info 보정: gz Fortress가 기본 320×240/60° K(주점 1/4 지점)를 내보내는 문제 →
        `<lens><intrinsics>`로 명시 → K = fx=fy=462.3, cx=320, cy=240 (이미지와 일치).
      - 탐지 대상 객체 배치: `semantic_nav_bringup/models/{fire_extinguisher,chair}`
        (Fuel 메시 visual + **프리미티브 collision** = assimp 충돌-메시 크래시 회피),
        `sim.launch.py`에서 `ros_gz_sim create`로 스폰(인자 `spawn_objects`, 기본 on).
        gz 월드에 fire_extinguisher/chair 로드 확인, 크래시 없음.
- [x] M3: object_detector + semantic_map 구현. RViz MarkerArray로
      맵 위에 객체 라벨이 찍히는 것 확인. 완료 사항:
      - 인식 파이프라인 동작: YOLOE(`yoloe-11l-seg.pt`, GPU cu128 ~33ms@imgsz1280,
        conf 0.25) → depth+TF로 map 좌표 투영 → semantic_map 누적 → RViz 초록 마커.
      - confirmation(`min_observations`=3): 미확정 객체는 회색 반투명 마커 + find_object 제외.
      - FindObject는 **배열 응답**(같은 라벨 다중 인스턴스). 인스턴스 분리 검증 통과
        (소화기 2개 >3m → 별개 항목 2 + 마커 2; sim.launch.py가 2개 스폰).
      - 디버그용 `/semantic_nav/debug_image`(탐지 박스+상태색). 단위 14개 + ROS 스모크 통과.
- [ ] M4: language_goal 구현 (키워드 파서). "go to fire extinguisher" 명령으로
      실제 주행 성공. ros2 service call로 데모.
- [ ] M5: 고도화 — LLM 파서 교체, 공간 관계(near/behind/between) 처리,
      DB 영속화(JSON), 데모 GIF 촬영 + README 정리.
      카메라-LiDAR 융합(`/points` 3D LiDAR 활용)으로 객체 거리 정확도 개선 (확장 아이템).

## 4. 완료 기준 (M4 시점)

- `ros2 launch semantic_nav_bringup sim.launch.py` 한 번으로 전체 기동
- `ros2 service call /semantic_nav/navigate_to_object ...` 호출 시
  로봇이 해당 객체 0.5~1.0m 이내 도달 후 객체를 바라보고 정지
- DB에 없는 객체 요청 시 accepted=false와 명확한 메시지 반환
- colcon test 통과 (파서 단위 테스트, 3D 투영 수학 단위 테스트 포함)
