# 구현 노트 (NOTES)

세션마다 알 필요는 없지만 보존 가치가 있는 구현 상세. 불변 요약은 `CLAUDE.md`,
환경/기동 트러블슈팅은 `docs/TROUBLESHOOTING.md`.

## M2 — RGB-D 카메라 (Intel RealSense D435i)

- `agilex_scout/urdf/sensors/camera.xacro`가 `<xacro:sensor_d435i parent="front_mount" name="camera"
  use_nominal_extrinsics="true">`(realsense2_description)로 실제 D435i 메시+프레임 트리를 넣고,
  그 위에 gz `rgbd_camera` 센서를 `camera_link`에 붙여 영상을 만든다. 실하드웨어에선 realsense2_camera
  노드가 같은 토픽/프레임으로 영상을 제공 → 시뮬↔실물 동일. `agilex_scout/package.xml`에 exec_depend,
  apt `ros-humble-realsense2-description` 설치 필요.
- 프레임 트리: `front_mount → camera_bottom_screw_frame → camera_link →
  camera_{color,depth}_optical_frame / camera_infra1·2_* / camera_accel·gyro_*`.
  카메라 영상은 모두 `camera_color_optical_frame`으로 스탬프(color/depth 동일 가상카메라라 정렬됨).
- **camera_info 보정:** gz Fortress `rgbd_camera`는 `<lens><intrinsics>`를 안 주면 이미지 해상도와
  안 맞는 기본 K(320×240/60°, 주점이 1/4 지점)를 발행한다. camera.xacro에 명시 intrinsics를 넣어
  K = fx=fy≈462.3, cx=320, cy=240(이미지와 일치)으로 고정. hfov/해상도 바꾸면 자동 갱신(xacro 식).
- **D435i 메시(d435.dae) 렌더링:** `IGN_GAZEBO_RESOURCE_PATH`에 `/opt/ros/humble/share`가 있어야
  `package://realsense2_description/...` → `model://...`가 해석된다(run_sim.sh에 추가됨). 누락 시
  `Unable to find file ... d435.dae`로 카메라 몸체만 안 보임(센서/토픽/프레임은 정상).
  collision은 box 프리미티브라 assimp 충돌 위험 없음(visual d435.dae 15MB는 visual-only).

## M3 — object_detector + semantic_map (인식 파이프라인)

- 패키지 3종 신규(`src/scout_nav2/` 아래): `semantic_nav_msgs`(ament_cmake/rosidl),
  `object_detector`·`semantic_map`(ament_python). 인터페이스는 M3 범위 3종만
  (`msg/DetectedObject3D`, `msg/DetectedObject3DArray`, `srv/FindObject`).
  `NavigateToObject.srv`는 M4에서 추가. **주의: `DetectedObject3DArray`는 header 없음**
  (각 `DetectedObject3D`가 자기 header를 가짐, frame_id=map).
- 신규 토픽/서비스:
  - `/semantic_nav/detections` (`DetectedObject3DArray`) — object_detector 발행.
  - `/semantic_nav/find_object` (`FindObject`) — semantic_map 제공.
  - `/semantic_nav/object_markers` (`MarkerArray`) — semantic_map 발행(객체당
    SPHERE `ns=objects` + TEXT_VIEW_FACING `ns=labels`, 매 주기 DELETEALL 후 재발행).
- **object_detector 구조(torch 격리):** `projection.py`(순수 역투영 수학),
  `detector.py`(`YoloeDetector`가 ultralytics를 **지연 import**), `object_detector_node.py`
  (글루). color+depth는 `ApproximateTimeSynchronizer(slop)`로 동기화, camera_info는
  최신 K 캐시. **무거운 추론은 콜백 밖** — 콜백은 최신 프레임을 단일 슬롯에 저장,
  워커 스레드가 `inference_rate_hz`로 detect→투영→TF→publish. TF lookup source는
  이미지 `header.frame_id`(`camera_color_optical_frame`), 이미지 stamp 기준.
- **탐지 모델 = YOLOE**(ultralytics, YOLO-World 후속). small 변형 `yoloe-11s-seg.pt`.
  어휘 설정은 YOLOE 형식 `set_classes(names, get_text_pe(names))` — `YoloeDetector._set_classes`가
  2-arg 우선, `TypeError` 시 1-arg fallback. **`.engine`(TensorRT)이면 set_classes 스킵**
  (export 시 어휘 고정 — SPEC 2.2 실배포 노트). 실배포 타겟은 Jetson Orin NX + TensorRT FP16.
- **개발머신(CPU) 설치 사실:**
  - **GPU(RTX 4090 Laptop, 드라이버 580/CUDA13):**
    `pip install --user torch==2.11.0 torchvision==0.26.0 --index-url https://download.pytorch.org/whl/cu128`
    → `torch 2.11.0+cu128`, `cuda True`, 추론 ~6ms. cu 휠이 자체 CUDA 런타임을 내장하므로
    시스템 CUDA 툴킷 불필요(드라이버만). **함정:** ① cpu로 깔린 `torch==2.12.0`이 있으면 pip가
    로컬라벨(+cpu vs +cu128)을 무시하고 "이미 충족"으로 **교체 안 함** → 먼저
    `pip uninstall -y torch torchvision`. ② **cu128 인덱스엔 torch 2.12.0이 없음**(최대 2.11.0+cu128)
    → 버전을 2.11.0/torchvision 0.26.0으로 맞춤. 디바이스는 `device` 파라미터(`""`=자동 cuda:0).
  - (CPU 전용 대안) `pip install --user torch torchvision --index-url https://download.pytorch.org/whl/cpu`
    → torch 2.12.0+cpu (느림, 데모 비권장).
  - `pip install --user ultralytics` (8.4.66). **주의: numpy를 2.x로 올려 Ubuntu 22.04 시스템
    패키지(matplotlib 등)·ROS(cv_bridge/rclpy, numpy 1.x ABI)를 깨뜨림** → `pip install --user
    "numpy<2"`(1.26.4)로 되돌리고, numpy2 요구하는 `opencv-python`은 `==4.10.0.84`로 다운그레이드.
  - YOLOE 텍스트 인코더용 `clip` 모듈 필요. ultralytics 자동설치는 `UNKNOWN` 깨진 패키지를 남기므로
    수동: `pip install --user ftfy regex tqdm "clip @ git+https://github.com/openai/CLIP.git"`.
  - 가중치는 gitignore된 `models/`에: `yoloe-11l-seg.pt`(68M, **기본**) / `yoloe-11s-seg.pt`(27M) +
    텍스트 인코더 `mobileclip_blt.ts`(572M, 최초 추론 시 자동 다운로드). **ultralytics SETTINGS
    `weights_dir`를 절대경로 `/home/user/nav2_semantic_ws/models`로 설정**(`~/.config/Ultralytics/
    settings.json`에 영속) → 노드 cwd 무관하게 `mobileclip_blt.ts` 재다운로드 없이 resolve.
  - 검증: ultralytics `assets/bus.jpg`에 `["person","bus",...]` 어휘로 추론 → person×5+bus×1 탐지 OK.
- **ultralytics/torch 미설치 상태에서도:** 노드는 정상 기동(import OK)하고 detect 시점에
  설치 안내 로그만 남김 → `/semantic_nav/detections`는 빔. 단위 테스트(투영/DB)와 빌드는 torch 불필요.
- semantic_map 데이터 어소시에이션: 같은 label & 거리<`merge_distance`(0.5)면 EMA
  병합(`ema_alpha`, count++), 아니면 신규. `find(label, min_count)`는 해당 라벨 중 count>=min_count인
  **모든 인스턴스를 confidence 내림차순 리스트로 반환**(FindObject 배열 응답). JSON 영속화·객체 삭제는
  없음(정적 가정, M4).
- RViz: `semantic_nav_bringup/rviz/semantic_nav.rviz`(nav2.rviz 복제 + Image
  `/camera/color/image_raw`·`/camera/depth/image_raw` + MarkerArray
  `/semantic_nav/object_markers` + DetectionDebug `/semantic_nav/debug_image`).
  `sim.launch.py`가 이 config 사용 + `perception` 인자(기본 on)로 두 노드를 nav2 이후 기동.
- params: `semantic_nav_bringup/params/{object_detector,semantic_map}.yaml`.

### M3 후반 결정/튜닝 (사유 기록)

- **recall 튜닝(소화기가 거의 안 잡히던 문제):** 모델 `yoloe-11s` → **`yoloe-11l-seg.pt`**,
  `imgsz` 640 → **1280**, `min_confidence` 0.5 → **0.25**, `inference_rate_hz` 5 → 8.
  사유: 시뮬 렌더는 open-vocab confidence가 낮고 소화기가 작아 small 모델+640+0.5에선 놓침.
  GPU(33ms@1280)라 큰 모델/고해상도 비용이 무의미. **부작용**: 단발 오탐 유입 ↑ → 아래 confirmation으로 상쇄.
- **프롬프트 엔지니어링(소화기 미검출 + 의자 오탐 해결):** YOLOE는 텍스트 프롬프트 단어에
  매우 민감. 실측(`/tmp/cam_raw.png` 프레임, conf 무관 점수):
  소화기 = `"fire extinguisher"` **0.017**(놓침) → `"red fire extinguisher cylinder"` 0.345
  → **`"red metal cylinder"` 0.855**(시뮬 메시가 그냥 빨간 원통이라 외형 묘사가 의미어보다 잘 맞음).
  의자 오탐 = `"chair"`는 노란 창고 선반/구조물을 **0.313**으로 오인식하지만 `"office chair"`는
  **0.015**(시뮬 의자 모델이 `OfficeChairBlack`이라 정확). →
  **탐지 프롬프트와 저장 라벨 분리**: `detection_prompts`(`["red metal cylinder","office chair"]`)를
  YOLOE에 먹이고, cls_id를 `target_classes`(`["fire extinguisher","chair"]`)로 되매핑해 저장.
  `YoloeDetector(prompts=...)`, `_canonical_label`이 처리. **교훈: 시뮬/도메인 객체는 라벨 의미어가
  아니라 "모델이 보는 외형"으로 프롬프트를 잡아야 한다** — 새 클래스 추가 시 conf 무관 점수부터 찍어볼 것.
- **M3는 소화기 단일 클래스로 확정(중요 발견):** 여러 후보를 실제 sim 프레임에 실측한 결과
  소화기(진한 빨강 디테일 메시)만 잘 잡힘 — 점수: 소화기 0.855, 의자(메시) 0.027,
  파란 공·화분(**프리미티브**) 0.000, 트래픽 콘(텍스처 메시지만 창백 렌더) 0.008.
  - **프리미티브(평면 단색) 객체는 YOLOE에 사실상 안 보인다(0.00).** 실사 학습 모델이라
    텍스처/명암이 없는 CG 단색 면을 인식 못 함 → 탐지 대상은 **반드시 텍스처 있는 실제 메시**.
  - 그래도 시뮬 렌더가 **과노출/저대비**라 색이 날아가, 진한 빨강(소화기) 정도만 도메인 갭을 넘김.
    COCO yolo11l(chair 학습 클래스)조차 이 의자를 <0.05로 놓침 → 객체 교체로는 해결 안 됨.
  - 결정: **M3/M4 데모는 소화기 단일 클래스 + 다중 인스턴스(소화기 2개)로 마무리.** 파이프라인
    (투영·DB·find_object 배열·주행)은 클래스 수와 무관하게 완성이라 데모 충분. 2번째 클래스는
    M5에서 (a)시뮬 조명/노출 개선 또는 (b)YOLOE 비주얼 프롬프트(예시 크롭, 의자 0.57이지만 오탐) 도입 시 추가.
  - `target_classes`/`detection_prompts`를 소화기만 남김. 프리미티브 후보 모델(blue_ball/potted_plant)·
    traffic_cone은 제거. chair 모델 파일은 M2 산물로 보존(스폰/탐지에선 빠짐).
- **confirmation count(`min_observations`=3) 추가 사유:** conf를 0.25로 낮춰 오탐이 맵에 영구로
  남을 위험(객체 삭제 안 함)을 상쇄. N번 미만 관측은 미확정 → find_object/초록 마커 제외, 회색 반투명
  마커로만 표시(디버깅 `라벨? (count/N)`). 스친 오탐은 count를 못 채우므로 자연 필터.
- **debug 박스 깜빡임 / tracking 보류 결정:** 프레임별 독립 추론이라 임계값 근처 객체는 박스가
  ON/OFF 깜빡임. 이는 **debug_image(시각화)만의 현상이고 시맨틱 맵은 누적·영속이라 안정적**.
  ByteTrack 등 tracking은 (1)워커가 프레임을 드롭하고 카메라가 움직여 이득이 부분적, (2)상태 복잡도 추가,
  (3)M3 목표(정적 객체 맵)엔 불필요 → **M4/M5로 보류**(주행 시 특정 객체 끊김없는 조준이 필요해질 때).

## M2 — 탐지 대상 객체

- `semantic_nav_bringup/models/{fire_extinguisher,chair}` — **visual=Fuel 메시, collision=프리미티브**
  (cylinder/box). collision 메시를 안 써서 assimp 크래시 위험 없음. M3에서 객체 추가 시 같은 패턴 유지.
- `sim.launch.py`가 `ros_gz_sim create`로 스폰(인자 `spawn_objects`, 기본 on). 재사용 중인 AWS 월드
  SDF는 손대지 않는다.
