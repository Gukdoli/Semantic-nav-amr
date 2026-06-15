# 시뮬레이션 기동 트러블슈팅 (M1)

`ros2 launch semantic_nav_bringup sim.launch.py` (또는 `rl1`)이 안 뜨던 문제를 잡은 기록.
증상은 "노드가 전부 죽음" → "흰/검은 화면" → "malloc 크래시"로 계속 바뀌었는데,
**서로 다른 6개 문제가 겹쳐** 있었다. 아래는 양파 껍질을 벗긴 순서.

> **결론(진짜 원인):** `savoury1/graphics` PPA가 `libassimp5`를 5.4로 올렸는데
> DART/ign-physics는 assimp 5.2로 빌드돼 있어 **ABI 불일치 → 메시 충돌 로드 시 힙 손상
> (`malloc(): invalid size`)**. 나머지(DDS·EGL·좀비·conda)는 그 위에 겹친 별개 잔가지.

---

## 최종 정상 기동 방법

```bash
bash ~/nav2_semantic_ws/run_sim.sh
```
`run_sim.sh`가 아래(1~5)를 자동 처리한다. conda가 켜져 있으면 먼저 `conda deactivate`.

---

## 문제별 원인과 해결

### 1. 모든 ROS2 노드가 즉사 — DDS 인터페이스
- **증상:** `rmw_create_node: failed to create domain` / `enp118s0: does not match an available interface`
- **원인:** `~/.bash_aliases`의 `CYCLONEDDS_URI`가 `enp118s0`(현재 down)에 고정.
- **해결:** 루프백으로 교체
  ```bash
  export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name='lo' multicast='true'/></Interfaces></General></Domain></CycloneDDS>"
  ```

### 2. 좀비 ign 충돌 — 검은 화면 / 서버 멈춤
- **증상:** `Found additional publishers on /clock ... tcp://...`, GUI가 "server may be busy"만 반복.
- **원인:** 크래시한 ign 프로세스가 좀비로 남아 ign-transport 버스에서 충돌.
- **해결:** 매 실행 전 정리 + 로컬 격리
  ```bash
  pkill -9 -f 'ign gazebo'; pkill -9 -f ign-gazebo; pkill -9 -f ruby
  export IGN_IP=127.0.0.1
  ```

### 3. 창이 안 뜸 — gui 인자
- **원인:** `gui:=false`는 `ign gazebo -s`(headless, 창 없음). 정상 동작.
- **해결:** 창이 필요하면 `gui:=false`를 빼고 실행 (기본 `gui:=true`).

### 4. 흰 화면 / 모델 없음 — 리소스 경로
- **원인:** 오버레이(`install/setup.bash`)를 안 소스해서 `IGN_GAZEBO_RESOURCE_PATH` 미설정
  → `model://...` 미해석.
- **해결:** `source ~/nav2_semantic_ws/install/setup.bash` 후 실행. (`echo $IGN_GAZEBO_RESOURCE_PATH` 확인)

### 5. EGL `failed to create dri2 screen` — 하이브리드 GPU
- **원인:** Intel+NVIDIA 노트북. ogre2의 EGL device platform이 Intel Mesa를 골라 dri2 실패.
  추가로 사용자 계정이 `render`/`video` 그룹에 없어 `/dev/dri/renderD128` 접근 불가.
- **해결:**
  ```bash
  sudo usermod -aG render,video $USER     # 후 재로그인
  export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json
  ```
  (NVIDIA EGL 벤더만 노출 → 정상인 NVIDIA 디바이스 사용. GLX 쓰는 RViz는 영향 없었음.)
- **헛다리:** `LIBGL_ALWAYS_SOFTWARE`/`__NV_PRIME_RENDER_OFFLOAD`는 오히려 악화. 쓰지 말 것.

### 6. ⭐ `malloc(): invalid size (unsorted)` — assimp ABI 불일치 (진짜 원인)
- **증상:** 빈 월드는 정상, **메시가 있는 월드**(웨어하우스/모델 1개)만 로드 중 크래시.
- **진단:** `-s`(서버 전용) + `libignition-gazebo6-dbg` + gdb 백트레이스 →
  `ignition::physics::dartsim::CustomMeshShape::CustomMeshShape(common::Mesh...)`
  → `AttachMeshShape` → `CreateCollisionEntities`. assimp `aiScene` 구조체 할당 중 힙 손상.
- **원인:** `libassimp5`가 **5.4.3(savoury1 PPA)**, 그러나 `libdart6.12`·
  `libignition-physics5-dartsim`은 **jammy 표준 assimp 5.2**로 빌드됨 → 구조체 레이아웃 불일치 →
  버퍼 오버플로 → 힙 손상.
- **해결:** assimp를 표준으로 다운그레이드 + 고정
  ```bash
  sudo apt install --allow-downgrades -y libassimp5=5.2.2~ds0-1 libassimp-dev=5.2.2~ds0-1
  sudo apt-mark hold libassimp5 libassimp-dev
  ```
- **확인:** `ign gazebo -s -v 4 -r ~/nav2_semantic_ws/test_one_model.world` → malloc 없이 통과.

---

## 디버깅 교훈

- 증상(노드 죽음/흰화면/malloc)이 계속 바뀌면 **여러 문제가 중첩**됐다고 의심.
  하나 고치면 다음 게 드러난다.
- "빈 입력 vs 실제 입력" 분리 테스트가 강력했다 (빈 월드 OK → 모델 로딩이 범인).
- 힙 손상(`malloc invalid size`)은 **탐지 지점 ≠ 손상 지점**. gdb `thread apply all bt` +
  디버그 심볼로 실제 손상 함수를 찾아야 한다.
- **그래픽/멀티미디어 백포트 PPA(savoury 등)는 robotics apt 스택의 ABI를 조용히 깰 수 있다.**
  의심 라이브러리는 `apt-cache policy <lib>`로 `.sav`/PPA 버전인지 확인.

---

## 7. M2 추가 — 카메라/객체 렌더링 (위 1·2·5·6과 같은 뿌리)

- **`rgbd_camera` 센서도 gpu_lidar처럼 EGL 렌더링**을 쓴다 → 카메라 영상이 나오려면 5번의
  NVIDIA EGL(`__EGL_VENDOR_LIBRARY_FILENAMES=.../10_nvidia.json`) + `render`/`video` 그룹 필요.
- **탐지 대상 객체/카메라를 추가하면 메시가 늘어 6번(assimp ABI) 노출이 커진다.** 객체·카메라가
  안 보이고 죽으면 assimp부터 의심(`libassimp5` 5.2.2 hold 유지). 회피책: collision은 프리미티브로.
- **D435i 몸체 메시가 안 보이면** `IGN_GAZEBO_RESOURCE_PATH`에 `/opt/ros/humble/share` 누락 의심
  (4번 리소스 경로의 연장). run_sim.sh가 설정함. 로그 `Unable to find file ... d435.dae` 확인.

---

## 8. M3 추가 — YOLOE/ultralytics 설치 함정 (개발머신, CPU)

> `pip install ultralytics` 한 줄이 **numpy를 2.x로 올려 ROS/시스템 파이썬 스택을 조용히
> 깨뜨리는 게 핵심**. 아래 8-1~8-4는 거의 항상 같이 터지므로 묶어서 본다.
> 전체 정상 설치 레시피는 `docs/NOTES.md` M3 섹션 참조.

### 8-1. ⭐ numpy 2.x로 올라가 ROS/시스템 패키지가 깨짐
- **증상:** ultralytics 설치 후 `import matplotlib`(또는 `cv_bridge`, 그 외 apt 파이썬 패키지)에서
  `ImportError: numpy.core.multiarray failed to import`. ROS 노드도 numpy 쓰는 곳에서 죽음.
- **원인:** ultralytics 의존성이 **numpy를 2.2.x로 업그레이드**. Ubuntu 22.04 시스템 패키지와
  ROS Humble(rclpy/cv_bridge)은 **numpy 1.x ABI로 빌드** → 2.x 런타임과 ABI 불일치.
  `--user` 설치라 `~/.local`의 numpy 2.x가 시스템 1.26.4를 가림.
- **해결:** numpy를 1.x로 되돌려 고정.
  ```bash
  pip install --user "numpy<2"        # -> 1.26.4
  ```

### 8-2. opencv-python이 numpy>=2를 요구
- **증상:** 8-1에서 numpy를 1.26으로 내리면
  `opencv-python 4.13.x requires numpy>=2 ... but you have numpy 1.26.4`.
  `import cv2`가 ABI 깨짐(2.x로 빌드된 휠을 1.x에서 로드).
- **원인:** 최신 `opencv-python`(4.11+)이 numpy 2.x ABI로 빌드됨. ultralytics는 cv2가 필요.
- **해결:** numpy 1.x 호환 버전으로 다운그레이드(ultralytics는 ≥4.6이면 OK).
  ```bash
  pip install --user "opencv-python==4.10.0.84"
  ```

### 8-3. `No module named 'clip'` — ultralytics 자동설치가 깨진 `UNKNOWN` 패키지를 남김
- **증상:** YOLOE `set_classes`/`get_text_pe` 호출 시
  `ModuleNotFoundError: No module named 'clip'`. `pip list`에 `UNKNOWN 0.0.0`만 있고 `clip` 없음.
- **원인:** YOLOE 텍스트 인코더가 `import clip` 필요 → ultralytics가
  `git+https://github.com/ultralytics/CLIP.git`를 자동설치하는데, 이 레포가 메타데이터 없이
  **`UNKNOWN-0.0.0`으로 빌드돼 `clip` 모듈을 안 깔아 줌**.
- **해결:** 깨진 패키지 제거 후 OpenAI CLIP(정상 `clip` 모듈 제공)을 수동 설치.
  ```bash
  pip uninstall -y UNKNOWN
  pip install --user ftfy regex tqdm "clip @ git+https://github.com/openai/CLIP.git"
  ```

### 8-4. 텍스트 인코더(mobileclip_blt.ts)가 매번 재다운로드 / 못 찾음
- **증상:** 노드 기동마다 `mobileclip_blt.ts`(~572MB)를 다시 받거나, cwd에 따라 위치가 달라짐.
- **원인:** YOLOE seg 모델의 텍스트 인코더는 **MobileCLIP**(`mobileclip_blt.ts`). ultralytics
  `attempt_download_asset`는 **cwd → `SETTINGS["weights_dir"]`(기본 상대경로 `weights`)** 순으로만
  찾음 → 노드 cwd가 불정해 매번 못 찾음.
- **해결:** 가중치를 gitignore된 `models/`에 두고, `weights_dir`을 그 절대경로로 고정.
  `YoloeDetector`가 생성 시 `model_path`의 디렉터리로 `SETTINGS["weights_dir"]`을 자동 설정하므로
  **`yoloe-11s-seg.pt`와 `mobileclip_blt.ts`를 같은 폴더에 두기만 하면** cwd 무관하게 resolve됨.

### 8-5. GPU를 쓰려는데 torch가 CPU 빌드 / 교체가 안 됨
- **증상:** `torch.cuda.is_available()`가 `False`. `+cu128`로 재설치해도 `2.12.0+cpu` 그대로.
- **원인:** ① 처음에 CPU 인덱스(`whl/cpu`)로 깔아 `2.12.0+cpu`. ② pip는 **로컬라벨(+cpu/+cu128)을
  무시**하고 버전(2.12.0)이 같으면 "이미 충족"으로 교체 안 함. ③ **cu128 인덱스엔 2.12.0이 없음**
  (`No matching distribution`, 최대 2.11.0+cu128).
- **해결:** 먼저 제거 후, cu 인덱스에 **존재하는 버전**으로 설치.
  ```bash
  pip uninstall -y torch torchvision
  pip install --user torch==2.11.0 torchvision==0.26.0 --index-url https://download.pytorch.org/whl/cu128
  ```
  cu 휠은 CUDA 런타임 내장(시스템 CUDA 툴킷 불필요, 드라이버만). `device:""`(자동)면 cuda:0 사용.
  노드 로그 `YOLOE detector loaded (device=cuda:0)` 확인. numpy가 다시 2.x로 안 올라갔는지도 점검.

### 검증
```bash
python3 -c "import numpy,cv2,torch,clip; from ultralytics import YOLOE; print('ok', torch.cuda.is_available())"
# assets/bus.jpg + 어휘 ["person","bus"] 추론 → person/bus 탐지(GPU면 ~6ms)면 정상.
```

---

## 디버깅 교훈 (M3 추가)

- **`pip install <ML패키지>`는 robotics apt/ROS 스택의 numpy ABI를 조용히 깬다.** PPA가 시스템
  C++ ABI를 깨는 것(6번 assimp)과 같은 패턴의 파이썬 버전. 설치 후 `import cv_bridge` /
  `import matplotlib`로 회귀 확인하고, numpy는 `"numpy<2"`로 고정.
- 모델 가중치·텍스트 인코더 같은 **대용량 런타임 자산은 cwd 의존 경로에 받지 말고** 명시적 절대경로
  (`models/` + `weights_dir`)로 고정 — 노드는 cwd가 불정하다.

---

## 관련 파일
- `run_sim.sh` — 위 1·2·5·7 환경을 자동 설정해 기동하는 스크립트.
- `test_one_model.world` — malloc 원인 분리용 단일 모델 월드.
- `docs/NOTES.md` (M3) — YOLOE 정상 설치 레시피 전체.
