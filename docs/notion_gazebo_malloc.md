# 🐛 Gazebo `malloc(): invalid size` 크래시 잡은 기록

> **TL;DR**
> Ignition Gazebo가 메시 충돌이 있는 월드만 로드하면 `malloc(): invalid size (unsorted)`로 죽었다.
> 진짜 원인은 **`savoury1/graphics` PPA가 `libassimp5`를 5.4로 올렸는데, DART/ign-physics는
> assimp 5.2로 빌드돼 있어 ABI가 깨진 것**. → assimp를 5.2.2로 다운그레이드 + hold 하면 끝.
> GPU·DDS·conda 문제처럼 보였지만 전부 헛다리였고, **gdb 백트레이스**가 범인을 찍어줬다.

---

## 🧩 환경

| 항목 | 값 |
| --- | --- |
| 머신 | Lenovo Legion Pro 7 (Intel iGPU + NVIDIA dGPU 하이브리드) |
| OS / ROS | Ubuntu 22.04 / ROS 2 Humble |
| 시뮬 | Ignition Gazebo 6 (Fortress), 물리엔진 DART |
| 월드 | AWS RoboMaker Small Warehouse |

---

## 😱 증상

```
[ign gazebo-1] malloc(): invalid size (unsorted)
```

- **빈 월드(`ign gazebo`)는 멀쩡** → 격자 바닥 정상.
- **모델(메시)이 있는 월드**(웨어하우스, 또는 모델 1개짜리)만 로드 중 크래시.
- 서버가 죽으니 GUI는 흰/검은 화면으로 멈춤, `/clock`도 안 나옴.
- conda 제거, 환경변수 초기화(`env -i`), 패키지 재설치… 다 해도 **똑같이 재현**.

---

## 🔍 결정타: gdb 백트레이스

`malloc invalid size`는 **힙 손상**이라 "죽은 자리 ≠ 깨뜨린 자리". 그래서 백트레이스가 필요.

```bash
# 서버 전용(-s)이라 GUI/렌더 없이 순수 malloc만 재현됨
sudo apt install -y gdb libignition-gazebo6-dbg
gdb -batch -ex run -ex 'bt' -ex 'thread apply all bt' \
  --args "$(which ruby)" /usr/bin/ign gazebo -s -v 4 -r ~/nav2_semantic_ws/test_one_model.world
```

핵심 스택:

```
#9  operator new(unsigned long)
#10 ignition::physics::dartsim::CustomMeshShape::CustomMeshShape(common::Mesh const&, ...)
#11 ignition::physics::dartsim::ShapeFeatures::AttachMeshShape(...)
#18 PhysicsPrivate::CreateCollisionEntities (Physics.cc:1177)
#20 Physics::Update
```

→ **DART가 충돌 메시를 assimp `aiScene` 구조체로 변환하는 `CustomMeshShape` 생성자**에서 힙이 깨짐.

---

## 🎯 진짜 원인: assimp ABI 불일치

```bash
apt-cache policy libassimp5
```

```
libassimp5:
  설치: 5.4.3+ds-2~22.04.sav0      ← savoury1/graphics PPA
  표준: 5.2.2~ds0-1                ← jammy universe
```

- `libdart6.12`, `libignition-physics5-dartsim`은 **jammy 표준 assimp 5.2에 맞춰 빌드**됨.
- 그런데 런타임 `libassimp5`는 PPA가 올린 **5.4** → `aiScene`/`aiMesh` **구조체 레이아웃이 다름(ABI 깨짐)**.
- dartsim이 5.2 기준으로 구조체를 채우는데 실제 라이브러리는 5.4 → **버퍼 오버플로 → 힙 손상 → malloc 크래시.**
- 그래서 "예전엔 됐는데" → 언젠가 그래픽 백포트 PPA를 추가하며 assimp가 슬쩍 올라간 게 화근.

> 💡 `libassimp5`를 단순 `reinstall`해도 안 고쳐졌던 이유 = **버전이 그대로 5.4**였기 때문.

---

## ✅ 해결

```bash
# 1) assimp를 jammy 표준으로 다운그레이드
sudo apt install --allow-downgrades -y \
  libassimp5=5.2.2~ds0-1 libassimp-dev=5.2.2~ds0-1

# 2) PPA가 다시 올리지 못하게 고정
sudo apt-mark hold libassimp5 libassimp-dev
```

확인:

```bash
ign gazebo -s -v 4 -r ~/nav2_semantic_ws/test_one_model.world
# → malloc 없이 "Creating postupdate worker thread" 통과하면 성공 🎉
```

---

## 🧠 배운 것

- 증상(노드 죽음 → 흰화면 → malloc)이 **계속 바뀌면 여러 문제가 중첩**된 것. 하나 까면 다음이 드러난다.
- **"빈 입력 vs 실제 입력" 분리 테스트**가 강력했다 → 빈 월드 OK = "모델/메시 로딩"이 범인이라고 즉시 좁힘.
- 힙 손상은 **gdb `thread apply all bt` + 디버그 심볼**로 실제 깨뜨린 함수를 찾아야 한다.
- ⚠️ **그래픽/멀티미디어 백포트 PPA(savoury 등)는 apt로 깐 robotics 스택의 ABI를 조용히 깰 수 있다.**
  의심되면 `apt-cache policy <lib>`로 `.sav`/PPA 버전인지 먼저 확인.

---

## 📌 곁다리로 같이 잡은 문제들 (참고)

| 증상 | 원인 | 해결 |
| --- | --- | --- |
| 모든 노드 즉사 | `CYCLONEDDS_URI`가 down된 `enp118s0` 고정 | 루프백 인터페이스로 교체 |
| 검은 화면 / 서버 멈춤 | 좀비 ign 프로세스 transport 충돌 | `pkill` + `IGN_IP=127.0.0.1` |
| 흰 화면 / 모델 안 보임 | 오버레이 미소스 → `IGN_GAZEBO_RESOURCE_PATH` 없음 | `source install/setup.bash` |
| `failed to create dri2 screen` | 하이브리드 GPU + `render` 그룹 누락 | `usermod -aG render,video` + NVIDIA EGL 강제 |
