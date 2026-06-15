# Semantic Navigation AMR

Drive a mobile robot with natural language. Say *"go to the fire extinguisher"*
(or *"소화기 앞으로 가"*) and the robot resolves the named object to a map
coordinate it learned by looking around, then plans and navigates there with
Nav2 — stopping in front of the object, facing it.

Built on ROS 2 Humble + Ignition Gazebo (Fortress) with an AgileX Scout v2 and
an Intel RealSense D435i, in the AWS RoboMaker Small Warehouse world.

![demo](docs/media/demo.gif)

> _Recording: see [Demo](#demo) for how the GIF is captured from `./run_sim.sh`._

## What it does

Two pipelines run on top of Nav2:

```
[Perception — always on]
RGB-D camera ─▶ YOLOE open-vocab detection ─▶ 2D→3D (depth + tf2) ─▶ semantic map (map frame)

[Command — on request]
free-text command ─▶ parser (LLM, keyword fallback) ─▶ map lookup ─▶ approach pose ─▶ Nav2 NavigateToPose
```

- **Open-vocabulary perception.** A YOLOE detector finds objects, projects the
  bounding-box depth into the `map` frame via tf2, and accumulates them into a
  semantic map. Repeated observations are merged (EMA); an object is *confirmed*
  only after `min_observations`, which filters one-off false positives.
- **Natural-language commands.** A free-text command is parsed into a target
  object and an instance *selector*. The parser uses an LLM (Google Gemini, with
  JSON output) and **falls back to keyword matching** when the LLM is disabled
  or unavailable — so the demo always works offline.
- **Grounded goals.** The target is looked up in the semantic map (which can
  return *several* same-label instances); the commander picks the nearest (or
  *farthest*, if the command says so), computes an approach pose offset from the
  object and facing it, and sends it to Nav2. Costmap occupancy is delegated to
  Nav2 — a rejected goal is reported back as `accepted: false`.

## Packages

| Package | Role |
| --- | --- |
| `semantic_nav_msgs` | Custom msgs/srv (`DetectedObject3D[]`, `FindObject`, `NavigateToObject`) |
| `object_detector` | YOLOE detection + depth/tf2 projection to map frame |
| `semantic_map` | Object DB (data association + confirmation), `find_object` service, RViz markers, JSON persistence |
| `language_goal` | Command parser (LLM + keyword) + approach-pose math + Nav2 client |
| `semantic_nav_bringup` | Unified `sim.launch.py`, params, demo-object spawning, map |

## Quickstart

```bash
# Build
cd ~/nav2_semantic_ws
colcon build --symlink-install
source install/setup.bash

# Launch the full simulation (Gazebo + Scout + Nav2 + perception + command nodes)
./run_sim.sh
```

`run_sim.sh` wraps `ros2 launch semantic_nav_bringup sim.launch.py` with the
environment workarounds this machine needs (see `docs/TROUBLESHOOTING.md`).

Drive the robot around a little so the camera observes the fire extinguishers
and they get confirmed (green markers in RViz), then issue a command:

```bash
# Natural language (LLM parser) — varied phrasings all resolve:
ros2 service call /semantic_nav/navigate_to_object \
  semantic_nav_msgs/srv/NavigateToObject "{command: 'go stand by the fire extinguisher'}"

ros2 service call /semantic_nav/navigate_to_object \
  semantic_nav_msgs/srv/NavigateToObject "{command: '소화기 쪽으로 가줘'}"

# Instance selector — pick the farther of the two extinguishers:
ros2 service call /semantic_nav/navigate_to_object \
  semantic_nav_msgs/srv/NavigateToObject "{command: 'go to the farthest fire extinguisher'}"
```

The response is **asynchronous**: `accepted: true` means *Nav2 accepted the
goal* (not that the robot has arrived); `message` carries the chosen instance
count and distance. Arrival is observed in RViz.

### Enabling the LLM parser

The LLM parser is optional, uses Google Gemini (free tier), and reads its key
**only** from the environment — never from code or params (this repo is public):

```bash
pip install google-genai                # one-time
export GEMINI_API_KEY=...               # free key: https://aistudio.google.com/apikey
./run_sim.sh
```

Without a key/SDK (or with `llm_enabled: false` in
`semantic_nav_bringup/params/language_goal.yaml`), the node silently uses the
keyword parser, so the demo runs fully offline. The model is set by `llm_model`
(default `gemini-2.5-flash`).

## How it works (a few details)

- **Frames.** The robot body frame is `mobile_robot_base_link` (not the standard
  `base_link`). The camera publishes in `camera_color_optical_frame`; depth is
  `32FC1` (metres) and aligned to color. All projections use tf2 with the image
  timestamp.
- **Detection vocabulary.** YOLOE is fed *appearance* prompts (e.g. "red metal
  cylinder") that match the simulated meshes far better than semantic names, and
  results are remapped to canonical labels. The demo uses a single confirmed
  class (fire extinguisher) — see [Limitations](#limitations).
- **Persistence.** The semantic map is saved to JSON on shutdown (and
  periodically) and reloaded at startup, so the learned objects survive a
  restart (`persistence_path` in `params/semantic_map.yaml`).

## Testing

```bash
colcon test --packages-select language_goal semantic_map object_detector \
  && colcon test-result --verbose
```

Pure logic (command parsing, approach-pose math, data association, persistence,
the LLM parser with a mocked client) is unit-tested without ROS.

## Limitations

The simulation render is over-exposed, so YOLOE reliably detects only the fire
extinguisher (chairs/cones/primitives score < 0.03, and even a COCO chair
detector fails). The demo therefore uses a single class with two instances
~4 m apart, which is enough to show multi-instance disambiguation and the
nearest/farthest selector. See `docs/NOTES.md`.

## Future Work

- **Spatial relations** (`near` / `behind` / `between`). The parser schema
  already captures a `relation` field; relation-aware pose selection is not yet
  implemented. With the two-extinguisher demo, *"between the two extinguishers"*
  and *behind/in front of* are feasible; arbitrary object-to-object relations
  need a richer detected vocabulary.
- **8-direction candidate search.** When the default approach pose is occupied,
  query the global costmap and search candidate poses around the object instead
  of relying solely on Nav2 rejection.
- **Multi-class detection.** Texture-mapped meshes + better sim lighting (or a
  fine-tuned detector) to lift recall beyond the single demo class.
- **Camera–LiDAR fusion.** Use the 3D LiDAR (`/points`) to refine object range.
- **Dynamic-object handling.** The semantic map assumes static landmarks: it
  never deletes entries, and data association is label + distance only. So if an
  object is relocated more than `merge_distance` (0.5 m), a new entry is created
  while the stale one persists — the move is not reflected, and the robot may
  drive to the old, now-empty spot. Proper handling needs un-observation
  clearing (drop an object when its expected location is seen empty) and
  motion-based tracking (e.g. a Kalman filter) to tie A→B as one moving object,
  ideally separating static vs dynamic objects.

## Milestones

- [x] **M1** — Scout v2 + Nav2 bring-up; RViz 2D-goal driving.
- [x] **M2** — RGB-D (D435i) integration in gz sim; color/depth/`camera_info` topics.
- [x] **M3** — `object_detector` + `semantic_map`; map-frame object markers in RViz.
- [x] **M4** — `language_goal` keyword commander; end-to-end driving to the object.
- [ ] **M5** — LLM parser + instance selector, JSON persistence, demo/README
  (this milestone); spatial relations and 8-direction search documented as
  Future Work.

See `docs/SPEC.md` for the full design and `docs/NOTES.md` /
`docs/TROUBLESHOOTING.md` for implementation and environment notes.

## Demo

The GIF above is recorded from a `./run_sim.sh` session: drive to confirm the
extinguishers (green markers), then issue the service calls above and capture
RViz + the terminal. Save the recording to `docs/media/demo.gif`.
