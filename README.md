# Autonomous Security Patrol Robot

A simulated differential-drive robot patrols a two-room building. It localizes on a map it built itself, navigates with Nav2, detects a person and a backpack with YOLO, and logs everything so the results can be checked against measured data.

## Stack

| Item | Version / choice |
|---|---|
| OS | Ubuntu 24.04 (VMware VM) |
| ROS / simulator | ROS 2 Jazzy, Gazebo Harmonic (`ros_gz`) |
| Mapping | SLAM Toolbox 2.8.5 |
| Navigation | Nav2 1.3.13, AMCL, Regulated Pure Pursuit controller |
| Detection | Ultralytics 8.4.210, YOLOv8-nano, CPU PyTorch (in a venv) |

The VM runs at roughly 0.8 times real time. All logged times are **simulation time**.

## Repository layout

```
src/
  patrol_description/   robot model (URDF/xacro)
  patrol_sim/           Gazebo world, bridge config, Fuel model swap tool
  patrol_nav/           SLAM and Nav2 launch files, params, saved map
  patrol_task_manager/  patrol manager (5 waypoints, logs, skips failed waypoints)
  patrol_perception/    YOLO node, detection logger, detection analyzers
  patrol_bringup/       skeleton only (no one-command launch yet)
results/                run logs (CSV/YAML) for every experiment, without most snapshots
```

`results/` holds the evidence: `waypoint_events.csv`, `run_summary.yaml`, `frames.csv`, `detections.csv` and the analyzer outputs (`analysis_v4`, `analysis_v5`) per run. `results/experiment_changes.log` records world and configuration changes.

## Not in this repository

- The Python virtual environment (`~/yolo_venv`): create it with `python3 -m venv`, then install `torch`, `ultralytics` and `Pillow`.
- YOLO weights (`~/yolo_models/yolov8n.pt`): download once, so the VM needs no internet at run time.
- Gazebo Fuel models ("Standing person", "Backpack"): downloaded into `~/.gz/fuel/` on first use.
- Most detection snapshots (large).

## Build

```bash
cd ~/patrol_ws
colcon build --symlink-install
source install/setup.bash
export LIBGL_ALWAYS_SOFTWARE=1     # needed in the VM
```

## Run

Always start from a clean state and use the default spawn point (the AMCL initial pose is (0, 0, 0)). Wait 60 to 90 s after each launch.

**Patrol only**

```bash
ros2 launch patrol_sim sim.launch.py
ros2 launch patrol_nav nav.launch.py rviz:=false
bash src/patrol_nav/tools/check_nav2.sh          # all nodes should be active [3]
ros2 launch patrol_task_manager patrol.launch.py laps:=3
```

**Patrol with detection and logging**

```bash
# after sim and nav are running:
ros2 launch patrol_perception detect.launch.py
ros2 launch patrol_perception log.launch.py
ros2 launch patrol_task_manager patrol.launch.py laps:=3
```

Stop the logger with Ctrl+C when the patrol finishes. Detection must be evaluated only on runs with AMCL running.

**Score a detection run**

```bash
python3 src/patrol_perception/tools/analyze_detections.py ~/detection_runs/<run>
```

## Results (simulation time)

**Patrol**

| Condition | Result |
|---|---|
| Clean baseline, 3 laps (`baseline_3laps_clean`) | 15/15 waypoints, 138.2 s per lap |
| Detection on, 3 laps | No measurable lap-time cost |
| UDP transport test, 3 runs (`udp_test1` to `udp_test3`) | 45/45 waypoints, 0 aborts, 136.0 to 138.5 s per lap |

An earlier intermittent "error 0" abort occurred in about 4 of 85 attempts. Its cause is **not determined**. Zero aborts in 45 UDP attempts is consistent with a transport cause but is not statistically conclusive (one-sided Fisher p of about 0.18).

**Detection (frozen scoring rules, strict v4 and lenient v5 reported together)**

| | Result |
|---|---|
| Person | 81 to 90% of in-view frames; detected on every pass |
| Backpack | 34 to 47% of in-view frames, depending on the scorer |
| Backpack range | Detected on every close approach (first detections at 3.3 to 3.8 m); missed on every pass that stayed at about 4.35 m |

The backpack range limit applied equally to a second model ("Jansport backpack red"), which YOLO also labels `suitcase`.

## Known limitations

- YOLO's standard model labels the backpack as `suitcase`. The detector renames it to `backpack` and keeps the original label as `raw_class`. This is a model-specific workaround and would also count a real suitcase.
- Detection of the small backpack is reliable only within about 3.5 m.
- The robot pose lags fast turns, which causes some detections to be scored as unmatched. Only six unmatched person detections were inspected by eye; all were on a person.
- The analyzer rules changed during development (v1 to v5). The original v4 file was overwritten and was reconstructed, then validated by reproducing the recorded results.
- The blocked-doorway failure test was not completed.
- Several earlier patrol runs (11 Nov evening) have unrecorded conditions and are not used as evidence.
