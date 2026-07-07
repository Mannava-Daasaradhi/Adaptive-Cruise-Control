# D-015 — Car visualization: rviz2 live 3D + offline MP4 renderer

**Date:** 2026-07-07 · **Status:** accepted · **Type:** architecture / demo

## Context
The user asked for an "actual car simulation" — seeing cars drive, not just
plots. Asked which flavor (AskUserQuestion), the user chose **rviz2 3D +
MP4 renderer** over a Gazebo physics simulator and over 2D-only animation.

## Options considered
1. **rviz2 markers + MP4 renderer** — CHOSEN. Zero new installs
   (`ros-jazzy-desktop` ships rviz2, WSLg works per [D-012]); the physics
   stays our paper-validated τ-lag model; MP4s drop into slides.
2. Gazebo Harmonic — multi-GB install and, decisively, it would *replace*
   the validated vehicle model with engine/tire physics, breaking the
   reproduce-the-paper contract ([D-007]).
3. 2D matplotlib animation only — kept as the second half of option 1.

## Decision — live 3D (`viz_node.py` in `cacc_platoon`)
- Subscribes to all `/platoon/v*/state`; publishes a
  `visualization_msgs/MarkerArray` on `/platoon/markers`:
  - one CUBE per vehicle (L = 4 m × 1.9 m × 1.5 m) at its true longitudinal
    position, lane-centered;
  - body color = spacing-error heat: green (|e| = 0) → red (|e| ≥ 1 m),
    leader white;
  - TEXT marker above each car: index, speed, spacing error.
- Publishes a TF `map → platoon` at the leader's position; the shipped
  rviz config uses **`platoon` as the Fixed Frame**, so the camera rides
  along with the platoon (4 km of travel stays on screen) while the road
  dashes (grey CUBE strip markers in `map`) slide past underneath — that
  relative motion is what makes it read as *driving*.
- Enabled via `viz:=true` launch argument; rviz2 launched separately with
  `config/platoon.rviz` (script `scripts/ros2_view_demo.sh`).

## Decision — offline MP4/GIF (`scripts/render_platoon_video.py`)
- Input: any recorder CSV (`results/ros2/*.csv`) **or** any scenario YAML
  (runs the offline core first — so the animation works without ROS).
- Top-down highway view, camera tracking the platoon; cars as rounded
  rectangles with the same error-color scale; HUD with time, leader speed,
  live speed traces and spacing-error traces below the road.
- Writer: ffmpeg (H.264 MP4) if available, else Pillow GIF fallback;
  default ~20 s of video for the 200 s run (10× time-lapse, configurable).

## Why not follow each car with its own camera / first-person view
The course story is *string stability* — a platoon-level property; the
platoon-frame top-down/chase view is the one where gap waves are visible.

## Consequences
- `cacc_platoon` gains a `viz_node` entry point and a `platoon.rviz`
  config; `package.xml` gains `visualization_msgs`, `geometry_msgs`,
  `tf2_ros` deps.
- Demo recipe documented in `08_ROS2_Architecture.md` §7.
