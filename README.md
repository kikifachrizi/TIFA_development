# TIFA — General Overview

## Peran singkat
- **Nav2**: perencanaan global/lokal, costmap, behavior tree.
- **SLAM Toolbox/AMCL**: estimasi pose di frame `map`.
- **ros2_control**: terjemah `cmd_vel` ke aktuator diff-drive.
- **Supabase/PostgreSQL (opsional)**: kirim goal & log hasil.

---

## Fitur Inti
- 🚗 **Autonomous Navigation**: path planning & obstacle avoidance via Nav2.
- 🗺️ **SLAM/Localization**: peta & pose konsisten (SLAM Toolbox / AMCL).
- 🔧 **Hardware Abstraction**: `ros2_control` + driver diff-drive.
- ☁️ **Cloud Bridge (opsional)**: dispatch tujuan navigasi & telemetri ke DB.
- 🧠 **Advanced Control (roadmap)**: MPC/TEB untuk tracking halus & constraint-aware.

---

## Lingkup Teknis (Ringkas)

| Komponen | Catatan |
| --- | --- |
| **OS / SBC** | Ubuntu 22.04 pada Raspberry Pi 4 |
| **ROS 2** | Humble Hawksbill |
| **Persepsi** | LiDAR via `rplidar_ros` (LaserScan) |
| **Lokalisasi/SLAM** | `slam_toolbox` (online), atau AMCL pada peta statis |
| **Navigasi** | **Nav2** (planner, controller, recoveries, BT Navigator) |
| **Aktuasi** | **ros2_control** diff-drive → motor driver via USB serial (CH340/CP210x) |
| **Cloud/DB (opsional)** | Supabase / PostgreSQL (goal queue, status) |
| **Simulasi (opsional)** | Ignition/Gazebo + `gazebo_ros2_control` |

---

## 📦 Cabang & Struktur Repo

> Pisahkan **dokumen** dan **kode** untuk alur kerja yang rapi.

- **`main`** — Dokumentasi (README, proposal, aset gambar, SOP).
- **`dev`** — Integrasi pengembangan harian (kode).
- **`feat/*`** — Fitur fokus (mis. `feat/nav2-mpc`, `feat/ros2-control`).
- **`fix/*`** — Perbaikan bug.
- **`docs/*`** — Dokumentasi besar/terfokus.

**Direktori pada branch `main`:**
