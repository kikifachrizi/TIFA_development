# TIFA — Autonomous Mobile Robot Platform

> **General Overview & Documentation Hub** (branch `main`)  
> Cabang ini berisi dokumentasi, proposal, dan aset administratif. Kode & eksperimen ada di branch pengembangan (`dev`, `feat/*`, `fix/*`).

![Cover](assets/cover.png)

---

## TL;DR

- **TIFA** adalah platform **Autonomous Mobile Robot (AMR)** berbasis **ROS 2 Humble** di **Ubuntu 22.04** (target edge: **Raspberry Pi 4**).  
- Stack utama: **Nav2** (navigasi), **SLAM Toolbox** (SLAM), **rplidar_ros** (LiDAR), **ros2_control** (aktuasi diff-drive), integrasi cloud/database (mis. **Supabase/PostgreSQL**) untuk pengiriman goal/telemetri.  
- Branch ini = **dokumentasi**, bukan kode. Lihat “📦 Cabang & Struktur Repo” di bawah untuk cabang kode.

---

## Sejarah Singkat TIFA

> Ringkas & kronologis. Isi poin di bawah sesuai sejarah proyekmu.

- **[Tahun/Bulan]** — *Inisiasi ide*: …  
- **[Tahun/Bulan]** — *Prototype v0*: rangkaian diff-drive + LiDAR, teleop dasar…  
- **[Tahun/Bulan]** — *Integrasi ROS 2 Humble*: porting paket, Nav2 bring-up…  
- **[Tahun/Bulan]** — *Field test*: uji navigasi indoor, tuning costmap…  
- **[Tahun/Bulan]** — *Cloud Bridge*: kirim `NavigateToPose` via Supabase (PostgreSQL/`libpq`)…  
- **[Tahun/Bulan]** — *Roadmap ke MPC/TEB*: eksperimen controller advance…

> **Highlight**: tantangan teknis yang sempat muncul (USB serial CH340/CP210x, IP dinamis RPi, dependency Gazebo/ignition, dsb) dan solusinya singkat.

---

## Arsitektur Tingkat-Tinggi

```mermaid
flowchart LR
  subgraph Robot[TIFA Robot (ROS2 Humble, Ubuntu 22.04)]
    SENS[LiDAR / IMU / Kamera]
    PERCEPTION[Perception<br/>Filters, LaserScan]
    LOC[Localization/SLAM<br/>(SLAM Toolbox / AMCL)]
    PLAN[Nav2<br/>(BT Navigator, Costmaps, Planners)]
    CTRL[Controller<br/>(ros2_control / (opsi) MPC/TEB)]
    ACT[Actuators<br/>Motor Driver (ttyUSB)]
    SENS --> PERCEPTION --> LOC --> PLAN --> CTRL --> ACT
    LOC --> PLAN
  end

  subgraph Cloud[Backend (opsional)]
    DB[(Supabase / PostgreSQL)]
  end

  DB <--> |Goals, Telemetry| PLAN
