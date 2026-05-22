# Portfolio 2 — Sensor Fusion: Wheel Odometry + Monocular ORB-SLAM

Sensor fusion project dat **wheel encoder odometry** (differentieel-drive kinematica) combineert met **monoculaire ORB-SLAM** (visuele SLAM) via een **Extended Kalman Filter (EKF)**. Gebouwd voor het Duckietown-platform met ROS Noetic.

---

## Inhoudsopgave

1. [Projectoverzicht](#projectoverzicht)
2. [Architectuur](#architectuur)
3. [Repository structuur](#repository-structuur)
4. [ROS Nodes & Topics](#ros-nodes--topics)
5. [Gebruikte technologieën & dependencies](#gebruikte-technologieën--dependencies)
6. [Installatie](#installatie)
7. [Pangolin installatie (troubleshooting)](#pangolin-installatie-troubleshooting)
8. [Uitvoeren](#uitvoeren)
9. [EKF Sensor Fusion — uitleg](#ekf-sensor-fusion--uitleg)
10. [ORB-SLAM pipeline — uitleg](#orb-slam-pipeline--uitleg)
11. [Configuratie-parameters](#configuratie-parameters)
12. [Bekende beperkingen](#bekende-beperkingen)
13. [Wat niet werkte & aanpassingen](#wat-niet-werkte--aanpassingen)
14. [Auteur](#auteur)

---

## Projectoverzicht

Dit project implementeert een compleet localisatie-systeem voor een Duckiebot:

| Bron | Methode | Sterkte | Zwakte |
|------|---------|---------|--------|
| **Wheel encoders** | Differentieel-drive kinematica | Hoge frequentie, soepel | Drift over tijd (slip, kalibratie) |
| **Monoculaire camera** | ORB-SLAM (feature matching + pose recovery) | Corrigeert drift | Schaalambiguïteit, lagere frequentie, gevoelig voor textuur |
| **EKF Fusion** | Extended Kalman Filter | Best of both worlds | — |

De drie trajecten (odometry, ORB-SLAM, gefused) worden real-time gevisualiseerd via:
- **Pangolin** — 3D point cloud + camera trajectory (op de laptop)
- **OpenCV path viewer** — 2D bird's-eye view van alle drie paden (op de laptop)

---

## Architectuur

```
┌─────────────────────────────────────────────────────────────────┐
│                        DUCKIEBOT (Docker)                        │
│                                                                 │
│  ┌──────────────┐     ┌──────────────────┐     ┌────────────┐  │
│  │ encoder_pose │────▶│ sensor_fusion    │────▶│ /sensor_    │  │
│  │ _node        │     │ _node (EKF)      │     │ fusion/pose │  │
│  │              │     │                  │     └────────────┘  │
│  │ /{veh}/pose  │     │  TCP:9998 ───────────────────────────────▶ path_viewer
│  └──────────────┘     └──────────────────┘                     │    (laptop)
│                              ▲                                  │
│  ┌──────────────┐            │                                  │
│  │ orb_slam     │────────────┘                                  │
│  │ _node        │──▶ /orb_slam/point_cloud                     │
│  │              │──▶ /orb_slam/camera_poses                    │
│  │              │──▶ /orb_slam/annotated_image                 │
│  │  TCP:9999 ────────────────────────────────────────────────────▶ visualizer
│  └──────────────┘                                               │    (laptop)
│         ▲                                                       │
│         │                                                       │
│  /{veh}/camera_node/image/compressed                            │
└─────────────────────────────────────────────────────────────────┘
```

---

## Repository structuur

```
portfolio-2-odometry-orb/
├── Dockerfile                          # Duckietown dt-core image (ROS Noetic)
├── dependencies-apt.txt                # APT packages (libgl, python3-tk)
├── dependencies-py3.txt                # pip packages (open3d, matplotlib)
├── launchers/
│   └── default.sh                      # Entrypoint → roslaunch sensor_fusion.launch
├── packages/
│   ├── encoder_pose/                   # Wheel encoder odometry package
│   │   ├── include/odometry/
│   │   │   ├── __init__.py
│   │   │   └── odometry.py            # delta_phi() + estimate_pose() — differentieel-drive
│   │   ├── src/
│   │   │   └── encoder_pose_node.py   # ROS node: leest encoder ticks → publiceert /{veh}/pose
│   │   ├── launch/
│   │   │   └── encoder_pose_node.launch
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   └── orb_slam_node/                  # ORB-SLAM + sensor fusion + visualizers
│       ├── src/
│       │   ├── orb_slam_node.py                # ORB-SLAM pipeline (feature → match → pose → triangulate)
│       │   ├── pointcloud_visualizer_node.py   # Pangolin 3D viewer (multiprocessing)
│       │   ├── sensor_fusion_node.py           # EKF sensor fusion node
│       │   └── path_viewer_node.py             # 2D path viewer node (OpenCV)
│       ├── launch/
│       │   ├── sensor_fusion.launch            # VOLLEDIGE PIPELINE (alle 3 nodes)
│       │   ├── orb_slam.launch                 # Alleen ORB-SLAM node
│       │   └── visualizer.launch               # Alleen de Pangolin visualizer
│       ├── CMakeLists.txt
│       └── package.xml
├── visualizer_standalone.py            # Laptop-side: Pangolin 3D viewer (geen ROS nodig)
├── path_viewer_standalone.py           # Laptop-side: 2D path viewer (geen ROS nodig)
├── run_visualizer.sh                   # Shell wrapper voor visualizer_standalone.py
├── run_path_viewer.sh                  # Shell wrapper voor path_viewer_standalone.py
├── docs/                               # Sphinx documentatie configuratie
└── assets/                             # Module assets
```

---

## ROS Nodes & Topics

### Nodes

| Node | Package | Script | Functie |
|------|---------|--------|---------|
| `orb_slam_node` | `orb_slam_node` | `orb_slam_node.py` | Monoculaire SLAM pipeline; publiceert point cloud + camera poses |
| `pointcloud_visualizer_node` | `orb_slam_node` | `pointcloud_visualizer_node.py` | Pangolin 3D visualisatie van point cloud + trajectory |
| `encoder_pose_node` | `encoder_pose` | `encoder_pose_node.py` | Leest wheel encoder ticks, berekent pose via differentieel-drive model |
| `sensor_fusion_node` | `orb_slam_node` | `sensor_fusion_node.py` | EKF: odometry prediction + ORB-SLAM correction + schaalschatting |
| `path_viewer_node` | `orb_slam_node` | `path_viewer_node.py` | 2D bird's-eye view van alle trajecten (OpenCV) |

### Topics

| Topic | Type | Beschrijving |
|-------|------|-------------|
| `/{veh}/camera_node/image/compressed` | `sensor_msgs/CompressedImage` | Camera input (van Duckiebot) |
| `/{veh}/camera_node/camera_info` | `sensor_msgs/CameraInfo` | Camera intrinsics (na kalibratie) |
| `/{veh}/pose` | `nav_msgs/Odometry` | Wheel encoder odometry (output encoder_pose_node) |
| `/orb_slam/point_cloud` | `sensor_msgs/PointCloud2` | 3D puntwolk van de omgeving |
| `/orb_slam/camera_poses` | `std_msgs/Float64MultiArray` | ORB-SLAM camera poses (4×4 matrices) |
| `/orb_slam/annotated_image` | `sensor_msgs/CompressedImage` | Camera beeld met getekende features |
| `/sensor_fusion/pose` | `nav_msgs/Odometry` | **Gefusede pose** (EKF output) met covariance |

### TCP Streams (voor standalone laptop-viewers)

| Poort | Bron | Data |
|-------|------|------|
| `9999` | `orb_slam_node` | Point cloud + camera poses + annotated image |
| `9998` | `sensor_fusion_node` | Path histories (odometry, SLAM, fused) |

---

## Gebruikte technologieën & dependencies

### Software stack

| Component | Versie / Details |
|-----------|-----------------|
| **ROS** | Noetic (via Duckietown `dt-core` ente) |
| **Python** | 3.8+ |
| **OpenCV** | 4.x — ORB feature detection, BFMatcher, recoverPose, triangulatePoints |
| **NumPy** | Lineaire algebra, matrixbewerkingen |
| **Pangolin** | `pypangolin` (v0.8+) — 3D OpenGL visualisatie |
| **PyOpenGL** | OpenGL bindings voor Pangolin rendering |
| **Open3D** | Fallback 3D visualisatie (als Pangolin niet beschikbaar) |
| **Matplotlib** | Fallback 2D plotting |
| **Docker** | Containerisatie voor Duckietown |
| **Duckietown Shell** (`dts`) | Build & deploy tooling |

### Python packages (in container)

Zie `dependencies-py3.txt`:
```
open3d
matplotlib
```

### Python packages (laptop — standalone viewers)

```
numpy
opencv-python
PyOpenGL
pangolin  (pypangolin)
```

---

## Installatie

### 1. Clone de repository

```bash
git clone https://github.com/quincysoerohardjo2002/portfolio-2-odometry-orb.git
cd portfolio-2-odometry-orb
```

### 2. Build de Docker image (voor Duckiebot)

```bash
# Voor simulatie (amd64):
dts devel build -a amd64

# Voor echte Duckiebot (arm64):
dts devel build -a arm64v8 -H [DUCKIEBOT_HOSTNAME]
```

### 3. Laptop-omgeving (voor standalone viewers)

```bash
python3 -m venv ~/ORBSLAM/venv
source ~/ORBSLAM/venv/bin/activate
pip install numpy opencv-python PyOpenGL
```

Voor Pangolin — zie de volgende sectie.

---

## Pangolin installatie (troubleshooting)

Pangolin (`pypangolin`) is nodig voor de 3D-visualisatie op de laptop. De installatie gaf aanvankelijk problemen door **versie-incompatibiliteiten** met de systeembibliotheek en Python bindings.

### Probleem

De standaard `pip install pangolin` installeerde een versie die **niet compatibel** was met de geïnstalleerde OpenGL/GLEW-versies op Ubuntu. Dit resulteerde in segfaults of importfouten.

### Oplossing: Pangolin bouwen vanuit broncode

```bash
# 1. Installeer systeem-dependencies
sudo apt-get install -y \
    libgl1-mesa-dev libglew-dev cmake \
    libpython3-dev python3-numpy \
    libeigen3-dev

# 2. Clone Pangolin (specifieke versie die werkt)
git clone --recursive https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin
git checkout v0.8

# 3. Build
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release \
         -DBUILD_PANGOLIN_PYTHON=ON \
         -DPYTHON_EXECUTABLE=$(which python3)
make -j$(nproc)

# 4. Installeer Python bindings
cd ../python
pip install .
```

### Verificatie

```bash
python3 -c "import pangolin; print('Pangolin OK')"
```

> **Tip:** Als je `libGL error: MESA-LOADER: failed to open` krijgt, installeer dan `mesa-utils` en controleer of je GPU-drivers correct zijn.

---

## Uitvoeren

### Optie A: Volledige pipeline op Duckiebot (Docker)

```bash
# 1. Pas het voertuignaam aan in sensor_fusion.launch (default: duckiebot21)
# 2. Start de container:
dts devel run \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  --net host
```

Dit start `sensor_fusion.launch` dat alle drie nodes activeert:
- `encoder_pose_node` — wheel odometry
- `orb_slam_node` — visuele SLAM
- `sensor_fusion_node` — EKF fusion

### Optie B: Standalone viewers op laptop (geen ROS nodig)

De nodes streamen data via TCP. Op je laptop:

```bash
# 3D Point Cloud + Camera Trajectory (Pangolin):
./run_visualizer.sh [DUCKIEBOT_IP]

# 2D Bird's-eye Path Viewer (OpenCV):
./run_path_viewer.sh [DUCKIEBOT_IP]
```

De path viewer toont:
- **Blauw** = Wheel encoder odometry
- **Groen** = ORB-SLAM traject
- **Rood** = Gefused (EKF) traject

### Optie C: Alleen ORB-SLAM (zonder sensor fusion)

```bash
# In de launch file:
roslaunch orb_slam_node orb_slam.launch
```

---

## EKF Sensor Fusion — uitleg

De `sensor_fusion_node` implementeert een Extended Kalman Filter met state vector **[x, y, θ]**:

### Prediction stap (wheel odometry)

Bij elk odometry-bericht wordt de state voorspeld op basis van de verandering in (x, y, θ) van de encoders:

```
x̂⁻ = x̂ + Δx_odom
P⁻  = P + Q
```

Waar `Q` de process-noise covariance is (onzekerheid in de wielen).

### Correction stap (ORB-SLAM)

Wanneer een nieuw ORB-SLAM pose-bericht binnenkomt:

1. **Schaalschatting** — Monoculaire SLAM heeft geen absolute schaal. De node schat de schaal via een Exponential Moving Average (EMA) door de afgelegde afstand van SLAM te vergelijken met die van odometry.
2. **Innovation** — Het verschil tussen de geschaalde SLAM-meting en de huidige state.
3. **Kalman Gain** — Weegt af hoeveel de correctie mag bijdragen.
4. **Update** — State en covariance worden gecorrigeerd.

```
K = P⁻ · H^T · (H · P⁻ · H^T + R)^{-1}
x̂ = x̂⁻ + K · (z - H · x̂⁻)
P  = (I - K · H) · P⁻
```

### Tuning-parameters

| Parameter | Default | Beschrijving |
|-----------|---------|-------------|
| `~q_x` | 0.002 | Process noise x (hoe onzeker zijn de wielen) |
| `~q_y` | 0.002 | Process noise y |
| `~q_theta` | 0.005 | Process noise heading |
| `~r_x` | 0.05 | Measurement noise x (hoe onzeker is ORB-SLAM) |
| `~r_y` | 0.05 | Measurement noise y |
| `~r_theta` | 0.1 | Measurement noise heading |
| `~scale_alpha` | 0.1 | EMA smoothing voor schaalschatting |
| `~scale_window` | 0.05 | Minimale afgelegde afstand voordat schaal wordt bijgewerkt |

---

## ORB-SLAM pipeline — uitleg

De `orb_slam_node` voert per framepaar de volgende stappen uit:

### 1. Feature Extraction (ORB)

ORB (**O**riented FAST and **R**otated **B**RIEF) detecteert visueel opvallende hoekpunten in elk grijswaardenframe en berekent compacte binaire descriptors die rotatiebestendig zijn. Er worden maximaal `n_features` (standaard 5000) keypoints geëxtraheerd per frame.

### 2. Feature Matching

Brute-Force Hamming-distance matcher met cross-check vergelijkt descriptors tussen opeenvolgende frames. Matches worden gesorteerd op afstand zodat de beste correspondences worden gebruikt.

### 3. Camera Pose Estimation

Uit de matchparen wordt de **Fundamental matrix** F geschat met RANSAC. Daarna:

```
E = K^T · F · K            (Essential matrix)
[R | t] = decomposeE(E)    (recoverPose)
```

De node houdt een cumulatieve camera-to-world transformatie bij:

```
R_cw_new = R_cw_old · R^T
t_cw_new = t_cw_old − R_cw_new · t
```

### 4. Triangulatie

Matched pixelcoördinaten worden genormaliseerd en getrianguleerd via `cv2.triangulatePoints`. Punten achter de camera worden verworpen. De 3D-punten worden getransformeerd naar het wereldframe en geaccumuleerd in een rolling buffer.

> **Schaalambiguïteit** — een enkele camera kan geen absolute schaal bepalen. De EKF sensor fusion node compenseert dit door de schaal te schatten t.o.v. de wheel odometry.

---

## Configuratie-parameters

### `sensor_fusion.launch` (volledige pipeline)

| Parameter | Locatie | Default | Beschrijving |
|-----------|---------|---------|-------------|
| `veh` | launch arg | `duckiebot21` | Naam van de robot |
| `image_topic` | orb_slam_node | `/{veh}/camera_node/image/compressed` | Camera topic |
| `camera_info_topic` | orb_slam_node | `/{veh}/camera_node/camera_info` | Camera info topic |
| `n_features` | orb_slam_node | `5000` | ORB keypoints per frame |
| `max_map_points` | orb_slam_node | `10000` | Max 3D-punten in de map |
| `camera_matrix` | orb_slam_node | zie launch file | Fallback K-matrix (9 floats) |

### Visualizer parameters

| Parameter | Default | Beschrijving |
|-----------|---------|-------------|
| `point_cloud_topic` | `/orb_slam/point_cloud` | Point cloud input |
| `image_topic` | `/orb_slam/annotated_image` | Annotated image input |
| `poses_topic` | `/orb_slam/camera_poses` | Camera poses input |

---

## Bekende beperkingen

- **Schaalambiguïteit** — Monoculaire SLAM kan geen metrische schaal bepalen. De EKF schat de schaal, maar deze is afhankelijk van voldoende verplaatsing.

- **Geen loop closure** — De huidige implementatie detecteert niet wanneer de camera een eerder gezien locatie herbezoekt. Drift accumuleert over lange trajecten.

- **Texture-arme omgevingen** — ORB features vereisen visuele textuur. Witte muren of donkere scènes produceren weinig matches.

- **Pangolin op arm64** — Pangolin is niet beschikbaar als pip wheel voor arm64. De visualiser draait daarom op de laptop (standalone via TCP).

- **Encoder drift** — Wheel odometry drifts door wielslip en kalibratiefouten. De EKF compenseert dit, maar bij langdurig stilstaan of abrupte bewegingen kan de schaalschatting tijdelijk onjuist zijn.

---

## Wat niet werkte & aanpassingen

### Pangolin versie-incompatibiliteit

**Probleem:** `pip install pangolin` installeerde een verouderde of incompatibele versie die crashte bij het openen van het OpenGL-venster (segfault of `ImportError`).

**Oplossing:** Pangolin v0.8 vanuit broncode compileren met de juiste Python- en OpenGL-flags (zie [Pangolin installatie](#pangolin-installatie-troubleshooting)).

### Open3D fallback verwijderd

**Probleem:** De originele visualizer gebruikte Open3D, maar dit gaf problemen op arm64 en was minder geschikt voor real-time streaming.

**Oplossing:** Overgestapt naar Pangolin (in een apart `multiprocessing.Process`) met TCP-streaming naar een standalone laptop-client. Open3D/matplotlib bleef als fallback in de dependencies.

### Encoder odometry stubs

**Probleem:** De originele `odometry.py` bevatte lege functies (`delta_phi` en `estimate_pose` retourneerden `None`).

**Oplossing:** Geïmplementeerd met het differentieel-drive kinematica model:
- `delta_phi`: tick-verschil × 2π / resolutie
- `estimate_pose`: mid-point integratie met arc lengths

### Monoculaire schaal

**Probleem:** ORB-SLAM levert poses in willekeurige eenheden (geen meters). Direct fuseren met odometry geeft nonsens.

**Oplossing:** EMA-gebaseerde schaalschatting in de sensor fusion node. De schaal wordt continu bijgewerkt door de afgelegde SLAM-afstand te vergelijken met de odometry-afstand.

---

## Auteur

**Quincy Soerohardjo** — Fontys Hogescholen ICT  
GitHub: [quincysoerohardjo2002](https://github.com/quincysoerohardjo2002)

Gebaseerd op het originele [orb-slam-demo](https://github.com/VikramRadhakrishnan/orb-slam-demo) door Vikram Radhakrishnan.
