<div align="center">

# ROS 2 Global Coordinate — Differential-Drive Robot

**Estimasi pose berbasis roda, transformasi koordinat, dan kontrol
waypoint di Gazebo**

![ROS 2](https://img.shields.io/badge/ROS%202-Python-22314E?logo=ros)
![Gazebo](https://img.shields.io/badge/Simulator-Gazebo-orange)
![License](https://img.shields.io/badge/License-MIT-green)

</div>

Repository ini berisi package ROS 2 `simple_mover` untuk simulasi robot
**differential-drive**. Sistem menghitung estimasi posisi global dari
perubahan sudut roda (*dead reckoning*), lalu mengarahkan robot ke
waypoint yang ditentukan.

> **Catatan kompatibilitas:** proyek awal dikembangkan pada workspace
> `~/ros2_ws` dengan launch `robin_bringup my_robot_gazebo.launch.py`.
> Package robot dan launch Gazebo tersebut merupakan dependensi
> eksternal dan tidak disertakan di repository ini.

## Daftar Isi

- [Fitur](#fitur)
- [Arsitektur Sistem](#arsitektur-sistem)
- [Parameter Robot](#parameter-robot)
- [Model Matematis](#model-matematis)
- [Struktur Repository](#struktur-repository)
- [Instalasi dan Build](#instalasi-dan-build)
- [Menjalankan Simulasi](#menjalankan-simulasi)
- [Pengujian Topik](#pengujian-topik)
- [Troubleshooting](#troubleshooting)
- [Keterbatasan](#keterbatasan)
- [Kontribusi Kelompok](#kontribusi-kelompok)

## Fitur

- Membaca posisi sudut roda kiri dan kanan dari `/joint_states`.
- Menghitung estimasi pose robot $(x,y,\theta)$ dengan *dead reckoning*.
- Menerbitkan pose melalui `/global_pose` (`geometry_msgs/msg/Pose2D`).
- Mengubah waypoint lokal menjadi koordinat global.
- Mengirim perintah gerak melalui `/input_ik`.
- Menyediakan node inverse kinematics untuk menghitung kecepatan roda.

## Arsitektur Sistem

``` mermaid
flowchart TD
    A[Gazebo robot] --> B["/joint_states"]
    B --> C["position_tracker"]
    C --> D["/global_pose<br/>Pose2D"]
    D --> E["global_coordinate"]
    E --> F["/input_ik<br/>Twist"]
    F --> G["inverse_kin"]
    G --> H["/left_wheel/command"]
    G --> I["/right_wheel/command"]
    G --> J["/cmd_vel"]
    H --> A
    I --> A
    J --> A
```

| Node                | Input           | Output                       | Tugas                                                 |
|---------------------|-----------------|------------------------------|-------------------------------------------------------|
| `position_tracker`  | `/joint_states` | `/global_pose`               | Menghitung estimasi $x,y,\theta$                      |
| `global_coordinate` | `/global_pose`  | `/input_ik`                  | Mengarahkan robot ke waypoint                         |
| `inverse_kin`       | `/input_ik`     | Perintah roda dan `/cmd_vel` | Mengubah kecepatan badan robot menjadi kecepatan roda |

> Jalur aktuator pada model Gazebo bergantung pada plugin dan
> konfigurasi robot. Pastikan nama topic output inverse kinematics
> sesuai dengan model yang digunakan.

## Parameter Robot

| Parameter               |                    Nilai | Keterangan                      |
|-------------------------|-------------------------:|---------------------------------|
| Radius roda, $r$        |                 `0.10 m` | Radius roda penggerak           |
| Jarak antarroda, $s$    |                 `0.45 m` | Jarak roda kiri ke kanan        |
| Joint kiri              |  `base_left_wheel_joint` | Nama joint pada `/joint_states` |
| Joint kanan             | `base_right_wheel_joint` | Nama joint pada `/joint_states` |
| Panjang lintasan contoh |                  `4.0 m` | Ukuran waypoint persegi panjang |
| Lebar lintasan contoh   |                  `2.0 m` | Ukuran waypoint persegi panjang |

Sesuaikan parameter dengan model robot dan ukuran world Gazebo. Lintasan
4 m × 2 m membutuhkan ruang simulasi yang cukup.

## Model Matematis

Persamaan di bawah menggunakan notasi matematika LaTeX. GitHub akan
merender blok persamaan ini sebagai equation, sehingga lebih mudah
dibaca daripada rumus dalam satu baris teks.

### 1. Perubahan jarak roda

Sudut roda diukur dalam radian. Perubahan sudut dikalikan radius roda
untuk mendapatkan jarak tempuh roda:

$$
\Delta d_L = r\,\Delta\phi_L
$$

$$
\Delta d_R = r\,\Delta\phi_R
$$

Keterangan:

- $r$: radius roda (m).
- $\Delta\phi_L$: perubahan sudut roda kiri (rad).
- $\Delta\phi_R$: perubahan sudut roda kanan (rad).
- $\Delta d_L,\Delta d_R$: jarak tempuh roda kiri dan kanan (m).

### 2. Perpindahan linear dan perubahan orientasi

Perpindahan linear robot merupakan rata-rata jarak kedua roda:

$$
\Delta d = \frac{\Delta d_L+\Delta d_R}{2}
$$

Perubahan orientasi robot ditentukan oleh selisih jarak roda dibagi
jarak antarroda:

$$
\Delta\theta = \frac{\Delta d_R-\Delta d_L}{s}
$$

dengan $s$ adalah jarak antara roda kiri dan kanan (m).

### 3. Pembaruan pose global (*dead reckoning*)

Untuk mengurangi galat pendekatan selama satu langkah, posisi diperbarui
menggunakan orientasi tengah:

$$
\theta_{\mathrm{mid}} = \theta_k + \frac{\Delta\theta}{2}
$$

$$
x_{k+1} = x_k + \Delta d\cos(\theta_{\mathrm{mid}})
$$

$$
y_{k+1} = y_k + \Delta d\sin(\theta_{\mathrm{mid}})
$$

$$
\theta_{k+1} = \operatorname{wrap}(\theta_k+\Delta\theta)
$$

Fungsi $\operatorname{wrap}$ membatasi sudut ke rentang $[-\pi,\pi]$.

### 4. Transformasi waypoint lokal ke global

Waypoint ditentukan dalam koordinat lokal relatif terhadap pose awal
$(x_0,y_0,\theta_0)$. Transformasinya:

# \$\$ \begin{bmatrix} x_g\\ y_g \end{bmatrix}

\begin{bmatrix} x_0\\ y_0 \end{bmatrix} + \begin{bmatrix} \cos\theta_0 &
-\sin\theta_0\\ \sin\theta_0 & \cos\theta_0 \end{bmatrix}
\begin{bmatrix} x_l\\ y_l \end{bmatrix} \$\$

- $(x_l,y_l)$: waypoint dalam koordinat lokal.
- $(x_g,y_g)$: waypoint dalam koordinat global.
- $(x_0,y_0,\theta_0)$: pose robot ketika pengendali mulai berjalan.

### 5. Kecepatan roda dari inverse kinematics

Node `inverse_kin` menerima kecepatan linear $v$ dan kecepatan sudut
$\omega$ dari pesan `Twist`:

$$
v = \texttt{linear.x},\qquad
\omega = \texttt{angular.z}
$$

Kecepatan sudut roda kiri dan kanan:

$$
\omega_L = \frac{v-\frac{\omega s}{2}}{r}
$$

$$
\omega_R = \frac{v+\frac{\omega s}{2}}{r}
$$

Jika $v_L=v_R$, robot bergerak lurus. Jika kecepatan kedua roda berbeda,
robot berbelok.

## Struktur Repository

``` text
ros2-global-coordinate/
├── README.md
├── LICENSE
├── .gitignore
├── docs/
│   └── EQUATIONS.md
└── simple_mover/
    ├── package.xml
    ├── setup.py
    ├── setup.cfg
    ├── resource/
    │   └── simple_mover
    └── simple_mover/
        ├── __init__.py
        ├── position_tracker.py
        ├── global_coordinate.py
        └── inverse_kin.py
```

## Instalasi dan Build

### 1. Prasyarat

- ROS 2 sudah terpasang di Ubuntu/WSL.
- Workspace bernama `~/ros2_ws`.
- Package robot `robin_bringup` dan launch Gazebo tersedia di lingkungan
  lokal.

### 2. Salin package

Ekstrak repository, lalu salin folder `simple_mover` ke
`~/ros2_ws/src/`. Jika package `simple_mover` lama sudah ada, cadangkan
terlebih dahulu sebelum menggantinya.

### 3. Build

Jalankan di terminal Ubuntu/WSL:

``` bash
cd ~/ros2_ws
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --packages-select simple_mover
source install/setup.bash
ros2 pkg executables simple_mover
```

Jika `$ROS_DISTRO` tidak disetel otomatis, gunakan setup file distro ROS
2 yang memang terpasang di komputer.

## Menjalankan Simulasi

Gunakan terminal terpisah. Jangan menjalankan pengendali gerak manual
atau `rectangle_mover` bersamaan dengan `global_coordinate`.

### Terminal 1 — Gazebo

``` bash
source ~/ros2_ws/install/setup.bash
ros2 launch robin_bringup my_robot_gazebo.launch.py
```

### Terminal 2 — Position tracker

``` bash
source ~/ros2_ws/install/setup.bash
ros2 run simple_mover position_tracker
```

### Terminal 3 — Global coordinate controller

``` bash
source ~/ros2_ws/install/setup.bash
ros2 run simple_mover global_coordinate
```

Node `global_coordinate` mengirim pesan `geometry_msgs/msg/Twist` ke
`/input_ik`. Node `inverse_kin` harus berjalan agar perintah tersebut
diterjemahkan ke output aktuator:

### Terminal 4 — Inverse kinematics

``` bash
source ~/ros2_ws/install/setup.bash
ros2 run simple_mover inverse_kin
```

**Urutan praktis:** jalankan Gazebo, kemudian `inverse_kin`,
`position_tracker`, dan terakhir `global_coordinate`. Pastikan tidak ada
node lain yang mengirim perintah gerak secara bersamaan.

## Pengujian Topik

Daftar topik yang tersedia:

``` bash
ros2 topic list
```

Periksa data sudut roda:

``` bash
ros2 topic echo /joint_states
```

Periksa estimasi pose global:

``` bash
ros2 topic echo /global_pose
```

Periksa antarmuka perintah:

``` bash
ros2 topic info /input_ik
ros2 topic info /cmd_vel
```

Jika `/global_pose` belum muncul, pastikan `position_tracker` sedang
berjalan. Jika robot tidak bergerak, pastikan `inverse_kin` menerima
`/input_ik` dan output-nya sesuai dengan plugin penggerak pada model
Gazebo.

## Troubleshooting

| Gejala                                | Hal yang diperiksa                                                       |
|---------------------------------------|--------------------------------------------------------------------------|
| `/global_pose` belum muncul           | Jalankan `position_tracker`; periksa import `Pose2D` dan publisher       |
| `position_tracker` menunggu joint     | Periksa apakah `/joint_states` mengandung nama joint yang sesuai         |
| Robot tidak bergerak                  | Periksa `inverse_kin`, subscriber `/input_ik`, dan topic aktuator Gazebo |
| Posisi tidak berubah                  | Pastikan roda benar-benar bergerak dan data joint berubah                |
| Robot menyimpang dari lintasan        | Periksa radius roda, jarak antarroda, selip, toleransi, dan ukuran world |
| Robot menerima perintah tak konsisten | Hentikan publisher manual dan node penggerak lain                        |

## Keterbatasan

- Metode *dead reckoning* tidak mengoreksi drift dengan sensor
  lokalisasi eksternal.
- Kesalahan radius roda, jarak antarroda, dan selip dapat terakumulasi.
- Hasil posisi merupakan estimasi, bukan ground truth dari Gazebo.
- Kesesuaian topic aktuator harus diverifikasi untuk model robot yang
  digunakan.

## Kontribusi Kelompok

Isi nama anggota dan perannya sebelum repository dikumpulkan:

| Anggota   | Peran                                             |
|-----------|---------------------------------------------------|
| Anggota 1 | Konsep, teori, dan rumus                          |
| Anggota 2 | Implementasi ROS 2, pengujian, dan dokumentasi    |
| Anggota 3 | Menjalankan dan mendemonstrasikan simulasi Gazebo |

## Lisensi

Proyek ini menggunakan lisensi MIT. Lihat [`LICENSE`](LICENSE).
