# ROS 2 Global Coordinate untuk Robot Differential-Drive di Gazebo

Proyek ini mengimplementasikan estimasi posisi global robot
differential-drive menggunakan perubahan sudut roda (*dead reckoning*),
lalu menggunakan posisi tersebut untuk mengarahkan robot menuju waypoint
di Gazebo.

## Fitur

-   Membaca posisi sudut roda melalui `/joint_states`.
-   Menghitung estimasi posisi global `x`, `y`, dan orientasi `theta`.
-   Menerbitkan pose melalui `/global_pose` menggunakan
    `geometry_msgs/msg/Pose2D`.
-   Mengubah target koordinat menjadi perintah gerak melalui
    `/input_ik`.
-   Menguji lintasan waypoint berbentuk persegi panjang.

## Arsitektur

``` text
Gazebo
  |
  v
/joint_states
  |
  v
position_tracker
  | menghitung x, y, theta
  v
/global_pose (Pose2D)
  |
  v
global_coordinate
  | mengirim Twist
  v
/input_ik
  |
  v
inverse_kin
  |
  v
/cmd_vel dan pengendali robot
  |
  v
Robot bergerak di Gazebo
```

> Catatan: topik akhir dan penghubung ke Gazebo mengikuti konfigurasi
> robot pada lingkungan masing-masing. Proyek ini menggunakan
> `/joint_states` untuk estimasi posisi dan tidak bergantung pada
> `/odom`.

## Parameter robot

  Parameter                                     Nilai
  ------------------------ --------------------------
  Jenis penggerak                  Differential-drive
  Radius roda                                  0,10 m
  Jarak antarroda                              0,45 m
  Joint roda kiri             `base_left_wheel_joint`
  Joint roda kanan           `base_right_wheel_joint`
  Ukuran waypoint contoh                    4 m × 2 m

Sesuaikan dimensi lintasan dengan ukuran world Gazebo yang digunakan.

## Rumus utama

Perubahan jarak tiap roda:

\[ `\Delta `{=tex}d_L = r`\Delta`{=tex}`\phi`{=tex}\_L,`\qquad`{=tex}
`\Delta `{=tex}d_R = r`\Delta`{=tex}`\phi`{=tex}\_R \]

Perpindahan linear dan perubahan orientasi:

\[ `\Delta `{=tex}d =
`\frac{\Delta d_L+\Delta d_R}{2}`{=tex},`\qquad`{=tex}
`\Delta`{=tex}`\theta `{=tex}= `\frac{\Delta d_R-\Delta d_L}{s}`{=tex}
\]

Pembaruan estimasi pose dengan orientasi rata-rata:

\[
x\_{k+1}=x_k+`\Delta `{=tex}d`\cos`{=tex}(`\theta`{=tex}\_k+`\Delta`{=tex}`\theta`{=tex}/2)
\]

\[
y\_{k+1}=y_k+`\Delta `{=tex}d`\sin`{=tex}(`\theta`{=tex}\_k+`\Delta`{=tex}`\theta`{=tex}/2)
\]

\[ `\theta`{=tex}\_{k+1}=`\theta`{=tex}\_k+`\Delta`{=tex}`\theta`{=tex}
\]

Transformasi waypoint dari kerangka lokal ke global:

\[
```{=tex}
\begin{bmatrix}x_g\\y_g\end{bmatrix}
```
=
```{=tex}
\begin{bmatrix}x_0\\y_0\end{bmatrix}
```
-   ```{=tex}
    \begin{bmatrix}\cos\theta_0&-\sin\theta_0\\
    \sin\theta_0&\cos\theta_0\end{bmatrix}
    \begin{bmatrix}x_l\\y_l\end{bmatrix}
    ```
    \]

## Struktur package yang relevan

``` text
simple_mover/
├── package.xml
├── setup.py
└── simple_mover/
    ├── position_tracker.py
    ├── inverse_kin.py
    ├── global_coordinate.py
    ├── rectangle_mover.py
    └── mover_node.py
```

Nama file yang tidak digunakan boleh tidak ada pada versi repository
kalian. Pastikan `setup.py` mendaftarkan executable yang memang
tersedia.

## Persiapan dan build

Jalankan perintah berikut dari Ubuntu/WSL, sesuaikan distro ROS 2 jika
diperlukan:

``` bash
cd ~/ros2_ws
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --packages-select simple_mover
source install/setup.bash
ros2 pkg executables simple_mover
```

Jika ROS 2 sudah otomatis tersumber di terminal dan perintah
`source /opt/ros/$ROS_DISTRO/setup.bash` tidak sesuai lingkungan,
gunakan setup file distro yang benar.

## Menjalankan simulasi

### Terminal 1 --- Gazebo

``` bash
source ~/ros2_ws/install/setup.bash
ros2 launch robin_bringup my_robot_gazebo.launch.py
```

### Terminal 2 --- estimator posisi

``` bash
source ~/ros2_ws/install/setup.bash
ros2 run simple_mover position_tracker
```

### Terminal 3 --- pengendali koordinat global

``` bash
source ~/ros2_ws/install/setup.bash
ros2 run simple_mover global_coordinate
```

`global_coordinate` mengasumsikan `inverse_kin` menerima
`geometry_msgs/msg/Twist` pada `/input_ik`, dengan `linear.x` sebagai
kecepatan maju dan `angular.z` sebagai kecepatan sudut. Verifikasi
antarmuka ini pada `inverse_kin.py` sebelum menjalankan jika kode kalian
berbeda.

Jangan jalankan `rectangle_mover` atau publisher gerak manual bersamaan
dengan `global_coordinate`, karena beberapa node dapat mengirim perintah
gerak yang saling bertentangan.

## Pemeriksaan topik

``` bash
ros2 topic list
ros2 topic echo /joint_states
ros2 topic echo /global_pose
ros2 topic info /input_ik
ros2 topic info /cmd_vel
```

`/global_pose` baru akan muncul ketika node `position_tracker` berjalan
dan publisher sudah dibuat di dalam node tersebut.

## Keterbatasan

-   *Dead reckoning* mengakumulasi kesalahan dari pembacaan roda dan
    selip, sehingga estimasi posisi dapat bergeser dari posisi
    sebenarnya.
-   Akurasi bergantung pada radius roda dan jarak antarroda yang benar.
-   Ukuran lintasan 4 m × 2 m harus sesuai dengan area simulasi.
-   Kinerja kontrol waypoint bergantung pada pemetaan topik dan perilaku
    node `inverse_kin`.

## Anggota tim

Tambahkan nama tiga anggota kelompok di sini:

1.  **Anggota 1:** konsep, latar belakang, dan rumus.
2.  **Anggota 2:** implementasi ROS 2, arsitektur, dan pengujian.
3.  **Anggota 3:** demonstrasi simulasi Gazebo.

## Kredit dan pengembangan

Proyek ini dibuat untuk pembelajaran ROS 2, odometri roda, transformasi
koordinat, dan kontrol gerak robot differential-drive di Gazebo.
