# Model Matematis Global Coordinate

Dokumen ringkas persamaan untuk proyek robot differential-drive.

## Parameter

- $r$: radius roda.
- $s$: jarak antarroda.
- $\Delta\phi_L,\Delta\phi_R$: perubahan sudut roda kiri dan kanan.
- $\Delta d_L,\Delta d_R$: perubahan jarak roda kiri dan kanan.
- $(x,y,\theta)$: estimasi pose global robot.

## 1. Jarak tempuh roda

$$
\Delta d_L = r\,\Delta\phi_L
$$

$$
\Delta d_R = r\,\Delta\phi_R
$$

## 2. Gerak badan robot

$$
\Delta d = \frac{\Delta d_L+\Delta d_R}{2}
$$

$$
\Delta\theta = \frac{\Delta d_R-\Delta d_L}{s}
$$

## 3. Pembaruan pose

$$
\theta_{\mathrm{mid}}=\theta_k+\frac{\Delta\theta}{2}
$$

$$
x_{k+1}=x_k+\Delta d\cos(\theta_{\mathrm{mid}})
$$

$$
y_{k+1}=y_k+\Delta d\sin(\theta_{\mathrm{mid}})
$$

$$
\theta_{k+1}=\operatorname{wrap}(\theta_k+\Delta\theta)
$$

## 4. Transformasi waypoint lokal ke global

# \$\$ \begin{bmatrix}x_g\y_g\end{bmatrix}

\begin{bmatrix}x_0\y_0\end{bmatrix} + \begin{bmatrix}
\cos\theta_0&-\sin\theta_0\\ \sin\theta_0&\cos\theta_0 \end{bmatrix}
\begin{bmatrix}x_l\y_l\end{bmatrix} \$\$

## 5. Inverse kinematics

$$
\omega_L=\frac{v-\frac{\omega s}{2}}{r}
$$

$$
\omega_R=\frac{v+\frac{\omega s}{2}}{r}
$$

GitHub mendukung blok matematika LaTeX yang dibatasi dengan `$$`,
sehingga persamaan akan ditampilkan secara terformat pada halaman
Markdown.
