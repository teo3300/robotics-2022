# Studying the Mechanum wheel robot equations

Variables notation:

$r =$ wheel radius\
$\pm l =$ wheel position along $x$\
$\pm w = $ wheel position along $y$\
$T =$ gear ratio\
$N =$ Ticks per revolution

Robot specific parameters:
Parameter | Value
-:|-
$r$ | $0.07 $ m
$l$ | $0.200$ m
$w$ | $0.169$ m
$T$ | $5:1  $

---

### The Long way around

## Hypothesis
4 Mechanum wheels behave like 4 omni-directional wheels perpendicular one another, with an angular velocity of the mechanum wheel $\omega_m$ with a radius $r_m$ and an angular velocity of the omni-directional wheel $\omega_o$ with a radius $r_o$ following the formula:

$\omega_m r_m = \sin\left(\frac\pi{4}\right) \omega_o r_o \Rightarrow \omega_m r_m = \frac {\sqrt 2} 2 \omega_o r_o$

With $r_o = r_m = r$

With a positive angular velocity meaning the actuator is rotating counter-clockwise.

Wheels numbered from left to rigth, from front to back (front is aligned with positive $x$ axis, left side aligned with positive $y$ axis)

Kinematic equations in matrix form will be:

$$
\left[\begin{array}{}
v_x\\
v_y\\
\dot\theta
\end{array}\right]
=
A
\left[\begin{array}{}
\omega^o_1\\
\omega^o_2\\
\omega^o_3\\
\omega^o_4
\end{array}\right]
$$

With $A \in M_{3\times4}\mathbb R$

It will be something like:

$\begin{array}{llll}
{v_x|}_{o_1} = -\frac {\sqrt 2}{2\cdot 4} r \omega^o_1 & 
{v_x|}_{o_2} =  \frac {\sqrt 2}{2\cdot4}r \omega^o_2 & 
{v_x|}_{o_3} = -\frac {\sqrt 2}{2\cdot4}r \omega^o_3 &
{v_x|}_{o_4} =  \frac {\sqrt 2}{2\cdot4}r \omega^o_4 \\
{v_y|}_{o_1} =  \frac {\sqrt 2}{2\cdot4}r \omega^o_1 &
{v_y|}_{o_2} =  \frac {\sqrt 2}{2\cdot4}r \omega^o_2 &
{v_y|}_{o_3} = -\frac {\sqrt 2}{2\cdot4}r \omega^o_3 &
{v_y|}_{o_4} = -\frac {\sqrt 2}{2\cdot4}r \omega^o_4 \\
{\dot\theta|}_{o_1} = \frac r {4L} \omega_{o_1}&
{\dot\theta|}_{o_2} = \frac r {4L} \omega_{o_2} &
{\dot\theta|}_{o_3} = \frac r {4L} \omega_{o_3} &
{\dot\theta|}_{o_4} = \frac r {4L} \omega_{o_4} \\
\end{array}$

With $L = \frac{\sqrt 2} 2 (l+w)$

So the matrix system will be

$$
\left[\begin{array}{}
v_x\\
v_y\\
\dot\theta
\end{array}\right]
=
\left[\begin{array}{}
-\frac14r & \frac14r & -\frac14r & \frac14r\\
\frac14r & \frac14r & -\frac14r & -\frac14r\\
\frac r{2(l+w)} & \frac r{2(l+w)} & \frac r{2(l+w)} & \frac r{2(l+w)}
\end{array}\right]
\left[\begin{array}{}
\omega^m_1\\
\omega^m_2\\
\omega^m_3\\
\omega^m_4
\end{array}\right]
$$

Or, in a cleaner way

$$
\left[\begin{array}{}
v_x\\
v_y\\
\dot\theta
\end{array}\right]
=
\left[\begin{array}{}
\frac14r & 0 & 0\\
0 & \frac14r & 0\\
0 & 0 & \frac{r}{2(l+w)}
\end{array}\right]
\left[\begin{array}{rrrr}
-1 &  1 & -1 &  1 \\
 1 &  1 & -1 & -1 \\
 1 &  1 &  1 &  1
\end{array}\right]
\left[\begin{array}{}
\omega_1\\
\omega_2\\
\omega_3\\
\omega_4
\end{array}\right]
$$

---

## RPM

$[\omega] = \frac {rad} s \rightarrow \omega_{rpm} = \omega_{rad\over s}\frac{60}{2\pi} $

---

## HAHA, no, it's

$$
\left[\begin{array}{}
v_x \\ v_y \\ \dot\theta
\end{array}\right] = \frac r 4
\left[\begin{array}{cccc}
-1 &  1 & -1 &  1 \\
 1 &  1 & -1 & -1 \\
\frac 1{(l+w)} &  \frac 1{(l+w)} &  \frac 1{(l+w)} &  \frac 1{(l+w)}
\end{array}\right]
\left[\begin{array}{}
\omega_1 \\ \omega_2 \\ \omega_3 \\ \omega_4
\end{array}\right]
$$

### The inverse kinematics equation is

$$
\left[\begin{array}{}
\omega_1 \\ \omega_2 \\ \omega_3 \\ \omega_4
\end{array}\right] = \frac 1 r
\left[\begin{array}{rrr}
-1 & 1  & (l+w)\\
1  & 1  & (l+w)\\
-1 & -1 & (l+w)\\
1  & -1 & (l+w)
\end{array}\right]
\left[\begin{array}{}
v_x \\ v_y \\ \dot\theta
\end{array}\right]
$$

---

## Discrete integration
The previous equations can be used to compute direct and inverse kinematics istantanously; to perform discrete integration they hypotethically needs to remains constants during a finite period of time.

To indicate a finite time period from now on all variable parameters will have a pedix to indicate the instant to consider

---

## Euler integration
Using Euler integration it's possible to obtain position in discrete time interval approximating movements.

$$
\begin{array}{l}
x_{k+1} = x_k + T_s ({v_x}_k\cos\theta_k - {v_y}_k \sin\theta_k)\\
y_{k+1} = y_k + T_s ({v_x}_k\sin\theta_k + {v_y}_k \cos\theta_k)\\
\theta_{k+1} = \theta_k + T_s\omega_k\\
T_s = t_{k+1} - t_k
\end{array}
$$

that in matrix format is

$$
\left[\begin{array}{}
x_{k+1} \\ y_{k+1} \\ \theta_{k+1}
\end{array}\right]
=
\left[\begin{array}{}
x_k \\ y_k \\ \theta_k
\end{array}\right]
+  T_s
\left[\begin{array}{}
\cos\theta_k & -\sin\theta_k & 0\\
\sin\theta_k & \cos\theta_k & 0\\
0 & 0 & 1
\end{array}\right]
\left[\begin{array}{}
{v_x}_k \\ {v_x}_k \\ \omega_k
\end{array}\right]
$$

---

## Runge-Kutta integration
For a more precise discrete integration is possible to use Runge-Kutta integration, that also consider current angular velocity and prevent errors with angular velocity $\omega_k \sim 0 $

$$
\begin{array}{l}
x_{k+1} = x_k + T_s ({v_x}_k\cos\varphi_k - {v_y}_k \sin\varphi_k)\\
y_{k+1} = y_k + T_s ({v_x}_k\sin\varphi_k + {v_y}_k \cos\varphi_k)\\
\theta_{k+1} = \theta_k + T_s\omega_k\\
T_s = t_{k+1} - t_k
\end{array}
$$

With $\varphi_k = \theta_k + \frac{\omega_k T_s} 2$

$$
\left[\begin{array}{}
x_{k+1} \\ y_{k+1} \\ \theta_{k+1}
\end{array}\right]
=
\left[\begin{array}{}
x_k \\ y_k \\ \theta_k
\end{array}\right]
+  T_s
\left[\begin{array}{}
\cos\varphi_k & -\sin\varphi_k & 0\\
\sin\varphi_k & \cos\varphi_k & 0\\
0 & 0 & 1
\end{array}\right]
\left[\begin{array}{}
{v_x}_k \\ {v_x}_k \\ \omega_k
\end{array}\right]
$$

The associated system is not linear due to the presence of $\omega_k$ inside of the trigonometric function.

---

## From RPM to Ticks
Since we are dealing with discrete integration, it can be pretty useful to use encoder's ticks instead of wheel angular velocity.

This also allows to use ticks data from the documentation Bag, witch is hypothetically more reliable since it should have less noise than RPM data.

### Variables notation:
$T_s = t_{k+1} - t_k=$ time period of discrete integration\
$b =$ number of encoder's bit precision\
$N = 2^b = $ ticks per round, since encoders have a full rotation scale\
$\delta = \frac{2\pi} N = \frac{2\pi} {2^b} =$ angle per tick in radiant\
${\alpha_k}_i = T_s{\omega_k}_i=$ angle rotation of a wheel in a time period\
${n_k}_i =$ ticks registered from a wheel's encoder in a time period

With this premises it's possible to convert previous equations for direct kinematics

Since $T_s{\omega_k}_i = {\alpha_k}_i$ is the angle of rotation in a time period, it's possible to express ${\omega_k}_i = \frac {{\alpha_k}_i} {T_s}$;\
${\alpha_k}_i$ represent an angle so it can be expressed in ticks using the formula ${\alpha}_i = {n_k}_i\delta$

So, simplyfing

$${\omega_k}_i = \frac{{n_k}_i\delta}{T_s} = \frac{2\pi}{T_sN}{n_k}_i $$
$${n_k}_i = \frac{T_sN}{2\pi} {\omega_k}_i$$

It's possible to rewrite direct and inverse kinematics equations as:

$$
\left[\begin{array}{}
{v_x}_k \\ {v_y}_k \\ \omega_k
\end{array}\right] = \frac r 4\ \frac {2\pi}{T_sN}
\left[\begin{array}{cccc}
-1 &  1 & -1 &  1 \\
 1 &  1 & -1 & -1 \\
\frac 1{(l+w)} &  \frac 1{(l+w)} &  \frac 1{(l+w)} &  \frac 1{(l+w)}
\end{array}\right]
\left[\begin{array}{}
{n_1}_k \\ {n_2}_k \\ {n_3}_k \\ {n_4}_k
\end{array}\right]
$$

and so, the inverse kinematics equation is

$$
\left[\begin{array}{}
{n_1}_k \\ {n_2}_k \\ {n_3}_k \\ {n_4}_k
\end{array}\right] = \frac 1 r \frac{T_sN}{2\pi}
\left[\begin{array}{rrr}
-1 & 1  & (l+w)\\
1  & 1  & (l+w)\\
-1 & -1 & (l+w)\\
1  & -1 & (l+w)
\end{array}\right]
\left[\begin{array}{}
v_x \\ v_y \\ \dot\theta
\end{array}\right]
$$

This also allows to simplify equation for Euler and Runge-Kutta integration

---

## Ticks Euler integration

The new equations don't need an integration over $T_s$ so it's possible to simplify them as:

$$
\begin{array}{l}
x_{k+1} = x_k ({\Delta x}_k\cos\theta_k - {\Delta y}_k \sin\theta_k)\\
y_{k+1} = y_k ({\Delta x}_k\sin\theta_k + {\Delta y}_k \cos\theta_k)\\
\theta_{k+1} = \theta_k + {\Delta\theta}_k\\
\end{array}
$$

representable as

$$
\left[\begin{array}{}
x_{k+1} \\ y_{k+1} \\ \theta_{k+1}
\end{array}\right]
=
\left[\begin{array}{}
x_k \\ y_k \\ \theta_k
\end{array}\right] +
\left[\begin{array}{}
\cos\theta_k & -\sin\theta_k & 0\\
\sin\theta_k & \cos\theta_k & 0\\
0 & 0 & 1
\end{array}\right]
\left[\begin{array}{}
{\Delta x}_k \\ {\Delta y}_k \\ {\Delta \theta}_k
\end{array}\right]
$$

and using simplified equation for inverse kinematic

$$
\left[\begin{array}{}
{\Delta x}_k \\ {\Delta y}_k \\ {\Delta \theta}_k
\end{array}\right] = \frac {\pi r} {2N}
\left[\begin{array}{cccc}
-1 &  1 & -1 &  1 \\
 1 &  1 & -1 & -1 \\
\frac 1{(l+w)} &  \frac 1{(l+w)} &  \frac 1{(l+w)} &  \frac 1{(l+w)}
\end{array}\right]
\left[\begin{array}{}
{n_1}_k \\ {n_2}_k \\ {n_3}_k \\ {n_4}_k
\end{array}\right]
$$

that can be written as a single matrix equation as

$$
\left[\begin{array}{}
x_{k+1} \\ y_{k+1} \\ \theta_{k+1}
\end{array}\right]
=
\left[\begin{array}{}
x_k \\ y_k \\ \theta_k
\end{array}\right] +

\frac {\pi r} {2N}
\left[\begin{array}{}
\cos\theta_k & -\sin\theta_k & 0\\
\sin\theta_k & \cos\theta_k & 0\\
0 & 0 & 1
\end{array}\right]
\left[\begin{array}{cccc}
-1 &  1 & -1 &  1 \\
 1 &  1 & -1 & -1 \\
\frac 1{(l+w)} &  \frac 1{(l+w)} &  \frac 1{(l+w)} &  \frac 1{(l+w)}
\end{array}\right]
\left[\begin{array}{}
{n_1}_k \\ {n_2}_k \\ {n_3}_k \\ {n_4}_k
\end{array}\right]
$$

---

## Ticks Runge-Kutta integration
It's also possible to simplify Runge-Kutta integration equations as

$$
\begin{array}{l}
x_{k+1} = x_k ({\Delta x}_k\cos\varphi_k - {\Delta y}_k \sin\varphi_k)\\
y_{k+1} = y_k ({\Delta x}_k\sin\varphi_k + {\Delta y}_k \cos\varphi_k)\\
\theta_{k+1} = \theta_k + {\Delta\theta}_k\\
\end{array}
$$

It's also possible to express $\omega_k = \frac r 4 \frac {2\pi}{T_sN} \frac1 {(l+w)} \sum_{i=1}^4{n_i}_k$ , and so represent $\varphi_k$ as

With $\varphi_k = \theta_k + \frac{r\pi}{4N(l+w)}\sum_{i=1}^4{n_i}_k$

$$
\left[\begin{array}{}
x_{k+1} \\ y_{k+1} \\ \theta_{k+1}
\end{array}\right]
=
\left[\begin{array}{}
x_k \\ y_k \\ \theta_k
\end{array}\right] +
\left[\begin{array}{}
\cos\varphi_k & -\sin\varphi_k & 0\\
\sin\varphi_k & \cos\varphi_k & 0\\
0 & 0 & 1
\end{array}\right]
\left[\begin{array}{}
{\Delta x}_k \\ {\Delta y}_k \\ {\Delta \theta}_k
\end{array}\right]
$$

This discrete equations can be easily converted in functions from any programming language, for example C++, required for this project

----