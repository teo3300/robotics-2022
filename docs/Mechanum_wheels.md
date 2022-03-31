# Studying the Mechanum wheel robot equations

Variable notation:

$r =$ wheel radius\
$\pm l =$ wheel position along $x$\
$\pm w = $ wheel position along $y$\
$T =$ gear ratio

Robot specific parameters:
Parameter | Value
-:|-
$r$ | $0.07 $ m
$l$ | $0.200$ m
$w$ | $0.169$ m
$T$ | $5:1  $

## Hypothesis
4 Mechanum wheels behave like 4 omni-directional wheels perpendicular one another, with an angular velocity of the mechanum wheel $\omega_m$ with a radius $r_m$ and an angular velocity of the omni-directional wheel $\omega_o$ with a radius $r_o$ following the formula:

$\omega_m r_m = \sin\left(\frac\pi{4}\right) \omega_o r_o \Rightarrow \omega_m r_m = \frac {\sqrt 2} 2 \omega_o r_o$

With $r_o = r_m = r$

With a positive angular velocity meaning the actuator is rotating counter-clockwise.

Kinematic equations in matrix form will be:

$$
\left[\begin{array}{}
V_x\\
V_y\\
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
{V_x|}_{o_1} = -\frac {\sqrt 2}{2\cdot 4} r \omega^o_1 & 
{V_x|}_{o_2} =  \frac {\sqrt 2}{2\cdot4}r \omega^o_2 & 
{V_x|}_{o_3} = -\frac {\sqrt 2}{2\cdot4}r \omega^o_3 &
{V_x|}_{o_4} =  \frac {\sqrt 2}{2\cdot4}r \omega^o_4 \\
{V_y|}_{o_1} =  \frac {\sqrt 2}{2\cdot4}r \omega^o_1 &
{V_y|}_{o_2} =  \frac {\sqrt 2}{2\cdot4}r \omega^o_2 &
{V_y|}_{o_3} = -\frac {\sqrt 2}{2\cdot4}r \omega^o_3 &
{V_y|}_{o_4} = -\frac {\sqrt 2}{2\cdot4}r \omega^o_4 \\
{\dot\theta|}_{o_1} = \frac r {4L} \omega_{o_1}&
{\dot\theta|}_{o_2} = \frac r {4L} \omega_{o_2} &
{\dot\theta|}_{o_3} = \frac r {4L} \omega_{o_3} &
{\dot\theta|}_{o_4} = \frac r {4L} \omega_{o_4} \\
\end{array}$

With $L = \frac{\sqrt 2} 2 (l+w)$

So the matrix system will be

$$
\left[\begin{array}{}
V_x\\
V_y\\
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
V_x\\
V_y\\
\dot\theta
\end{array}\right]
=
\left[\begin{array}{}
\frac14r\\
\frac14r\\
\frac{r}{2(l+w)}
\end{array}\right]
\left[\begin{array}{rrrr}
-1 & 1 & -1 & 1 \\
1 & 1 & -1 & -1 \\
1 & 1 & 1 & 1
\end{array}\right]
\left[\begin{array}{}
\omega_1\\
\omega_2\\
\omega_3\\
\omega_4
\end{array}\right]
$$