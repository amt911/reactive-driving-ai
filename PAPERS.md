# Lo que necesito que busques

## 2026-11-12 16:48 — aprendizaje del agarre vuelta a vuelta (fase 2, `learning.rs`)

El aprendizaje funciona (coche con 10 % menos de agarre: golpea solo en la vuelta 1), pero la
ganancia (0,3) está medida en un solo escenario. Para fijarla con fundamento y comprobar las citas
necesito el PDF de estos, que son de pago (IEEE):

1. **Kapania & Gerdes**, «Path tracking of highly dynamic autonomous vehicle trajectories via
   iterative learning control», *American Control Conference* 2015 — la regla de aprendizaje
   vuelta a vuelta que cito (ganancia, filtro, estabilidad). Lo más útil.
2. **Hsu, Laws & Gerdes**, «Estimation of tire slip angle and friction limits using steering
   torque», *IEEE Trans. Control Systems Technology* 18(4), 2010 — estimar el agarre real a partir
   de lo que hace el coche. Es la alternativa a mi señal actual («cuánto se abre de su línea»):
   medir la aceleración lateral conseguida frente a la pedida.
3. **Arimoto, Kawamura & Miyazaki**, «Bettering operation of robots by learning», *J. Robotic
   Systems* 1(2), 1984 — el origen del ILC, solo para verificar la cita.

Déjalos en `papers/` (local, no se sube). Los de arXiv ya los leo yo.

**Actualización 2026-11-12 17:10:** en `papers/` llegaron Treiber 2000, Heilmeier 2019 y Werling
2010 (gracias: con ellos he verificado esas tres citas). **Siguen pendientes los tres de arriba.**

# Fuentes

Cada modelo y cada constante de `rdai-core` y `rdai-sim` cita aquí su fuente, con la ecuación o
sección en el comentario del código (reglas en
[docs/agents/numerics-and-sources.md](docs/agents/numerics-and-sources.md)).

> **Sin verificar (2026-11-11):** las referencias se escribieron de memoria. Los autores, títulos y
> años son fiables, pero los volúmenes y las páginas hay que comprobarlos con el PDF antes de
> citarlos en el código con número de ecuación. Al comprobar una, marca su columna «Verificada»
> con la fecha.

## Bibliografía

| Clave | Referencia | Dónde se usa | Verificada |
| --- | --- | --- | --- |
| Werling 2010 | M. Werling, J. Ziegler, S. Kammel, S. Thrun, «Optimal Trajectory Generation for Dynamic Street Scenarios in a Frenét Frame», *IEEE ICRA*, 2010, pp. 987–993. | `planner.rs`: abanico de quínticas laterales en coordenadas de Frenet, coste por suavidad y desplazamiento, consistencia entre ticks | 2026-11-12 (PDF) |
| Treiber 2000 | M. Treiber, A. Hennecke, D. Helbing, «Congested traffic states in empirical observations and microscopic simulations», *Physical Review E* 62(2), 2000 (arXiv cond-mat/0002177). | `planner.rs` paso 5: término de interacción del IDM, `−a (s*/s)²` con `s* = s0 + vT + vΔv/(2√(ab))` (el término `s1√(v/v0)` del artículo a 0) | 2026-11-12 (preprint; volumen sin comprobar) |
| Coulter 1992 | R. C. Coulter, «Implementation of the Pure Pursuit Path Tracking Algorithm», CMU-RI-TR-92-01, Carnegie Mellon University, 1992. | `control.rs`: volante por pure pursuit | — |
| Kapania & Gerdes 2015 | N. R. Kapania, J. C. Gerdes, «Design of a feedback-feedforward steering controller for accurate path tracking and stability at the limits of handling», *Vehicle System Dynamics* 53(12), 2015. | `control.rs`: *feedforward* de subviraje en el volante | — |
| Arimoto 1984 | S. Arimoto, S. Kawamura, F. Miyazaki, «Bettering operation of robots by learning», *J. Robotic Systems* 1(2), 1984. | `learning.rs`: control por aprendizaje iterativo (ILC), el error de una repetición corrige la siguiente | — |
| Kapania & Gerdes 2015b | N. R. Kapania, J. C. Gerdes, «Path tracking of highly dynamic autonomous vehicle trajectories via iterative learning control», *American Control Conference*, 2015. | `learning.rs`: ILC vuelta a vuelta en un coche de carreras | — |
| Kalaria 2024 | D. Kalaria, Q. Lin, J. M. Dolan, «Adaptive Planning and Control with Time-Varying Tire Models for Autonomous Racing Using Extreme Learning Machine», *ICRA* 2024 (arXiv 2303.08235). | Alternativa estudiada: aprender el modelo de neumático en línea; aquí basta una escala de agarre por tramo | — |
| Kapania 2016 | N. R. Kapania, J. Subosits, J. C. Gerdes, «A Sequential Two-Step Algorithm for Fast Generation of Vehicle Racing Trajectories», *ASME J. Dynamic Systems, Measurement, and Control* 138(9), 2016. | `speed_profile.rs`: pasadas hacia delante y hacia atrás | — |
| Heilmeier 2019 | A. Heilmeier, A. Wischnewski, L. Hermansdorfer, J. Betz, M. Lienkamp, B. Lohmann, «Minimum curvature trajectory planning and control for an autonomous race car», *Vehicle System Dynamics*, 2019 (en línea; 58(10), 2020), DOI 10.1080/00423114.2019.1631455. | `racing_line.rs`: objetivo de mínima curvatura; formato CSV de circuitos de TUMFTM | 2026-11-12 (PDF; volumen sin comprobar) |
| Braghin 2008 | F. Braghin, F. Cheli, S. Melzi, E. Sabbioni, «Race driver model», *Computers & Structures* 86, 2008. | `racing_line.rs`: trazada de mínima curvatura | — |
| Rajamani | R. Rajamani, *Vehicle Dynamics and Control*, 2.ª ed., Springer, 2012, cap. 2 (modelos de bicicleta cinemático y dinámico). | `rdai-sim/vehicle.rs`; gradiente de subviraje en `control.rs` | — |
| Velenis & Tsiotras 2005 | E. Velenis, P. Tsiotras, «Optimal velocity profile generation for given acceleration limits: theoretical analysis», *American Control Conference*, 2005. | `speed_profile.rs`: perfil de velocidad de tiempo mínimo sobre la elipse de fricción | — |
| Milliken 1995 | W. F. Milliken, D. L. Milliken, *Race Car Vehicle Dynamics*, SAE, 1995, cap. 2 (círculo de fricción). | `agent.rs`: frenada en curva limitada por el agarre que usa el giro (*trail braking*) | — |
| ForzaETH 2024 | N. Baumann et al., «ForzaETH Race Stack — Scaled Autonomous Head-to-Head Racing on Fully Commercial off-the-Shelf Hardware», arXiv:2403.11784, 2024 (controlador MAP). | Candidato para sustituir pure pursuit (aún no usado) | — |
| GripMap 2025 | «GripMap: An Efficient, Spatially Resolved Constraint Framework for Offline and Online Trajectory Planning in Autonomous Racing», arXiv:2504.12115, 2025. | Coste de agarre por candidato: probado y rechazado (docs/FINDINGS.md) | — |
| A2RL 2026 | «Head-to-Head autonomous racing at the limits of handling in the A2RL challenge», arXiv:2602.08571, 2026. | Referencia del siguiente diseño del planificador (trayectorias relativas a la trazada) | — |
| Nocedal & Wright 2006 | J. Nocedal, S. J. Wright, *Numerical Optimization*, 2.ª ed., Springer, 2006, cap. 16 (programación cuadrática, conjunto activo primal, alg. 16.3). | `min_curvature.rs`: QP de mínima curvatura con límites de pista. | No (de memoria; comprobar el número del algoritmo) |
| Mayne 2000 | D. Q. Mayne, J. B. Rawlings, C. V. Rao, P. O. M. Scokaert, «Constrained model predictive control: Stability and optimality», *Automatica* 36(6), 2000. | `local_line.rs`: condición terminal del horizonte deslizante (el final de la ventana se une al plan de vuelta). | No |
| Betz 2022 | J. Betz et al., «Autonomous Vehicles on the Edge: A Survey on Autonomous Vehicle Racing», *IEEE Open Journal of Intelligent Transportation Systems* 3, 2022. | `local_line.rs`: arquitectura de dos niveles (trazada global + planificación local). | No |
| Fan 2018 | H. Fan et al., «Baidu Apollo EM Motion Planner», arXiv:1807.08048, 2018. | `local_line.rs`: *stitching* (el nuevo plan empieza en el anterior mientras el coche lo siga). | No |

## Lo que necesito que busques

El bloque más reciente va primero, ordenado por lo que más ayudaría.

### 2026-11-11

1. **Los PDF de Werling 2010 y Treiber 2000**, para citar ecuaciones concretas en `planner.rs` y
   comprobar que el término de consistencia entre ticks (empezar desde el plan anterior) es el del
   artículo. Busca el título exacto.
2. **Heilmeier 2019** (versión en abierto en el repositorio de TUM) y su código
   `TUMFTM/global_racetrajectory_optimization`, para comparar la trazada y la vuelta ideal con un
   circuito de `TUMFTM/racetrack-database`.
