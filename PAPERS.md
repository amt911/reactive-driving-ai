# Lo que necesito que busques

## 2026-11-12 19:28 — planificador de maniobras a baja velocidad (coches atascados)

Hay atascos que el planificador actual no puede resolver: un coche cruzado en la pista (63°) con el
morro hacia un escombro y la trasera contra el muro, y cinco más bloqueados detrás para siempre
(`docs/FINDINGS.md`, «Pile-up deadlocks»). Hace falta un planificador de maniobras (adelante y atrás)
para esos casos. Por orden de utilidad:

1. **Dolgov, Thrun, Montemerlo & Diebel**, «Path Planning for Autonomous Vehicles in Unknown
   Semi-structured Environments», *International Journal of Robotics Research* 29(5), 2010 — el A\*
   híbrido (Junior, DARPA Urban Challenge): nodos continuos en una rejilla, heurísticas, expansión
   analítica con Reeds-Shepp. Vale también la versión corta: «Practical Search Techniques in Path
   Planning for Autonomous Driving», AAAI Workshop 2008.
2. **Reeds & Shepp**, «Optimal paths for a car that goes both forwards and backwards», *Pacific
   Journal of Mathematics* 145(2), 1990 — las curvas mínimas con marcha atrás (heurística y tramo
   final del A\* híbrido).
3. **Agarwal, Schwarzer, Castro, Courville & Bellemare**, «Deep Reinforcement Learning at the Edge
   of the Statistical Precipice», *NeurIPS* 2021 (arXiv 2108.13264) — cómo comparar resultados de
   simulación con mucha varianza (intervalos por *bootstrap*); lo cito en `scripts/compare.sh`.

Déjalos en `papers/` (local, no se sube).

## 2026-11-11 — Werling 2010, Treiber 2000, Heilmeier 2019 (cerrado 2026-11-12)

Llegaron los tres y están verificados en la bibliografía. Pendiente aparte, sin papers: comparar la
trazada con el código de TUMFTM (`global_racetrajectory_optimization`) en un circuito de
`racetrack-database`.

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

**Cerrado 2026-11-12 17:40:** esos tres no se han podido conseguir; no hacen falta. GripMap 2025
(Werner et al., A2RL) hace lo mismo que `learning.rs` en un coche real: un factor de agarre por
zona que escala a la vez la aceleración longitudinal y la lateral, afinado iterando (se baja donde
los errores de control se repiten, se sube donde sobra neumático). Lo ajustan entre sesiones, así
que tampoco dan una ganancia: la de `learning.rs` sigue siendo una medida. Verificadas además
Braghin, Velenis, Mayne, Nocedal, ForzaETH, GripMap y A2RL. Después llegaron también Coulter 1992, Kapania & Gerdes 2015 (*VSD*, el del *feedforward*, no el de ILC) y Kapania 2016: verificados.

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
| Coulter 1992 | R. C. Coulter, «Implementation of the Pure Pursuit Path Tracking Algorithm», CMU-RI-TR-92-01, Carnegie Mellon University, 1992. | `control.rs`: volante por pure pursuit | 2026-11-12 (PDF) |
| Kapania & Gerdes 2015 | N. R. Kapania, J. C. Gerdes, «Design of a feedback-feedforward steering controller for accurate path tracking and stability at the limits of handling», *Vehicle System Dynamics* 53(12), 2015, DOI 10.1080/00423114.2015.1055279. | `control.rs`: *feedforward* de subviraje en el volante | 2026-11-12 (PDF) |
| Arimoto 1984 | S. Arimoto, S. Kawamura, F. Miyazaki, «Bettering operation of robots by learning», *J. Robotic Systems* 1(2), 1984. | `learning.rs`: control por aprendizaje iterativo (ILC), el error de una repetición corrige la siguiente | — |
| Kapania & Gerdes 2015b | N. R. Kapania, J. C. Gerdes, «Path tracking of highly dynamic autonomous vehicle trajectories via iterative learning control», *American Control Conference*, 2015. | `learning.rs`: ILC vuelta a vuelta en un coche de carreras | — |
| Kalaria 2024 | D. Kalaria, Q. Lin, J. M. Dolan, «Adaptive Planning and Control with Time-Varying Tire Models for Autonomous Racing Using Extreme Learning Machine», *ICRA* 2024 (arXiv 2303.08235). | Alternativa estudiada: aprender el modelo de neumático en línea; aquí basta una escala de agarre por tramo | — |
| Kapania 2016 | N. R. Kapania, J. Subosits, J. C. Gerdes, «A Sequential Two-Step Algorithm for Fast Generation of Vehicle Racing Trajectories», *ASME J. Dynamic Systems, Measurement, and Control* 138(9), 2016 (arXiv 1902.00606). | `speed_profile.rs`: pasadas hacia delante y hacia atrás | 2026-11-12 (preprint) |
| Heilmeier 2019 | A. Heilmeier, A. Wischnewski, L. Hermansdorfer, J. Betz, M. Lienkamp, B. Lohmann, «Minimum curvature trajectory planning and control for an autonomous race car», *Vehicle System Dynamics*, 2019 (en línea; 58(10), 2020), DOI 10.1080/00423114.2019.1631455. | `racing_line.rs`: objetivo de mínima curvatura; formato CSV de circuitos de TUMFTM | 2026-11-12 (PDF; volumen sin comprobar) |
| Braghin 2008 | F. Braghin, F. Cheli, S. Melzi, E. Sabbioni, «Race driver model», *Computers & Structures* 86, 2008, pp. 1503–1516. | `racing_line.rs`: trazada de mínima curvatura | 2026-11-12 (PDF) |
| Rajamani | R. Rajamani, *Vehicle Dynamics and Control*, 2.ª ed., Springer, 2012, cap. 2 (modelos de bicicleta cinemático y dinámico). | `rdai-sim/vehicle.rs`; gradiente de subviraje en `control.rs` | — |
| Velenis & Tsiotras 2005 | E. Velenis, P. Tsiotras, «Optimal velocity profile generation for given acceleration limits: theoretical analysis», *American Control Conference*, 2005. | `speed_profile.rs`: perfil de velocidad de tiempo mínimo sobre la elipse de fricción | 2026-11-12 (PDF; páginas sin comprobar) |
| Milliken 1995 | W. F. Milliken, D. L. Milliken, *Race Car Vehicle Dynamics*, SAE, 1995, cap. 2 (círculo de fricción). | `agent.rs`: frenada en curva limitada por el agarre que usa el giro (*trail braking*) | — |
| ForzaETH 2024 | N. Baumann et al., «ForzaETH Race Stack — Scaled Autonomous Head-to-Head Racing on Fully Commercial off-the-Shelf Hardware», *Journal of Field Robotics*, 2024, DOI 10.1002/rob.22429 (arXiv:2403.11784) (controlador MAP). | Candidato para sustituir pure pursuit (aún no usado) | 2026-11-12 (preprint) |
| GripMap 2025 | F. Werner, A.-K. Schwehn, M. Lienkamp, J. Betz, «GripMap: An Efficient, Spatially Resolved Constraint Framework for Offline and Online Trajectory Planning in Autonomous Racing», arXiv:2504.12115, 2025. | Coste de agarre por candidato: probado y rechazado (docs/FINDINGS.md) | 2026-11-12 (preprint) |
| A2RL 2026 | S. Hoffmann, S. Sagmeister, T. Betz, J. Bongard et al., «Head-to-Head autonomous racing at the limits of handling in the A2RL challenge», arXiv:2602.08571, 2026. | Referencia del siguiente diseño del planificador (trayectorias relativas a la trazada) | 2026-11-12 (preprint) |
| Nocedal & Wright 2006 | J. Nocedal, S. J. Wright, *Numerical Optimization*, 2.ª ed., Springer, 2006, cap. 16 (programación cuadrática, conjunto activo primal, alg. 16.3). | `min_curvature.rs`: QP de mínima curvatura con límites de pista. | No (de memoria; comprobar el número del algoritmo) |
| Mayne 2000 | D. Q. Mayne, J. B. Rawlings, C. V. Rao, P. O. M. Scokaert, «Constrained model predictive control: Stability and optimality», *Automatica* 36(6), 2000. | `local_line.rs`: condición terminal del horizonte deslizante (el final de la ventana se une al plan de vuelta). | No |
| Betz 2022 | J. Betz et al., «Autonomous Vehicles on the Edge: A Survey on Autonomous Vehicle Racing», *IEEE Open Journal of Intelligent Transportation Systems* 3, 2022. | `local_line.rs`: arquitectura de dos niveles (trazada global + planificación local). | No |
| Fan 2018 | H. Fan et al., «Baidu Apollo EM Motion Planner», arXiv:1807.08048, 2018. | `local_line.rs`: *stitching* (el nuevo plan empieza en el anterior mientras el coche lo siga). | No |
