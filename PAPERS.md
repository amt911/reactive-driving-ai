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
| Werling 2010 | M. Werling, J. Ziegler, S. Kammel, S. Thrun, «Optimal Trajectory Generation for Dynamic Street Scenarios in a Frenét Frame», *IEEE ICRA*, 2010. | `planner.rs`: abanico de quínticas laterales en coordenadas de Frenet, coste por suavidad y desplazamiento, consistencia entre ticks | — |
| Treiber 2000 | M. Treiber, A. Hennecke, D. Helbing, «Congested traffic states in empirical observations and microscopic simulations», *Physical Review E* 62(2), 2000. | `planner.rs` paso 5: término de interacción del IDM (`s*`, hueco deseado) | — |
| Coulter 1992 | R. C. Coulter, «Implementation of the Pure Pursuit Path Tracking Algorithm», CMU-RI-TR-92-01, Carnegie Mellon University, 1992. | `control.rs`: volante por pure pursuit | — |
| Kapania & Gerdes 2015 | N. R. Kapania, J. C. Gerdes, «Design of a feedback-feedforward steering controller for accurate path tracking and stability at the limits of handling», *Vehicle System Dynamics* 53(12), 2015. | `control.rs`: *feedforward* de subviraje en el volante | — |
| Kapania 2016 | N. R. Kapania, J. Subosits, J. C. Gerdes, «A Sequential Two-Step Algorithm for Fast Generation of Vehicle Racing Trajectories», *ASME J. Dynamic Systems, Measurement, and Control* 138(9), 2016. | `speed_profile.rs`: pasadas hacia delante y hacia atrás | — |
| Heilmeier 2019 | A. Heilmeier, A. Wischnewski, L. Hermansdorfer, J. Betz, M. Lienkamp, B. Lohmann, «Minimum curvature trajectory planning and control for an autonomous race car», *Vehicle System Dynamics*, 2019 (en línea; 58(10), 2020). | `racing_line.rs`: objetivo de mínima curvatura; formato CSV de circuitos de TUMFTM | — |
| Braghin 2008 | F. Braghin, F. Cheli, S. Melzi, E. Sabbioni, «Race driver model», *Computers & Structures* 86, 2008. | `racing_line.rs`: trazada de mínima curvatura | — |
| Rajamani | R. Rajamani, *Vehicle Dynamics and Control*, 2.ª ed., Springer, 2012, cap. 2 (modelos de bicicleta cinemático y dinámico). | `rdai-sim/vehicle.rs`; gradiente de subviraje en `control.rs` | — |
| Velenis & Tsiotras 2005 | E. Velenis, P. Tsiotras, «Optimal velocity profile generation for given acceleration limits: theoretical analysis», *American Control Conference*, 2005. | `speed_profile.rs`: perfil de velocidad de tiempo mínimo sobre la elipse de fricción | — |
| Milliken 1995 | W. F. Milliken, D. L. Milliken, *Race Car Vehicle Dynamics*, SAE, 1995, cap. 2 (círculo de fricción). | `agent.rs`: frenada en curva limitada por el agarre que usa el giro (*trail braking*) | — |
| ForzaETH 2024 | N. Baumann et al., «ForzaETH Race Stack — Scaled Autonomous Head-to-Head Racing on Fully Commercial off-the-Shelf Hardware», arXiv:2403.11784, 2024 (controlador MAP). | Candidato para sustituir pure pursuit (aún no usado) | — |
| GripMap 2025 | «GripMap: An Efficient, Spatially Resolved Constraint Framework for Offline and Online Trajectory Planning in Autonomous Racing», arXiv:2504.12115, 2025. | Coste de agarre por candidato: probado y rechazado (docs/FINDINGS.md) | — |
| A2RL 2026 | «Head-to-Head autonomous racing at the limits of handling in the A2RL challenge», arXiv:2602.08571, 2026. | Referencia del siguiente diseño del planificador (trayectorias relativas a la trazada) | — |

## Lo que necesito que busques

El bloque más reciente va primero, ordenado por lo que más ayudaría.

### 2026-11-11

1. **Los PDF de Werling 2010 y Treiber 2000**, para citar ecuaciones concretas en `planner.rs` y
   comprobar que el término de consistencia entre ticks (empezar desde el plan anterior) es el del
   artículo. Busca el título exacto.
2. **Heilmeier 2019** (versión en abierto en el repositorio de TUM) y su código
   `TUMFTM/global_racetrajectory_optimization`, para comparar la trazada y la vuelta ideal con un
   circuito de `TUMFTM/racetrack-database`.
