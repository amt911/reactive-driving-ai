# Algoritmos: del TFG a reactive-driving-ai

Este documento compara la IA de conducción del TFG (simulador de gestión de carreras en Unreal
Engine 5.1; capítulos `implementacion.tex` y `diseno.tex` de la memoria) con la de este
repositorio. Para cada pieza explica qué hacía el TFG, qué lo sustituye y por qué. Las referencias
entre corchetes están en [PAPERS.md](../PAPERS.md).

## El problema de partida

Los coches del TFG eran **poco reactivos**. La causa no era un mal ajuste de constantes, sino la
estructura de la IA:

- La ruta se calculaba con **A\*** sobre una rejilla de actores, y solo al cruzar cada checkpoint.
  Entre dos checkpoints, el coche seguía una ruta que ya no tenía en cuenta lo ocurrido delante
  (un choque, un coche más lento). Recalcularla costaba décimas de segundo con más de dos coches,
  incluso con montículo binario.
- El volante lo movía un **PID** cuyo error era *solo* la distancia al punto más cercano de la ruta.
  Ese error no dice hacia dónde apunta el coche ni hacia dónde va la ruta, así que el PID corregía
  tarde y oscilaba. Las constantes se buscaron con un **algoritmo genético** (selección por ruleta
  y mutación en [-0.1, 0.1]) porque Ziegler-Nichols no daba buen resultado: se ajustaba el síntoma,
  no la causa.
- Para frenar ante el coche de delante se lanzaban **line traces** hacia adelante, con una longitud
  que crecía con la velocidad. Un rayo solo ve lo que tiene justo delante: no ve un coche que
  invade el carril en diagonal, no sabe si el de delante está frenando y no propone una salida.
- La **velocidad de paso** de cada curva se ponía a mano en cada checkpoint, y la frenada se
  calculaba con MRUA hasta el siguiente checkpoint. Cada circuito nuevo había que afinarlo a mano.

## Visión general del nuevo diseño

Hay tres capas, de la más lenta a la más rápida. Solo la última se ejecuta en cada tick:

1. **Fuera de línea, una vez por circuito y tipo de coche:** la trazada de mínima curvatura y su
   perfil de velocidad.
2. **En cada tick, para cada agente:** un planificador local (unos 20 caminos candidatos × 12
   muestras × los obstáculos cercanos) que elige el camino y la velocidad objetivo.
3. **En cada tick, después del planificador:** los controladores (pure pursuit para el volante y un
   lazo de velocidad con *feedforward*) y la personalidad del piloto, que solo modifica parámetros.

Todo ocurre en el mismo tick: si un coche trompea delante, el plan que sale de esa llamada ya lo
esquiva o frena. No hay que esperar a ningún checkpoint.

## Pieza a pieza

| Pieza | TFG | reactive-driving-ai | Por qué |
| --- | --- | --- | --- |
| Modelo del circuito | Rejilla de actores (nodos con estado: libre, ocupado, límite, checkpoint, guía) | Línea central muestreada con anchuras a cada lado, en coordenadas de Frenet (`s` a lo largo, `d` lateral); se carga desde CSV con formato TUMFTM | Proyectar una posición cuesta O(1) con una pista de búsqueda, sin un actor por celda. Además, la misma representación sirve para carreteras. |
| Ruta global | A\* hasta el siguiente checkpoint, recalculado al cruzarlo | Trazada de **mínima curvatura**, calculada una vez [Braghin 2008; Heilmeier 2019] por descenso de gradiente proyectado *coarse-to-fine* | El camino óptimo de un circuito no cambia: se calcula una vez en milisegundos y todos los agentes lo leen. Lo que cambia (los otros coches) lo resuelve el planificador local. |
| Velocidad de paso | Puesta a mano en cada checkpoint, más MRUA | **Perfil de velocidad** con pasadas hacia delante y hacia atrás sobre la elipse de fricción, con límite de potencia y resistencia aerodinámica [Velenis & Tsiotras 2005; Kapania et al. 2016] | Sale de la física del coche, no de un número escrito a mano: un circuito nuevo funciona sin afinarlo. |
| Reacción a obstáculos | Line traces delante, con longitud según la velocidad | **Planificador en coordenadas de Frenet** [Werling et al. 2010]: un abanico de quínticas laterales hasta distintos desplazamientos, comprobadas contra los obstáculos predichos (aceleración constante hasta pararse y deriva lateral) | Ve a todos los obstáculos cercanos, sabe si el de delante está frenando y propone un camino alternativo en el mismo cálculo. |
| Frenar ante el de delante | Rayo más largo con más velocidad | **IDM** [Treiber et al. 2000] sobre el líder del camino elegido, y modo **Emergency** (frenada máxima por el camino menos grave) cuando ningún camino es seguro | El IDM da una deceleración suave y sin choques demostrada en tráfico. Emergency cubre lo que el IDM no puede: un choque inevitable que hay que hacer lo menos grave posible. |
| Adelantar | Tres condiciones (cerca, más rápido, mismo checkpoint) que vuelven a lanzar A\* periódicamente | No es un modo aparte: es otro camino candidato. Cuando el de delante hace perder velocidad, salir de la trazada pasa a ser más barato que quedarse detrás | Adelantar, esquivar escombros y abrirse ante un trompo son el mismo problema y comparten código. |
| Volante | PID sobre la distancia al punto más cercano; constantes buscadas con un algoritmo genético | **Pure pursuit** [Coulter 1992] hacia un punto del plan a una distancia que crece con la velocidad, más compensación de subviraje como *feedforward* [Rajamani; Kapania & Gerdes] | Pure pursuit tiene en cuenta el rumbo y hacia dónde va el camino, y solo tiene un parámetro con sentido físico (la distancia de anticipación). No hace falta buscar constantes. |
| Acelerador y freno | Velocidad del checkpoint, más MRUA y agresividad | Lazo de velocidad: *feedforward* (cómo cambia la velocidad objetivo a lo largo del plan) más un término proporcional, limitado por el IDM | Con el *feedforward* la frenada empieza a tiempo; un lazo P solo reacciona cuando el error ya existe. |
| Personalidad | Experiencia, aguante y agresividad (hasta 20 km/h menos en curva sin aguante, hasta 10 km/h más con agresividad) | Las mismas tres variables, ahora traducidas a parámetros: uso del agarre, hueco con el de delante, margen lateral, apego a la trazada y ruido del volante | Mantiene el diseño del TFG, pero la personalidad cambia *cuánto* arriesga el piloto, no *cómo* decide. |
| Recuperación | Marcha atrás si no se mueve, A\* y avanzar despacio; enderezar si vuelca | Marcha atrás cuando el coche está atascado, en el propio agente | Igual que en el TFG; el vuelco lo resuelve el motor. |
| Rebufo | Impulso extra si un line trace detecta un coche delante | Menor resistencia aerodinámica en `rdai-sim` | Es física, no IA: le toca al motor. |

## Por qué no hay una red neuronal en el core

**Decidido: sin red neuronal en `rdai-core`.** Motivos:

- **Cuesta entrenarla.** Hacen falta millones de pasos de simulación por circuito y por tipo de
  coche, y volver a entrenar cada vez que cambia la física del motor.
- **No es determinista ni verificable.** Con la misma semilla, la IA tiene que dar los mismos
  resultados, y los tests de comportamiento tienen que poder fijar valores exactos. Una red no
  garantiza ninguna de las dos cosas, y su fallo no se puede leer en una traza.
- **Es difícil de ajustar.** Si un coche frena tarde en una curva, con este diseño se cambia un
  parámetro con nombre y unidades (agarre usado, margen, horizonte). Con una red, solo queda
  volver a entrenar.
- **No hace falta para reaccionar.** El planificador de Frenet con IDM y pure pursuit ya reacciona
  en un solo tick y cuesta unos pocos microsegundos.

**Opcional, en el futuro, y solo para dar "personalidad"** encima del planificador, sin
sustituirlo:

- **MPC o iLQR** como seguidor del plan, para exprimir más el límite de agarre que pure pursuit.
- **Imitation learning** a partir de vueltas de personas, para ajustar los parámetros de
  personalidad (dónde frena cada piloto o cuánto arriesga al adelantar). Las garantías de seguridad
  seguirían viniendo del planificador.

## Coste

Medido con `bench` en release (1 coche, circuito por defecto): unos **3.5 µs por agente y tick**.
El coste crece con el número de obstáculos que hay dentro del alcance de percepción (160 m), no
con el total de coches. La trazada y el perfil se calculan una vez al cargar el circuito.

## Limitaciones conocidas

Están en [FINDINGS.md](FINDINGS.md): la velocidad objetivo cae más rápido de lo que el coche puede
frenar cuando va desplazado de la trazada, y el agente frena sin motivo en una elipse a baja
velocidad.
