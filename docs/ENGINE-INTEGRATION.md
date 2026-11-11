# Integración con motores

`rdai-core` no conoce ningún motor. El motor le da el estado del coche y lo que hay alrededor, y
recibe volante, acelerador y freno. Este documento explica el contrato, las unidades y cómo
conectarla a Godot, Unreal y Unity. La ABI en C es la de [`include/rdai.h`](../include/rdai.h).

## El contrato

Una vez por circuito:

1. `rdai_track_new(puntos, n, spacing, edge_margin, &limites)` recibe la línea central con las
   anchuras a cada lado (`RdaiTrackPoint`, mismo formato que los CSV de TUMFTM) y los límites del
   coche (`RdaiVehicleLimits`). Calcula la trazada y el perfil de velocidad, lo que tarda
   milisegundos. El resultado es de **solo lectura** y se comparte entre todos los agentes.

Una vez por coche:

2. `rdai_agent_new(&geometria, &perfil, seed)` crea el agente con la geometría del coche, la
   personalidad (experiencia, aguante, agresividad) y una semilla. Con la misma semilla y las
   mismas entradas, el agente da las mismas salidas.

En cada tick y para cada coche:

3. `rdai_agent_tick(agente, circuito, &observacion, obstaculos, n, dt, &controles)` devuelve 0 si
   todo fue bien y un valor negativo si algún argumento no es válido (por ejemplo, `obstaculos`
   nulo con `n > 0`). El motor aplica los `controles` a su propio modelo de vehículo.

Al terminar, `rdai_agent_free` y `rdai_track_free`. Comprueba `rdai_abi_version()` al cargar la
biblioteca: si no coincide con `RDAI_ABI_VERSION`, la cabecera y la biblioteca no son de la misma
versión.

### Qué hay que darle

| Campo | Unidad | Qué es |
| --- | --- | --- |
| `RdaiObservation.position` | m | Centro del coche, en el plano |
| `RdaiObservation.heading` | rad | Hacia dónde apunta el morro, en sentido antihorario desde +X |
| `RdaiObservation.speed` | m/s | Velocidad **hacia delante**, negativa en marcha atrás |
| `RdaiObstacle.position`, `heading`, `speed` | m, rad, m/s | Lo mismo para cada obstáculo: otros coches, coches parados, escombros (`speed = 0`), peatones |
| `RdaiObstacle.accel` | m/s² | Aceleración longitudinal del obstáculo. **Muy negativa si está frenando o se ha estrellado.** El motor la estima como `(v_ahora - v_antes) / dt`, suavizada. Sin ella, la IA no distingue un coche que frena de uno que va a velocidad constante |
| `RdaiObstacle.length`, `width` | m | Tamaño del rectángulo del obstáculo |
| `dt` | s | Tiempo real desde el tick anterior |

Pasa solo los obstáculos que estén cerca (la IA ignora lo que está a más de 160 m). El coste del
tick crece con los obstáculos cercanos, no con el total.

### Qué devuelve

| Campo | Rango | Qué es |
| --- | --- | --- |
| `steer` | -1 … 1 | **+1 = todo a la izquierda**, relativo a `max_steer` de la geometría |
| `throttle`, `brake` | 0 … 1 | Acelerador y freno |
| `reverse` | 0 / 1 | 1 = marcha atrás (maniobra para desatascarse) |
| `mode` | 0–3 | 0 cruise, 1 follow, 2 avoid, 3 emergency; para el HUD o los logs |

## Unidades y ejes

Por dentro todo está en **SI** (metros, segundos, radianes) y en un plano **dextrógiro**: X a la
derecha, Y hacia arriba y ángulos en sentido antihorario desde +X. La IA es 2D: el motor proyecta
sobre el plano del suelo. Todas las conversiones se hacen en el envoltorio de cada motor y nunca
dentro de `rdai-core`.

| Motor | Ejes del motor | Posición → `rdai` | Rumbo → `rdai` | `steer` → motor |
| --- | --- | --- | --- | --- |
| **Unreal** | cm, levógiro, X delante, Y derecha, Z arriba | `x = X / 100`, `y = -Y / 100` | `heading = -Yaw` (en rad) | El volante de Chaos (`SetSteeringInput`) es positivo a la derecha: `-steer` |
| **Godot 3D** | m, dextrógiro, Y arriba | `x = X`, `y = -Z` | `atan2(-f.z, f.x)` con `f` el vector "delante" del coche (en `VehicleBody3D`, `+basis.z`) | `VehicleBody3D.steering` es positivo a la izquierda: `steer * max_steer` |
| **Godot 2D** | píxeles, Y hacia abajo | `x = X / ppm`, `y = -Y / ppm` (`ppm` = píxeles por metro) | `-rotation` | Según el modelo de coche propio |
| **Unity** | m, levógiro, Y arriba, Z delante | `x = X`, `y = Z` | `atan2(f.z, f.x)` con `f = transform.forward` | `WheelCollider.steerAngle` es positivo a la derecha: `-steer * max_steer` (en grados) |

Las velocidades se convierten igual que las posiciones (en Unreal, de cm/s a m/s dividiendo entre
100). Calcula el rumbo a partir del vector "delante" del coche y no de los ángulos de Euler: así
no depende del orden de rotación ni del cabeceo en las rampas.

> Las convenciones de signo de cada motor están sacadas de su documentación; **ninguna se ha
> probado todavía en un motor real**. El primer envoltorio que se escriba tiene que comprobarlas
> con un coche que gire a la izquierda en una curva a la izquierda.

## Frecuencia de llamada

- **Llama a `tick` en cada paso de física del coche**, no en cada frame de render, con el `dt` real
  de ese paso. La simulación de referencia funciona a **120 Hz**.
- La reacción tarda un tick: a 60 Hz, un coche que trompea delante se tiene en cuenta como mucho
  16.7 ms después.
- Por debajo de 60 Hz no se ha medido. Pure pursuit tolera ticks más largos porque apunta a un punto
  por delante del coche, pero el lazo de velocidad y la frenada de emergencia pierden precisión.
  Si hace falta bajar la frecuencia, mídelo antes con `bench`.
- **Coste:** unos 3.5 µs por agente y tick (`bench`, 1 coche). 20 coches a 120 Hz son unos 8 ms de
  CPU por segundo.
- **Hilos:** los agentes son independientes y el circuito es de solo lectura, así que se pueden
  repartir agentes entre hilos. Para que sea determinista, cada agente tiene que recibir siempre
  la misma lista de obstáculos en el mismo orden.

## Godot (GDExtension con godot-rust)

En Godot no hace falta la ABI en C. Con [godot-rust (gdext)](https://github.com/godot-rust/gdext) se
escribe un crate `rdai-godot` que depende de `rdai-core` directamente:

- Un recurso `RdaiTrack` que construye `rdai_core::Track` y `RacingLine` a partir de un `Curve3D` o
  de un CSV.
- Un nodo `RdaiDriver` (hijo de un `VehicleBody3D`) que en `_physics_process(delta)` lee el estado
  del coche padre, reúne los obstáculos cercanos de un grupo (`"rdai_obstacles"`), llama a
  `Agent::tick` y escribe `engine_force`, `brake` y `steering`.
- Las conversiones de ejes de la tabla anterior, solo en ese nodo.

Es la siguiente fase del proyecto (está en la lista de tareas de AGENTS.md).

## Unreal

- Compila `cargo build --release -p rdai-ffi` y enlaza `librdai.a` / `rdai.lib` (estática) o
  `librdai.so` / `rdai.dll` desde el `Build.cs` del módulo (`PublicAdditionalLibraries` e
  `PublicIncludePaths` hacia `include/`).
- Un `UActorComponent` en el `AWheeledVehiclePawn` llama a `rdai_agent_tick` en su
  `TickComponent` (grupo `TG_PrePhysics`) y pasa los resultados a
  `UChaosVehicleMovementComponent` (`SetThrottleInput`, `SetBrakeInput`, `SetSteeringInput`).
- Las conversiones de cm, mano izquierda y Z arriba de la tabla anterior.
- La línea central se puede sacar de un `USplineComponent` muestreado cada pocos metros, con las
  anchuras como metadatos de cada punto.

## Unity (P/Invoke)

```csharp
[StructLayout(LayoutKind.Sequential)] struct RdaiVec2 { public float x, y; }
[StructLayout(LayoutKind.Sequential)] struct RdaiObservation { public RdaiVec2 position; public float heading, speed; }
[StructLayout(LayoutKind.Sequential)] struct RdaiControls { public float steer, throttle, brake; public byte reverse, mode; }

[DllImport("rdai")] static extern uint rdai_abi_version();
[DllImport("rdai")] static extern int rdai_agent_tick(IntPtr agent, IntPtr track, ref RdaiObservation obs,
    RdaiObstacle[] obstacles, UIntPtr n, float dt, out RdaiControls controls);
```

- Copia `librdai.so` / `rdai.dll` / `librdai.dylib` a `Assets/Plugins/`.
- Llama a `tick` en `FixedUpdate` con `Time.fixedDeltaTime`.
- Los `struct` de C# tienen que tener el mismo orden de campos y los mismos tipos que `rdai.h`
  (`float`, `byte` para `uint8_t`, `UIntPtr` para `size_t`).
- En iOS, el plugin se enlaza estáticamente: `[DllImport("__Internal")]`.
