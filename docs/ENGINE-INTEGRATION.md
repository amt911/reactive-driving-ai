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

En Godot no hace falta la ABI en C: el crate `crates/rdai-godot` (con
[godot-rust (gdext)](https://github.com/godot-rust/gdext) 0.5) depende de `rdai-core` directamente y
registra dos clases. Funciona con Godot 4.2 o posterior (probado con 4.7.2).

- **`RdaiTrack`** (`RefCounted`): `meters_per_unit` (1.0 en un juego 3D en metros; 0.05 si 20 px son
  un metro), `build(centro: PackedVector2Array, ancho_izq: PackedFloat32Array, ancho_der:
  PackedFloat32Array) -> bool`, `ideal_lap() -> float` y `racing_line() -> PackedVector2Array` (para
  dibujarla). Se construye una vez y lo comparten todos los coches.
- **`RdaiDriver`** (`RefCounted`): `setup(track, seed, experience, stamina, aggression)` y, en cada
  `_physics_process(delta)`, `tick(posicion: Vector2, rotacion: float, velocidad: float,
  obstaculos: PackedFloat32Array, delta) -> Dictionary` con `steer`, `throttle`, `brake`, `reverse`
  y `mode`. Los obstáculos van en un array plano, 7 números por obstáculo: x, y, rotación,
  velocidad, aceleración, largo, ancho. Devuelve un diccionario vacío si no hay `setup` o si llega
  algún valor no finito.

Todo entra en las **convenciones de Godot 2D**: Y hacia abajo, `rotation` en sentido horario en
pantalla y longitudes en unidades de Godot. La clase convierte a SI. En 3D se pasa
`Vector2(pos.x, pos.z)` y, como rotación, `atan2(f.z, f.x)`, con `f` el vector "delante" del coche
(en `VehicleBody3D`, `+basis.z`). `steer = +1` gira a la izquierda del conductor, que en pantalla 2D
es el sentido antihorario: la `rotation` disminuye.

Cómo cargarla en un proyecto:

1. `cargo build --release -p rdai-godot` (genera `target/release/librdai_godot.so`, `.dylib` o `.dll`).
2. Copia [`examples/godot/rdai.gdextension`](../examples/godot/rdai.gdextension) al proyecto y
   ajusta las rutas de la biblioteca.
3. Abre el proyecto en el editor una vez (o crea `.godot/extension_list.cfg` con la línea
   `res://rdai.gdextension`).

[`examples/godot/smoke.gd`](../examples/godot/smoke.gd) es un ejemplo completo: un coche cinemático
en GDScript que da una vuelta a una elipse conducido por `RdaiDriver`. `scripts/godot-smoke.sh` lo
ejecuta en Godot sin ventana y falla si el coche se sale de la pista.

Siguiente paso: un nodo hijo de `VehicleBody3D` que lea el coche padre y los obstáculos de un grupo
y escriba `engine_force`, `brake` y `steering` por sí solo.

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
