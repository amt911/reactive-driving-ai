# reactive-driving-ai

IA de conducción **reactiva y general** escrita en Rust: coches que reaccionan en el mismo tick a
un coche que trompea delante, a escombros en la vía, a una frenada brusca o a un hueco para
adelantar. Sirve para carreras, pero no está atada a ellas. Cuesta microsegundos por agente y tick
y se puede usar desde cualquier motor (Godot, Unreal o Unity) a través de una ABI en C.

Nace como reescritura de la IA de conducción de mi TFG: un simulador de gestión de carreras en
Unreal Engine 5.1. Aquella IA usaba A* sobre una rejilla de actores, un PID ajustado con un
algoritmo genético, line traces y velocidades puestas a mano en cada checkpoint, y los coches
reaccionaban tarde. En [docs/ALGORITHMS.md](docs/ALGORITHMS.md) está la comparación completa y el
porqué de cada cambio.

## Qué hay en el repositorio

| Crate | Qué es |
| --- | --- |
| `crates/rdai-core` | La IA. Sin dependencias, determinista. Modela el circuito en coordenadas de Frenet, calcula la trazada de mínima curvatura y su perfil de velocidad, y en cada tick ejecuta un planificador local que elige una trayectoria entre un abanico de candidatas. Después aplica el control (pure pursuit y lazo de velocidad) y la personalidad del piloto (experiencia, aguante y agresividad). |
| `crates/rdai-sim` | Simulación 2D de referencia: modelo de bicicleta dinámico con elipse de fricción, varios coches, muros, contactos, rebufo y vueltas. Incluye el binario `bench`. |
| `crates/rdai-ffi` | ABI en C (`librdai.so` / `librdai.a`) con la cabecera [`include/rdai.h`](include/rdai.h). |
| `crates/rdai-viewer` | Visor 2D (macroquad) para ver a la IA conducir. |

Aún no hay ningún motor conectado. La simulación 2D hace de motor para probar la IA, y
[docs/ENGINE-INTEGRATION.md](docs/ENGINE-INTEGRATION.md) explica cómo conectarla a Godot, Unreal o
Unity.

## Cómo ejecutarlo

Hace falta Rust estable (probado con 1.99.1). En Linux, el visor necesita las bibliotecas de X11 o
Wayland y OpenGL que ya trae cualquier escritorio.

```bash
# El visor: 8 coches en el circuito por defecto
cargo run --release -p rdai-viewer

# Carrera sin gráficos: resultados y coste de la IA por tick
cargo run --release -p rdai-sim --bin bench -- --cars 8 --laps 3

# Escenarios reactivos
cargo run --release -p rdai-sim --bin bench -- --scenario crash    # un coche trompea delante
cargo run --release -p rdai-sim --bin bench -- --scenario debris   # escombros en la trazada

# Otro circuito: el óvalo de pruebas o un CSV en formato TUMFTM (x_m, y_m, w_tr_right_m, w_tr_left_m)
cargo run --release -p rdai-sim --bin bench -- --track oval
cargo run --release -p rdai-sim --bin bench -- --track mi_circuito.csv

# Depuración: traza por tick del coche 0 y la trazada calculada, en CSV
cargo run --release -p rdai-sim --bin bench -- --cars 1 --laps 2 --trace traza.csv --dump-line linea.csv

# Tests y la prueba de la ABI en C
cargo test --workspace --release
scripts/c-smoke.sh
```

## Controles del visor

| Tecla | Acción |
| --- | --- |
| Espacio | Pausa / continuar |
| ↑ / ↓ | Velocidad de la simulación ×2 / ÷2 (de ×0.125 a ×16) |
| Tab | Seleccionar el siguiente coche |
| F | Cámara que sigue al coche seleccionado, sí/no |
| P | Mostrar u ocultar las trayectorias planificadas |
| K | El coche seleccionado pierde el control (accidente) |
| O | Soltar escombros 80 m por delante del coche seleccionado, sobre su trazada |
| A / Z | Subir / bajar la agresividad del coche seleccionado |
| R | Reiniciar la carrera |
| Rueda del ratón | Zoom |
| Botón derecho + arrastrar | Desplazar la vista (desactiva el seguimiento) |

La trazada se dibuja de rojo (lenta) a verde (rápida). La línea verde que sale del coche
seleccionado es su plan en ese tick. El panel muestra las posiciones, el modo de cada coche
(`cruise`, `follow`, `avoid` o `EMERGENCY`), los golpes contra el muro, los contactos, los
adelantamientos y el coste de la IA en µs por tick.

## Estado

Primera versión. La IA completa vueltas y reacciona a los escenarios, pero aún va lenta: la mejor
vuelta es de 82.6 s frente a 63.4 s ideales y toca el muro 9 veces en 2 vueltas. Hay un fallo
abierto en la frenada del perfil de velocidad. Los fallos conocidos están en
[docs/FINDINGS.md](docs/FINDINGS.md) y los cambios por versión, en [CHANGELOG.md](CHANGELOG.md).

## Documentación

- [docs/ALGORITHMS.md](docs/ALGORITHMS.md): del TFG a este proyecto, algoritmo por algoritmo.
- [docs/ENGINE-INTEGRATION.md](docs/ENGINE-INTEGRATION.md): unidades, ejes y cómo llamarla desde
  cada motor.
- [PAPERS.md](PAPERS.md): las fuentes de cada modelo.
- [AGENTS.md](AGENTS.md): reglas de trabajo del repositorio (para personas y agentes).

## Licencia

MIT o Apache-2.0, a elección.
