# TP2 — Navegación autónoma en laberinto
## Documentación técnica de la solución

Este documento describe la arquitectura, las decisiones de diseño y el procedimiento
de reproducción de la solución entregada. Las instrucciones de compilación y ejecución
están en `README.md`; acá se explica **qué hace cada pieza y por qué**.

---

## 1. Reproducción desde el archivo .zip

El `README.md` describe el flujo partiendo de un `git clone`. Si se parte del **.zip
entregado**, los pasos 1 y 2 se reemplazan por:

```bash
# 1. Crear el workspace
mkdir -p ~/ros2_ws/src

# 2. Descomprimir el entregable y mover los paquetes a src/
unzip TP2_entrega.zip
mv TP2_grupo ~/ros2_ws/src/
```

A partir del paso 3 del `README.md` el procedimiento es idéntico: `source`,
`rosdep install`, `colcon build`, `source install/setup.bash` y el `ros2 launch`.

Requisitos: **ROS 2 Jazzy** y **Gazebo Harmonic**.

Un único comando levanta la solución completa:

```bash
ros2 launch my_robot_bringup bringup.launch.xml
```

---

## 2. Arquitectura

El launch `bringup.launch.xml` levanta siete procesos:

| # | Nodo | Paquete | Función |
|---|------|---------|---------|
| 1 | `robot_state_publisher` | `robot_state_publisher` | Procesa el Xacro y publica el TF interno del robot |
| 2 | `gz sim` | `ros_gz_sim` | Simulador, carga `maze_2.world` |
| 3 | `create` | `ros_gz_sim` | Spawnea el robot en (4.500, −4.050), orientación 180° |
| 4 | `static_transform_publisher` | `tf2_ros` | Publica la transformada estática `world → odom` |
| 5 | `parameter_bridge` | `ros_gz_bridge` | Puente de tópicos Gazebo ↔ ROS 2 |
| 6 | `rviz2` | `rviz2` | Visualización |
| 7 | `ekf_node` | `robot_localization` | Fusión sensorial ruedas + IMU |
| 8 | `occupancy_grid` | `my_robot_navigation` | Publica la grilla del recorrido |
| 9 | `wall_following` | `my_robot_navigation` | Nodo de navegación (lógica principal) |

La posición de spawn está parametrizada (`spawn_x`, `spawn_y`) y es reutilizada por
el `static_transform_publisher`, de modo que un cambio de spawn no rompe el árbol de
transformadas.

---

## 3. Cadena de odometría

El sistema maneja **tres** fuentes de pose, deliberadamente separadas y con nombres
que declaran qué son:

| Tópico | Origen | Naturaleza |
|--------|--------|------------|
| `/wheel_odom` | Plugin `DiffDrive` de Gazebo | Odometría de ruedas. Deriva por patinaje. |
| `/imu` | Plugin IMU del Xacro | Aceleraciones y velocidad angular. |
| `/real_odom` | Plugin `OdometryPublisher` (30 Hz) | *Ground truth*: pose verdadera provista por el simulador. |
| `/odometry/filtered` | `ekf_node` (`robot_localization`) | Fusión de `/wheel_odom` + `/imu`. |

### Configuración del EKF (`config/ekf.yaml`)

La decisión central del filtro es **qué se fusiona de cada fuente**:

- **De las ruedas (`odom0: /wheel_odom`)** se toman únicamente las **velocidades
  lineales** (`vx`, `vy`). No se toma la pose: la pose de las ruedas es la integración
  de esas mismas velocidades, y es exactamente la magnitud que acumula deriva por
  patinaje. Se fusiona lo medido, no lo derivado. `vy = 0` es además un ancla física:
  un robot diferencial no desliza lateralmente.
- **De la IMU (`imu0: /imu`)** se toma la **orientación (yaw)** y la velocidad
  angular. El giróscopo no depende del contacto rueda–piso, por lo que su estimación
  angular no se degrada cuando las ruedas patinan.

El EKF trabaja con `world_frame: odom` y publica la transformada `odom → base_footprint`.

### Árbol de transformadas

```
world ──(static: spawn_x, spawn_y)──> odom ──(EKF)──> base_footprint ──(URDF)──> links
```

La transformada `world → odom` es **estática** y no es una simplificación: el frame
`odom` no deriva respecto del mundo. Lo que deriva es la estimación de la pose del
robot *dentro* de `odom`. Esta separación es la misma que utiliza la pila de
navegación estándar de ROS 2, donde la corrección dinámica se aplica sobre `map → odom`
y nunca sobre la odometría misma.

Para evitar que el plugin `DiffDrive` compita con el EKF publicando la misma arista
`odom → base_footprint`, su TF se redirige a un tópico interno de Gazebo
(`<tf_topic>/gazebo_internal/tf</tf_topic>`) que el puente no cruza a ROS 2.

### Valor de `/real_odom`

Al disponer simultáneamente de la pose verdadera y de la estimada, la diferencia entre
`/real_odom` y `/odometry/filtered` **es**, por definición, el error de estimación del
EKF, y resulta medible en cualquier instante de la corrida. Es una herramienta de
diagnóstico, no una entrada de la navegación: en un robot físico `/real_odom` no existe.

---

## 4. Nodo `occupancy_grid.py`

### Qué es y qué no es

Publica un mensaje `nav_msgs/OccupancyGrid` en el tópico `/grid` que representa
**el recorrido efectuado por el robot**. Es una **capa de visualización de
trayectoria**, no un mapa del entorno:

- No consume el LIDAR ni ningún sensor exteroceptivo.
- No registra la posición de las paredes ni distingue espacio libre de ocupado.
- Su única entrada es `/real_odom` (pose verdadera del simulador).

Se emplea el tipo de mensaje `OccupancyGrid` porque RViz lo renderiza nativamente como
una capa de celdas alineada con el árbol de TF, lo que permite superponer el trazo del
recorrido con el modelo del robot en una sola vista.

### Configuración

| Parámetro | Valor | Significado |
|-----------|-------|-------------|
| `resolution` | 0.2 | Metros por celda |
| `width` × `height` | 200 × 200 celdas | Cobertura de **40 m × 40 m** |
| `origin` | (−20.0, −20.0) | Origen centrado: el mapa cubre de −20 a +20 m en ambos ejes |
| `frame_id` | `world` | Requiere la transformada estática `world → odom` |
| Frecuencia de publicación | 1 Hz | Timer independiente del callback de odometría |

El origen se calcula como `−(width × resolution) / 2`, de modo que el centro del mapa
coincide con el origen del mundo y la cobertura es simétrica.

### Funcionamiento

1. **`odom_callback`**: recibe la pose, la convierte de metros a índices de celda
   mediante `floor((x − origin_x) / resolution)`, verifica que el índice caiga dentro
   de los límites de la grilla, calcula el índice del arreglo unidimensional en orden
   *row-major* (`index = grid_y × width + grid_x`) y marca la celda con el valor `100`.
2. **`publish_grid`**: cada 1 s ensambla el mensaje completo con su metadata y lo publica.

Las celdas nunca visitadas conservan el valor `-1` (desconocido), por lo que RViz las
renderiza transparentes y solo se dibuja el trazo recorrido.

### QoS

El publicador utiliza `DurabilityPolicy.TRANSIENT_LOCAL` con `depth = 1`. Esto hace que
el último mapa publicado quede retenido y se entregue automáticamente a cualquier
suscriptor que se conecte con posterioridad. Sin esta política, abrir RViz una vez
iniciada la simulación mostraría una vista vacía hasta el siguiente tick del timer, y
todo el recorrido previo se perdería visualmente.

### Nota sobre la semántica del valor 100

En la convención de `OccupancyGrid`, `-1` es desconocido, `0` es libre y `100` es
ocupado. Este nodo emplea `100` con el significado de *celda visitada*. Es un uso
deliberado del canal de valor para representar recorrido en lugar de ocupación, y es
consistente con el propósito del nodo: el mapa responde "¿por dónde pasé?", no
"¿dónde hay obstáculos?".

---

## 5. Nodo `wall_following.py`

Contiene la lógica de navegación. Suscribe a `/scan` (LIDAR) y a `/odometry/filtered`
(salida del EKF), y publica en `/cmd_vel`.

La elección de `/odometry/filtered` sobre la odometría cruda es deliberada: toda la
lógica de giros y alineación del nodo depende del yaw, y el yaw de `/odometry/filtered`
está anclado por el giróscopo de la IMU, que no se degrada con el patinaje de las ruedas.

### Máquina de estados

| Estado | Función |
|--------|---------|
| `AVANZAR` | Avance con control de centrado entre paredes |
| `AVANCE_PREVIO_DER` / `AVANCE_PREVIO_IZQ` | Reposicionamiento previo al giro |
| `GIRO_DER` / `GIRO_IZQ` | Giro de 90° por consigna angular |
| `POST_GIRO` | Enderezado geométrico y resincronización cardinal |
| `VERIFICAR_U` | Confirmación de callejón sin salida |
| `GIRO_U` | Giro de 180° |

### Regla de la mano derecha

La prioridad de decisión es **derecha → recto → izquierda → giro en U**. Es la regla
clásica de resolución de laberintos y garantiza la exploración completa de cualquier
componente de pared conectado al punto de partida.

### Resincronización cardinal

El nodo mantiene un `yaw_referencia` capturado en la primera lectura de odometría y
define las cuatro direcciones cardinales relativas a él. Tras cada giro, en el estado
`POST_GIRO`, se aplica:

```python
yaw_referencia = normalizar_angulo(yaw_referencia - error_a_cardinal())
```

El efecto es que **el error de odometría acumulado se anula en cada esquina** en lugar
de propagarse a lo largo de toda la corrida. Sin este mecanismo, un error angular de
pocos grados por giro se compone tras decenas de giros hasta desalinear por completo el
marco de referencia. En el log de la corrida final el error a cardinal se mantiene
acotado en el orden de 1 a 4 grados a lo largo de los 19 minutos de recorrido.

Conceptualmente, este mecanismo cumple la misma función que la corrección `map → odom`
de la pila de navegación estándar: una referencia externa y absoluta (en este caso, la
geometría de las paredes reales, medida por el LIDAR) corrige periódicamente la deriva
de la odometría, sin modificar la odometría misma.

### Alineación geométrica en `POST_GIRO`

Se comparan **dos rayos del LIDAR por lado**. Si ambos rayos de un mismo lado devuelven
la misma distancia, el robot está paralelo a esa pared; la diferencia entre ambos da
simultáneamente la magnitud del desalineamiento y el signo de la corrección. El término
de centrado se aplica **solo cuando ambos lados detectan pared**, dado que centrarse
respecto de un único muro carece de sentido geométrico.

Esta alineación contra la pared **real** es lo que convierte la resincronización
cardinal en una corrección genuina: la referencia no proviene de la odometría, sino de
una medición directa del entorno.

### Limitación topológica y árbitro de Trémaux

La regla de la mano derecha presenta una limitación demostrable: **solo explora el
componente de pared conectado al punto de partida**. Si el laberinto contiene una
"isla" —un bloque de paredes no conectado al perímetro—, el robot la circunvala
indefinidamente sin poder escapar. No se trata de un problema de tiempo ni de ajuste
de parámetros: es una imposibilidad topológica, y se verificó empíricamente en
`maze_2.world`.

La solución implementada superpone una **grilla de celdas visitadas** al estilo de
Trémaux. El nodo registra cuántas veces pasó por cada celda y, cuando la regla de la
mano derecha propone una dirección ya transitada, un **árbitro de menor cantidad de
visitas** desvía la decisión hacia la alternativa menos explorada:

```
Arbitro: DER tiene 2 visitas; voy IZQ (0)
```

La grilla emplea celdas de **0.45 m**, que no es un valor arbitrario: es la constante
de red del laberinto, dado que las paredes de `maze_2.world` están dispuestas en
múltiplos de 0.45 m. La discretización usa `round(pos / 0.45)` en lugar de `floor()`,
de modo que los centros de las celdas de la grilla coinciden con los centros de las
celdas reales del laberinto en vez de quedar desfasados media celda. La detección de
meta emplea un radio de 0.15 m, menor que media celda (0.225 m) y mayor que el radio
del disco objetivo.

Esta grilla es **interna al nodo de navegación** y se alimenta de `/odometry/filtered`
(la pose estimada). Es la entrada real del proceso de decisión, y es independiente de
la grilla de visualización publicada por `occupancy_grid.py`, que se alimenta de
`/real_odom` y no participa de la navegación.

---

## 6. Resultados de la corrida final

Ejecución completa sobre `maze_2.world`, desde el spawn hasta la meta:

| Métrica | Valor |
|---------|-------|
| Distancia recorrida | 143.11 m |
| Tiempo | 1157.3 s (19.3 min) |
| Celdas exploradas | 207 |
| Colisiones | 0 |
| Detención | Autónoma, por detección de meta |

El robot detecta la meta por coordenada conocida transformada al marco de odometría,
con un radio de tolerancia de 0.15 m, y se detiene por sí mismo.

---

## 7. Limitaciones conocidas

- **`occupancy_grid.py` depende de `/real_odom`**, que es información privilegiada del
  simulador. En un robot físico este tópico no existe: la capa de visualización debería
  alimentarse de `/odometry/filtered` y asumir la deriva correspondiente.
- **La grilla publicada no registra obstáculos.** Un mapa del entorno requiere
  incorporar el LIDAR y marcar tanto la celda del impacto (ocupada) como las celdas
  intermedias del rayo (libres).
- **Las dos grillas tienen resoluciones distintas y propósitos distintos**: 0.2 m la de
  visualización (arbitraria, elegida por resolución visual) y 0.45 m la de Trémaux
  (impuesta por la geometría del laberinto). Son estructuras independientes.
- **El plugin `DiffDrive` no publica su TF por configuración de tópico**, no por
  desactivación explícita. Si se habilitara el puente del tópico `/tf` de Gazebo,
  aparecerían dos publicadores compitiendo por la arista `odom → base_footprint`.
