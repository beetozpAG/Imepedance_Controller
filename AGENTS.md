# AGENTS.md — Proyecto Kinova Gen3 7DoF / Control de Impedancia

## 0. Propósito

Este archivo da contexto a una nueva sesión de ChatGPT/Codex sobre el proyecto. La prioridad es mantener la arquitectura actual, no romper la interfaz Kinova/Kortex y conservar seguridad en pruebas físicas.

## 1. Proyecto

- Robot: Kinova Gen3 normal, 7 DoF.
- Pinza: Robotiq 2F-85.
- Objetivo final: pick-and-place + peg-in-hole.
- Control: impedancia cartesiana.
- Preferencia: C++ / MATLAB / Simulink.
- Evitar ROS y MoveIt salvo necesidad explícita.
- Visión futura: RGB-D + OpenCV C++.
- No hay sensor F/T en el efector final.
- Se usan estados articulares, torques y corrientes disponibles del robot.

## 2. Restricciones críticas

No hacer sin autorización explícita:
- Cambiar `ENABLE_IMPEDANCE_HARDWARE` a `true`.
- Cambiar IP del robot.
- Cambiar modos de control.
- Aumentar límites de corriente/torque.
- Eliminar saturaciones.
- Eliminar checks NaN/Inf.
- Introducir ROS o MoveIt.
- Reemplazar la API local del controlador por la API de un paper o repositorio externo.

La API LOCAL compilable es la fuente de verdad.

## 3. Estructura aproximada

```text
Controlador_Impedancia/
├── CMakeLists.txt
├── Gen3_model.h
├── KinovaRobot/
├── Matrix/
├── DataLogging/
├── src/
│   ├── main_impedance.cpp
│   └── cartesian_impedance_controller.cpp
├── include/
│   ├── cartesian_impedance_controller.hpp
│   ├── gen3_adapter.hpp
│   └── controller_config.hpp
└── tests/
    ├── model_smoke_test.cpp
    ├── jacobian_validation_test.cpp
    └── rotational_jacobian_validation_test.cpp
```

## 4. Tipos de datos

La infraestructura del laboratorio usa una librería Matrix propia:
```cpp
vec<7>
mat<6,7>
```

Acceso típico:
```cpp
m[i][j]
```

No asumir sintaxis Eigen en esos objetos.

El controlador usa Eigen por comodidad para:
- álgebra lineal,
- cuaterniones,
- AngleAxis,
- pseudoinversa/SVD.

Alias usados:
```cpp
using Vec7E = CartesianImpedanceController::Vec7;
using Vec6E = CartesianImpedanceController::Vec6;
using Mat67E = CartesianImpedanceController::Mat67;
```

Conceptualmente:
```text
Vec7E  -> Eigen::Matrix<double,7,1>
Vec6E  -> Eigen::Matrix<double,6,1>
Mat67E -> Eigen::Matrix<double,6,7>
```

Mantener adaptadores entre Matrix y Eigen.

## 5. Modelo analítico

Archivo:
```text
Gen3_model.h
```

Funciones relevantes:
```cpp
GravityVector(q)
GetPose(q)
GetJacobian(q)
```

`robot->getPos()` devuelve las 7 posiciones articulares:

\[
q=[q_1,\dots,q_7]^T
\]

`GetPose(q)` hace cinemática directa y devuelve aproximadamente:
```text
[x, y, z, qw, qx, qy, qz]
```

El Jacobiano es:

\[
J\in\mathbb{R}^{6\times7}
\]

y:

\[
\dot{\xi}=J\dot q
\]

## 6. Controlador

Clase:
```cpp
CartesianImpedanceController
```

Objeto:
```cpp
CartesianImpedanceController controller;
```

La clase encapsula:
- stiffness,
- damping,
- referencia cartesiana,
- nullspace,
- wrench,
- filtros,
- torque anterior,
- estado interno.

Métodos conocidos de la API LOCAL actual:
```cpp
setCartesianStiffness(...)
setNullspaceStiffness(...)
setDampingFactors(...)
setFiltering(...)
setMaxTorqueDelta(...)
setWrench(...)
initialize(...)
setNullspaceReference(...)
compute(...)
```

No inventar métodos de otra versión como:
```cpp
setNumberOfJoints(...)
setStiffness(...)
initDesiredPose(...)
initNullspaceConfig(...)
applyWrench(...)
calculateCommandedTorques(...)
getPoseError(...)
getState(...)
```

## 7. Configuración

Se usa:
```cpp
struct ControllerConfig
```

porque funciona principalmente como contenedor de datos públicos.

Ejemplos:
```cpp
cfg.update_hz
cfg.stiffness
cfg.damping_factors
cfg.enable_nullspace
cfg.wrench
```

## 8. Referencia cartesiana actual

Por ahora se usa una referencia relativa a la pose inicial.

Posición:

\[
p_d=p_0+\Delta p
\]

Orientación:

\[
R_d=R_0R_\Delta
\]

Ejemplo:
```cpp
bool use_cartesian_offset = true;

Eigen::Vector3d position_offset =
    (Eigen::Vector3d()
        << 0.02, 0.00, 0.00)
    .finished();

Eigen::Vector3d orientation_offset_rpy_deg =
    (Eigen::Vector3d()
        << 0.0, 0.0, 0.0)
    .finished();
```

RPY está en grados en config por comodidad, pero se convierte a radianes antes de `Eigen::AngleAxisd`.

Más adelante se añadirán referencias absolutas para pick-and-place.

## 9. Inicialización

```cpp
controller.initialize(
    desired_position,
    desired_orientation,
    initial_q
);
```

## 10. Ley de impedancia

Error cartesiano:

\[
\Delta\xi=
\begin{bmatrix}
p-p_d\\
e_R
\end{bmatrix}
\]

Velocidad cartesiana:

\[
\dot{\xi}=J\dot q
\]

Wrench de tarea:

\[
F_{task}=-K\Delta\xi-DJ\dot q
\]

Torque de tarea:

\[
\tau_{task}=J^TF_{task}
\]

Nullspace:

\[
\tau_{null}=N[K_n(q_{n,d}-q)-D_n\dot q]
\]

con:

\[
N=I-J^T(J^T)^+
\]

Wrench externo/deseado:

\[
\tau_{ext}=J^TF_{ext}
\]

Torque total:

\[
\tau_{imp}=\tau_{task}+\tau_{null}+\tau_{ext}
\]

## 11. Orientación

Se usa:
```cpp
Eigen::Quaterniond
```

El error de orientación se obtiene mediante quaternion relativo + `Eigen::AngleAxisd`.

## 12. `compute()`

En el loop:
```cpp
const auto ctrl =
    controller.compute(
        q_eigen,
        dq_eigen,
        position,
        orientation,
        J_eigen
    );
```

Inputs:
- `q_eigen`
- `dq_eigen`
- posición EE
- orientación EE
- Jacobiano

Outputs usados:
```cpp
ctrl.tau_task
ctrl.tau_null
ctrl.tau_ext
ctrl.tau_impedance
ctrl.pose_error
```

Conceptualmente:
```cpp
struct ControllerOutput
{
    Vec7 tau_task;
    Vec7 tau_null;
    Vec7 tau_ext;
    Vec7 tau_impedance;
    Vec6 pose_error;
};
```

## 13. Shadow mode

Switch crítico:
```cpp
constexpr bool ENABLE_IMPEDANCE_HARDWARE = false;
```

Mientras sea `false`:

\[
\tau_{HW}=g(q)
\]

El controlador se calcula, monitorea y registra, pero no se aplica físicamente.

Virtualmente:

\[
\tau_{virtual}=g(q)+\tau_{imp}
\]

Físicamente:

\[
\tau_{HW}=g(q)
\]

No cambiar este switch automáticamente.

## 14. Blend de impedancia

Cuando eventualmente se habilite el hardware:

\[
\tau_{hw}=g(q)+\alpha(t)\tau_{imp}
\]

con:

\[
0\le\alpha\le1
\]

Código conceptual:
```cpp
impedance_blend +=
    dt / cfg.impedance_blend_time_s;

impedance_blend =
    std::clamp(
        impedance_blend,
        0.0,
        1.0
    );
```

El blend evita aplicar el 100% del torque de impedancia de golpe.

## 15. Torque a corriente

Se usa:
```cpp
KTGR.hadamard(tau)
```

Conceptualmente:

\[
I_{cmd}=KTGR\odot\tau_{cmd}
\]

## 16. Límites reales

Existen:
```cpp
maxCurrent
maxTau
```

`maxTau` se deriva de `maxCurrent` y `KTGR`.

Estos sí forman parte de la protección real de hardware.

## 17. Thresholds de shadow

Valores `max_shadow_*` son heurísticos de diagnóstico.

NO:
- vienen del paper,
- son límites oficiales Kinova,
- son límites físicos.

Solo sirven para warnings durante shadow mode.

## 18. Modo de control

Existe:
```cpp
enum class ControlMode
{
    CUR,
    POS,
    VEL
};
```

y puede usarse:
```cpp
using enum kinova_robot::ControlMode;
```

Los siete joints se configuran en current mode.

## 19. Hardware wrapper

Clase:
```cpp
KinovaRobotLowLevel
```

Métodos importantes:
```cpp
getPos()
getVel()
getTau()
getCur()
sendCur()
setControlMode()
```

## 20. Orden del loop

```text
1. Leer q, dq, corriente, torque
2. GetPose(q)
3. GetJacobian(q)
4. GravityVector(q)
5. Convertir Matrix -> Eigen
6. controller.compute(...)
7. Obtener outputs
8. Verificar NaN/Inf
9. Calcular tau_virtual
10. Revisar warnings
11. Elegir tau_hw
12. Aplicar límites/saturaciones
13. Convertir torque -> corriente
14. sendCur()
15. Log
```

## 21. Checks NaN/Inf

Se usa:
```cpp
.allFinite()
```

No eliminar.

## 22. Validaciones realizadas

Smoke test en `q=0`: pasó.

Pose aproximada:
```text
[0, -0.0246, 1.3613, 1, 0, 0, 0]
```

Configuración no cero probada:
```text
q [deg] = [10,30,-20,45,15,-30,20]
```

Pose aproximada:
```text
[0.741825,
 -0.00354281,
 0.981215,
 0.915474,
 -0.111015,
 0.356653,
 -0.149602]
```

Gravity aproximada:
```text
[0,
 -20.5377,
 -1.21335,
 -10.5437,
 -0.167791,
 -2.09165,
 -0.0350548] Nm
```

El Jacobiano translacional se validó por diferencias finitas con buen resultado.

## 23. Seguridad física

Antes de correr:
- Confirmar IP real.
- Workspace libre.
- E-stop accesible.
- Revisar signo de gravedad.
- Revisar orden de joints.
- Revisar corrientes.
- No eliminar saturaciones.
- Mantener shadow mode durante validación.
- Compilar correctamente NO implica seguridad física.

## 24. Roadmap

Actual:
- entender completamente el código,
- validar offsets pequeños,
- validar `pose_error`,
- validar `tau_task`,
- revisar signos,
- validar orientación,
- validar nullspace,
- validar wrench.

Después:
- referencias cartesianas absolutas,
- referencia suave `p_d(t)`,
- máquina de estados,
- pick-and-place,
- control de pinza,
- visión RGB-D,
- OpenCV C++,
- transformación cámara -> base,
- peg-in-hole,
- ajuste de rigidez direccional.

## 25. Arquitectura futura

```text
VISIÓN
  |
  v
pose objeto / fixture
  |
  v
TASK MANAGER / STATE MACHINE
  |
  +-- PREGRASP
  +-- GRASP
  +-- CLOSE
  +-- LIFT
  +-- MOVE
  +-- INSERT
  +-- RELEASE
  |
  v
pose deseada
  |
  v
CONTROLADOR DE IMPEDANCIA
  |
  v
tau_imp
  |
  v
+ gravedad
  |
  v
corriente
  |
  v
KINOVA
```

## 26. Reglas para Codex

Antes de tocar código:
1. Leer este archivo.
2. Inspeccionar archivos reales.
3. No asumir firmas de API.
4. Hacer cambios mínimos.
5. Compilar.
6. Mostrar `git diff`.
7. No activar hardware real.
8. No modificar límites sin justificación.
9. No introducir ROS/MoveIt.
10. Si un cambio toca `sendCur`, modos, límites, gravedad, torque real o `ENABLE_IMPEDANCE_HARDWARE`, tratarlo como seguridad crítica.

## 27. Resumen corto del proyecto

> El programa lee el estado articular del Kinova, usa un modelo analítico para obtener pose, Jacobiano y gravedad, calcula el error cartesiano respecto a una referencia y aplica una ley de impedancia tipo resorte-amortiguador. El wrench cartesiano se transforma a torques articulares mediante la transpuesta del Jacobiano. Además puede añadir nullspace y wrench deseado. Actualmente el controlador se valida en shadow mode: se calcula y registra todo el torque de impedancia, pero el hardware recibe únicamente compensación de gravedad.
