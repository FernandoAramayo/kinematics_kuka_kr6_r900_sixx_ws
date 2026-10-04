# Cinemática del KUKA KR 6 R900 sixx

## 1. Información General
* **Proyecto:** Implementación de Cinemática Directa e Inversa
* **Robot:** KUKA KR 6 R900 sixx
* **Autores:**\
   Fernando Aramayo\
   Cristina Montaño

## 2. Software y Versiones Requeridas
* **Sistema Operativo:** Ubuntu 24.04 LTS
* **Middleware de ROS:** ROS 2 Jazzy Jalisco
* **Lenguaje:** Python 3 (con librería `numpy`)

## 3. Instalación y Configuración del Workspace

El repositorio incluye un script automatizado que instala las dependencias de Ubuntu, recupera el paquete oficial del robot (a través de `vcs import`), descarga paquetes de ROS (`rosdep`) y realiza la compilación.

**Instrucciones de clonación e instalación:**
Abre una terminal y ejecuta secuencialmente:

```bash
# Clonar este repositorio
git clone https://github.com/FernandoAramayo/kinematics_kuka_kr6_r900_sixx_ws
cd kinematics_kuka_kr6_r900_sixx_ws

# Dar permisos de ejecución e instalar 
chmod +x instalar.sh abrir.sh recompilar.sh verificar.sh
./instalar.sh
```

**Carga del entorno:**
Para inicializar el workspace en cualquier terminal nueva, debes cargar ROS 2, configurar CycloneDDS y hacer un *source* a la instalación local. Puedes ejecutar el script `./entorno.sh` o realizarlo manualmente:

```bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source install/setup.bash
```

*Nota sobre compilación manual:* Si agregas nuevos nodos o modificas código, compila desde la raíz del repositorio usando:
```bash
colcon build --symlink-install
```

## 4. Comandos de Ejecución y Pruebas

### Comando exacto para abrir RViz2
Para visualizar el robot, ejecuta el archivo launch (asegúrate de haber cargado el entorno previamente):
```bash
ros2 launch grupo03_kuka_kr6_bringup display.launch.py
```
*(Alternativa rápida: puedes ejecutar `./abrir.sh` desde la raíz del repositorio).*

### Procedimiento A: Prueba Completa (Cinemática Inversa + Directa)
Este procedimiento permite enviar una coordenada en el espacio ($X, Y, Z$); el nodo IK calculará los ángulos necesarios, y el nodo FK verificará matemáticamente si la posición alcanzada coincide con la deseada.

1. Abre una terminal y ejecuta el *launch* file de RViz2.
2. **Importante:** Cierra la ventana emergente de *Joint State Publisher GUI* (la de los sliders) para que no interfiera publicando ceros en las articulaciones.
3. Abre una **segunda terminal** (recuerda ejecutar `source entorno.sh`) y corre el nodo IK:
   ```bash
   ros2 run kinematics ik_node
   ```
4. Abre una **tercera terminal** (`source entorno.sh`) y corre el nodo FK:
   ```bash
   ros2 run kinematics fk_node
   ```
5. Abre una **cuarta terminal** (`source entorno.sh`) y envía el objetivo espacial. *Ejemplo concreto de una prueba:*
   ```bash
   ros2 topic pub /target geometry_msgs/msg/Point "{x: 0.50, y: 0.20, z: 0.80}" --once
   ```
6. **Observar resultados:** El robot en RViz se moverá al punto deseado. La terminal del nodo IK informará las iteraciones de convergencia, y la terminal del nodo FK imprimirá la posición final ($p_f$), la posición solicitada ($p_d$) y calculará el error.

### Procedimiento B: Prueba de Solo Cinemática Directa (FK manual)
Este procedimiento permite mover el brazo manualmente y ver cómo el nodo FK calcula las posiciones cartesianas y los cuaterniones en tiempo real.

1. Abre una terminal y ejecuta el *launch* file de RViz2 (esta vez **mantén abierto** el *Joint State Publisher GUI*).
2. Abre una **segunda terminal** (`source entorno.sh`) y corre el nodo FK:
   ```bash
   ros2 run kinematics fk_node
   ```
3. Mueve las barras deslizantes (*sliders*) en la interfaz gráfica.
4. **Observar resultados:** La terminal de FK imprimirá en tiempo real las coordenadas $x, y, z$ y el cuaternión del extremo del robot.

## 5. Tópicos Utilizados
* `/joint_states` (`sensor_msgs/msg/JointState`): Utilizado por el nodo FK para leer la configuración angular, y por el nodo IK para publicar los ángulos calculados y mover el robot en RViz.
* `/target` (`geometry_msgs/msg/Point`): Tópico personalizado. El nodo IK se suscribe para recibir las coordenadas espaciales ($X, Y, Z$) deseadas. El nodo FK también se suscribe para almacenar el objetivo y realizar el cálculo de error.
* `/robot_description` (`std_msgs/msg/String`): Publicado por *robot_state_publisher* para cargar la estructura URDF en RViz.

## 6. Estructura del Repositorio
```text
.
├── .gitignore
├── README.md
├── dependencias.repos
├── abrir.sh
├── entorno.sh
├── instalar.sh
├── recompilar.sh
├── verificar.sh
├── docs/
│   └── informe_final.pdf
└── src/
    ├── grupo03_kuka_kr6_bringup/
    │   ├── launch/
    │   │   └── display.launch.py
    │   ├── CMakeLists.txt
    │   └── package.xml
    └── kinematics/
        ├── kinematics/
        │   ├── __init__.py
        │   ├── fk_node.py
        │   └── ik_node.py
        ├── package.xml
        ├── setup.cfg
        └── setup.py
```

## 7. Errores Conocidos y Consideraciones Particulares
* **Conflicto de publicación en `/joint_states`:** Si se utiliza el nodo de Cinemática Inversa (IK) con la interfaz gráfica (GUI) de *Joint State Publisher* abierta, el robot temblará erráticamente en RViz. Esto ocurre porque la GUI y el nodo IK intentan escribir comandos contradictorios en el mismo tópico al mismo tiempo. Es mandatorio cerrar la GUI para el Procedimiento A.

## 8. Ejemplo de resultados
Al seguir Procedimiento A: Prueba Completa (Cinemática Inversa + Directa), mediante el objetivo:\
{x: 0.50, y: 0.20, z: 0.80}\
La terminal para cinemática inversa muestra:\
Ángulos: [-0.3795, -1.5364, 1.713, -0.013, 0.0247, -0.0]\
Mientras que la terminal para cinemática directa muestra:\
pd: (0.50, 0.20, 0.80), pf: (0.5003, 0.1995, 0.7996) | Error: 0.000839 m


=======
# kinematics_kuka_kr6_r900_sixx_ws
>>>>>>> 2c563493db95e0d48e197cfcb5c149256cb60bc7
