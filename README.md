# USVot - WAM-V + RRT Planner + Control
Es necesario tener o Ubuntu 24 o un contenedor docker para ver la simulación.
Se necesita instalar ROS2 jazzy.
Todo nuestro código se encuentra dentro de wamv_control/wamv_control.

Mini tutorial rápido para lanzar la simulación 

```bash
# 1. Crea workspace y entra
mkdir -p ~/vrx_ws/src && cd ~/vrx_ws/src

# 2. Clona los dos repos (en la misma carpeta src)
git clone https://github.com/osrf/vrx.git
git clone https://github.com/danisanch118/cpr.git

# 3. Vuelve al workspace, instala dependencias y compila
cd ~/vrx_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install

# 4. Source (hazlo en cada terminal nueva que abras)
source ~/vrx_ws/install/setup.bash
```
Para lanzar la simulación seguimos estos pasos:
```bash
ros2 launch wamv_control competition2.launch.py
 ```
Despues en otra terminal abrimos rviz y elegimos el archivo de configuración wamv.rviz dentro de la carpeta config
```bash
ros2 launch vrx_gazebo rviz.launch.py
```
