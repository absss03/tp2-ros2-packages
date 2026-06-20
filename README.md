## Guía para descargar, compilar y ejecutar el proyecto:

1. Crear el espacio de trabajo de ROS y el directorio /src
   ```bash
   mkdir -p ~/ros2_ws/src
   ```
   
2. Ingresar al directorio /src y clonar el repositorio
    ```bash
    cd ~/ros2_ws/src
    git clone https://github.com/absss03/tp2-ros2-packages.git
    ```

3. Volver a la raíz del workspace y hacer el source para que la terminal reconozca los comando de ROS
    ```bash
    cd ~/ros2_ws
    source /opt/ros/jazzy/setup.bash
    ```

4. Instalar las dependencias requeridas por el proyecto
    ```bash
    rosdep update
    rosdep install --from-paths src --ignore-src -y --rosdistro jazzy
    ```

5. Compilar el espacio de trabajo
    ```bash
    colcon build
    ```

6. Cargar la configuración local para que la terminal reconozca los paquetes recién compilados
   ```bash
    source install/setup.bash
   ```

7. Ejecutar el launchfile
   ```bash
    ros2 launch my_robot_bringup bringup.launch.xml
   ```
