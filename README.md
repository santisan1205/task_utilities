# task_utilities

Este repositorio contiene funciones y herramientas de alto nivel para la ejecución de tareas complejas en un robot Pepper. task_utilities integra y coordina funcionalidades de percepción, habla, manipulación, navegación y otras utilidades (a través de PyToolkit) para facilitar el desarrollo de aplicaciones robóticas.

## Requisitos y Dependencias

Antes de ejecutar task_utilities, es necesario tener instalado ROS (Noetic, Melodic o la versión correspondiente a su sistema) y contar con las siguientes herramientas:
- [manipulation_utilities](https://github.com/SinfonIAUniandes/manipulation_utilities)
- [speech_utilities](https://github.com/SinfonIAUniandes/speech_utilities)
- [navigation_utilities](https://github.com/SinfonIAUniandes/navigation_utilities)
- [perception_utilities](https://github.com/SinfonIAUniandes/perception_utilities)

Además, asegúrese de que todas las dependencias de Python y de ROS están correctamente instaladas.

## Creación del Espacio de Trabajo

Si aún no cuenta con un workspace, puede crearlo usando los siguientes comandos:

```ROS
mkdir -p ~/sinfonia_ws/src
cd ~/sinfonia_ws/src
```

Una vez en el directorio `src`, clone los repositorios necesarios:

```ROS
git clone https://github.com/SinfonIAUniandes/manipulation_utilities.git
git clone https://github.com/SinfonIAUniandes/speech_utilities.git
git clone https://github.com/SinfonIAUniandes/navigation_utilities.git
git clone https://github.com/SinfonIAUniandes/perception_utilities.git
git clone https://github.com/SinfonIAUniandes/task_utilities.git
git clone https://github.com/SinfonIAUniandes/perception_msgs.git
git clone https://github.com/SinfonIAUniandes/robot_toolkit_msgs.git
git clone https://github.com/SinfonIAUniandes/navigation_msgs.git
git clone https://github.com/SinfonIAUniandes/manipulation_msgs.git
git clone https://github.com/SinfonIAUniandes/speech_msgs.git
```

Después, compile el workspace:

```ROS
cd ~/sinfonia_ws
catkin_make
```

Finalmente, configure el entorno:

```ROS
source devel/setup.bash
```

## Instrucciones de Ejecución

task_utilities depende de que las herramientas complementarias estén en ejecución. Para correr task_utilities y las demás utilidades, siga estos pasos:

1. **Ponga a correr el toolkit**  
   Siga los pasos en: [Documentacion del Robot](https://github.com/dcuevasa/SinfonIA-Wiki/wiki/Documentaci%C3%B3n-Robot)

2. **Inicie las utilidades complementarias**  
   En terminales separadas, ejecute los siguientes comandos para correr las herramientas esenciales:

   - **Manipulation Utilities:**  
     ```ROS
     rosrun manipulation_utilities manipulation_utilities
     ```
     
   - **Speech Utilities:**  
     ```ROS
     rosrun speech_utilities speech_utilities.py
     ```
     
   - **Navigation Utilities:**  
     ```ROS
     rosrun navigation_utilities NavigationUtilities.py
     ```
     
   - **Perception Utilities:**  
     ```ROS
     rosrun perception_utilities PerceptionUtilities.py
     ```

3. **Ejecute task_utilities**  
   Finalmente, en otra terminal, ejecute task_utilities:

   ```ROS
   rosrun task_utilities evento.py
   ```

   task_utilities utilizará los servicios expuestos por las otras herramientas para coordinar tareas complejas en el robot.

## Descripción

- **Integración de Servicios:**  
  task_utilities combina servicios de percepción, habla, manipulación y navegación para generar comportamientos autónomos y coordinados.
  
- **Funcionalidades de Alto Nivel:**  
  Entre otras, task_utilities ofrece funciones para:
  - Inicializar y configurar el robot.
  - Ejecutar tareas basadas en lenguaje natural.
  - Coordinar acciones de manipulación y navegación.
  - Integrar respuesta de servicios de reconocimiento (por ejemplo, búsqueda de objetos o reconocimiento facial).

## Enlaces a Repositorios Complementarios

- [manipulation_utilities](https://github.com/SinfonIAUniandes/manipulation_utilities)
- [speech_utilities](https://github.com/SinfonIAUniandes/speech_utilities)
- [navigation_utilities](https://github.com/SinfonIAUniandes/navigation_utilities)
- [perception_utilities](https://github.com/SinfonIAUniandes/perception_utilities)

## Notas

- Asegúrese de haber configurado correctamente las variables de entorno necesarias (por ejemplo, `ROBOT`, `PEPPER_IP`, `ROS_MASTER_URI`, etc.) en su archivo `.bashrc`.
- Revise la documentación específica de cada herramienta para detalles adicionales y para solucionar problemas comunes.
- task_utilities se utiliza para orquestar las funciones de las diferentes áreas y es el punto de integración para ejecutar tareas complejas en el robot.

¡Listo! Con estos pasos, podrá poner a correr task_utilities junto con sus herramientas complementarias y desarrollar aplicaciones robóticas integradas para Pepper.
