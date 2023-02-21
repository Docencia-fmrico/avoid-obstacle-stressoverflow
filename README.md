# ASR-0-AvoidObstacle
Práctica 0 - Arquitecturas Software para Robots 2023

Crea un comportamiento autónomo de un robot usando una máquina de estado finito (FSM) para evitar obstáculos:
1. El robot empieza parado, y comienza su comportamiento cuando se pulsa un botón del robot.
2. El robot avanza hasta encontrar un obstáculo a menos de un metro enfrente de él.
3. Cuando encuentra un obstáculo, el robot gira 90 grados, y realiza un movimiento de arco para sobrepasarlo.
4. Si mientras está haciendo el arco, se encuentra un nuevo obstáculo, vuelve a hacer lo mismo del punto 3.

![asr_practica_0](https://user-images.githubusercontent.com/3810011/217230998-a162f2e1-cf50-4e26-9155-53ca73e99f86.png)

El robot debe funcionar en el robot real Kobuki.

Puntuación (sobre 10):

* +8 correcto funcionamiento en el robot real.
* +2 Readme.md bien documentado con videos.
* -3 Warnings o que no pase los tests.
* +1 Setup de CI/CD

> Buenas Carlos, acuérdate de hacer pull antes de intentar pushear nada, que nos la lías. :D

Rama carlos

Seguro que os sale lo mejor que podáis !!

-------------------------------------------
# ASR-0-AvoidObstacle
Práctica 0 - Arquitecturas Software para Robots 2023

## Goals
Create an autonomous robot machine using a finite state machine (FSM) to avoid obstacles:
1. The robot starts stopped, and starts its behavior when a button on the robot is pressed.
2. The robot advances until it finds an obstacle less than one meter in front of it.
3. When it find an obstacle, the robot turns 90 degrees, and makes an arc movement to go over it.
4. If while you are making the arc, you find a new obstacle, do the same thing from point 3 again.

## Implementation ⚙️ 
To get hands-on with this practice, the first thing we did was sketch a state diagram in which you could see at a glance the different states through which the robot would go through in its operation. ![image](https://user-images.githubusercontent.com/102520602/220409787-f4fcc1de-3ffe-4928-ac15-92db50e80ffd.png)
This was the first sketch we did and as you can see there are the states: stopped, go forward, obstacle turn, arc turn, turn exit.

Later we thought that in order for the practice to be more complete, we could add some more things that would provide the robot with better functioning in addition to what was already requested
.
### Additional implementations to practice 👨‍🔧 
  - **Bumper:** To make the detection of the robot as accurate as possible we decided to take readings from the bumper because the laser only detects things that are at the same height as it, but does not detect anything below it. For this reason, we helped with the bumper because it could detect any object that the laser does not capture below it. When the robot crash with an undetected object, it moves back a few meters and begins to make an arc to try to avoid the obstacle as if it had been detected by the laser.
  
  - **Wheel drop:** We also decided to add the wheel drop topic to know when the robot is in the air because a person has lifted it. When it is detected that the wheels have been lifted, it stops walking and returns to the start state. This allows a greater handling of the robot in unexpected situations.
 
  - **Leds and sounds:** To make it easier and more comfortable to debug the robot's status, we decided to add colored leds and sounds to indicate what it was doing on inside the robot in each case. For example, when the robot changes state, it emits a certain sound or when the robot detects an unexpected object, the led turns red.


  - **New node:** (completar luego)


### State diagram 📊 
Since we decided to create new features and new states, we had to restructure the state diagram. Some of the states were renamed and others created from scratch. In addition, the Lidar node was added with the leds apart from the main node.

(cometar el diagrama de estadios)

## Observations 🔎 
  - Grip change: Durante la realizacion de la practica nos dimos cuenta de que el robot no andaba igual en el suelo y en la moqueta. Esto se deve a que tienen distinto rozaminto y dificulta de mayor o menor medida la circulacion del robot. Esto causa que el robot
  
  - Sensor remaping:
  - Parameters in params.yaml
  
