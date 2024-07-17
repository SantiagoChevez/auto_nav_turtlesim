# auto_nav_turtlesim

Para correr esta simulación debes correr la simulación del turtlsim con el siguiente comando

ros2 run turtlesim turtlesim_node

Después debes correr la navegación autonoma con el siguiente comando

ros2 run auto_nav_turtlesim nav_controller

Por útlimo caa vez que se quiera mandar un objetivo se debe correr la simulación de Web

ros2 run auto_nav_turtlesim web 

Los tipos de objetivo son 
1 - GPS Only
2 - ARUCO

Solo ingresas las coordenadas y listo!