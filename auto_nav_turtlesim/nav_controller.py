#!/usr/bin/env python3
#Código hecho por: Santiago Chevez Trejo 
#  Este código es una simula la navegación autonoma de Quantum Robotics en la parte de la lógica del movimiento
#suponiendo que va a estar recibiendo información (a traves de tópicos) de la posición, orientación, obstaculos y encontrar objetivo.
#Se necesita correr la simulación de turtlesim

import rclpy, math, sys
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int8, Bool
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class TurtleController(Node):
    #Se declaran las variables de clase
    def __init__(self):
        super().__init__("turtle_controller")
        self.get_logger().info('Navegation controller is ready ...')
        #Crear los timers los publicadores y los subscriptores
        self.timer = self.create_timer(0.5, self.callback_follow_gps)
        self.timer2 = self.create_timer(1,self.callback_nav_controller)
        self.timer3 = self.create_timer(1,self.callback_routine_controller)
        self.timer4 = self.create_timer(0.5,self.go_to_point)
        self.timer5 = self.create_timer(1,self.rotate_360)
        self.timer5 = self.create_timer(1,self.obstacle_avoidance)
        self.timer6 = self.create_timer(1,self.center_and_approach)
        self.pub =self.create_publisher(Twist, "turtle1/cmd_vel",10)
        self.sub =self.create_subscription(Pose, "turtle1/pose",self.callback_turtle_pose,10)
        self.sub2 = self.create_subscription(Float32MultiArray, "target_coordinates", self.callback_target_coords,10)
        self.sub3 = self.create_subscription(Int8, "target_type",self.callback_target_type,10)
        self.sub4 = self.create_subscription(Bool, "obstacle_detected",self.callback_obstacle,10)
        self.sub5 = self.create_subscription(Bool, "objective_detected",self.callback_objective,10)
        # Se crean las variables que se utilizarán para la navegación
        self.pose = None
        self.target_x=None
        self.target_y=None
        self.target_type = None
        self.follow_gps_active=False
        self.follow_gps_done = False
        self.search_routine_active = False
        self.search_routine_done = False
        self.go_to_point_active=False
        self.go_to_point_done = False
        self.rotate_360_active =False
        self.rotate_360_done = False
        self.obstacle_avoidance_active = False
        self.pointsx=[]
        self.pointsy=[]
        self.center_and_approach_active=False
        self.center_and_approach_done=False
        
        self.once=True
        self.once_center=True
        self.i=0
        self.count=0

    def callback_nav_controller(self):
        #Lleva a cabo el control de todas las demás funciones, las activa y las desactiva siempre que sea necesario
        #Si recibe el tipo de target 1 se activa el GPS only
        #Si recibe el tipo de target 2 se activa la ARUCO
        if self.target_type is None: 
            print("Waiting ...")
            return
        elif self.target_type == 1:
            if not self.follow_gps_active:
                print("GPS ONLY")
                print("Activating Follow GPS")
                if self.pose is None or self.target_x is None or self.target_y is None: 
                    print("Couldn't activate")
                    return
                else:
                    self.follow_gps_active=True
            if self.follow_gps_done:
                print("Arrived")
                self.target_type=None
                self.target_x = None
                self.target_y = None
                self.follow_gps_active=False
                self.follow_gps_done=False
        elif self.target_type == 2:
            if not self.follow_gps_active and not self.follow_gps_done:
                print("ARUCO")
                print("Activating Follow GPS")
                if self.pose is None or self.target_x is None or self.target_y is None: 
                    print("Couldn't activate")
                    return
                else:
                    self.follow_gps_active=True
            if self.follow_gps_done:
                self.follow_gps_active=False
                if not self.search_routine_active:
                    print("Arrived at coordinates. Activating search routine")
                    self.search_routine_active=True
                    self.pointsx=[self.target_x,self.target_x+ 0.5, self.target_x+0.5, self.target_x+0.5, self.target_x, self.target_x-0.5, self.target_x-0.5, self.target_x-0.5, self.target_x-0.5, self.target_x, self.target_x + 0.5, self.target_x+ 1, self.target_x+ 1, self.target_x+ 1, self.target_x+ 1 ]
                    self.pointsy=[self.target_y+0.5,self.target_y+0.5,self.target_y,self.target_y-0.5,self.target_y-0.5,self.target_y-0.5,self.target_y,self.target_y+0.5, self.target_y+1,self.target_y+1,self.target_y+1,self.target_y+1, self.target_y+0.5, self.target_y, self.target_y-0.5, self.target_y-1 ]
                if self.search_routine_done:
                    if not self.center_and_approach_active:
                        print("No se encontro el ARUCO")
                        self.target_type=None
                        self.target_x = None
                        self.target_y = None
                        self.follow_gps_active=False
                        self.follow_gps_done=False
                        self.search_routine_active=False
                        self.search_routine_done=False
                        self.go_to_point_active=False
                        self.go_to_point_done=False
                    else:
                        if self.center_and_approach_done:
                            print("Arrived")
                            self.center_and_approach_active=False
                            self.center_and_approach_done = False
                            self.target_type=None
                            self.target_x = None
                            self.target_y = None
                            self.follow_gps_active=False
                            self.follow_gps_done=False
                            self.search_routine_active=False
                            self.search_routine_done=False
                            self.go_to_point_active=False
                            self.go_to_point_done=False


    def callback_follow_gps(self):
        #Esta función lleva a cabo el mover a la tortuga a la ubicación objetivo
        if not self.follow_gps_active or self.obstacle_avoidance_active: return
        else:
            Dx = self.target_x-self.pose.x
            Dy = self.target_y-self.pose.y
            distance = math.sqrt(Dx**2 + Dy**2)
            error = math.atan2(Dy,Dx)-self.pose.theta
            error = math.atan2(math.sin(error),math.cos(error))
            print("Distance: ")
            print(distance)
            print("Angle error: ")
            print (error)

            msg = Twist()
            if abs(error)>0.0436332 and distance > 0.1:
                msg.linear.x = 0.0
                msg.angular.z = self.signo(error)*self.velocity_rotation(abs(error))
            elif distance > 0.1:
                msg.linear.x = self.velocity_calculator(distance)
                msg.angular.z = 0.0
            else:
                print("Done")
                msg.linear.x = 0.0
                msg.angular.z = 0.0
                self.follow_gps_done=True
            self.pub.publish(msg)


    def callback_routine_controller(self):
        #Esta función lleva a cabo la rutina de busqueda sin importar el tipo de objetivo
        if not self.search_routine_active or self.obstacle_avoidance_active: return
        else:
            if not self.go_to_point_active:
                print("Yendo a punto", self.i)
                self.go_to_point_active=True
            if self.go_to_point_done:
                print("girar")
                if self.once:
                    self.target_angle=self.pose.theta-0.0872665
                    self.count=0
                    self.once=False
                self.rotate_360_active=True
                if self.rotate_360_done:
                    self.go_to_point_done=False
                    self.go_to_point_active=False
                    self.rotate_360_active=False
                    self.rotate_360_done=False
                    self.once=True
                    self.i +=1
                    if self.i>=15 or self.center_and_approach_active:
                        self.i=0
                        self.search_routine_done=True
                    
            
    def go_to_point(self):
        #Es muy parecido a follow gps, pero se utiliza para la evación de obstaculos, para ir de punto a punto en la rutina
        # y unicamente por ser simulació, para el center and approach 
        if self.go_to_point_active and not self.go_to_point_done:
            print(self.pointsx[self.i],"," ,self.pointsy[self.i])
            Dx = self.pointsx[self.i]-self.pose.x
            Dy = self.pointsy[self.i]-self.pose.y
            distance = math.sqrt(Dx**2 + Dy**2)
            error = math.atan2(Dy,Dx)-self.pose.theta
            error = math.atan2(math.sin(error),math.cos(error))

            msg = Twist()
            if abs(error)>0.0436332 and distance > 0.1:
                msg.linear.x = 0.0
                msg.angular.z = self.signo(error)*self.velocity_rotation(abs(error))
            elif distance > 0.1:
                msg.linear.x = self.velocity_calculator(distance)
                msg.angular.z = 0.0
            else:
                msg.linear.x = 0.0
                msg.angular.z = 0.0
                self.go_to_point_done = True
            self.pub.publish(msg)

    def rotate_360(self):
        #Esta funión la utilicé para poder girar para buscar el objetivo
        if not self.rotate_360_active:return
        elif self.center_and_approach_active:
            print("Object_detected")
            self.rotate_360_done=True
        else:
            print(self.target_angle)
            msg = Twist()
            if self.count<=3:
                msg.linear.x = 0.0
                msg.angular.z = 0.3
                self.count+=1
            elif self.pose.theta > self.target_angle-0.174533 and self.pose.theta < self.target_angle+0.174533 :
                msg.linear.x = 0.0
                msg.angular.z = 0.0
                self.rotate_360_done=True
            else:
                msg.linear.x = 0.0
                msg.angular.z = 0.3
                
                
            self.pub.publish(msg)
    
    def obstacle_avoidance(self):
        #Esta genera las acciones para evadir un obstaculo
        if self.obstacle_avoidance_active:
            print("Avoiding obstacle")
            if self.once:
                self.target_angle=self.pose.theta+math.pi/2
                self.once=False
                self.temp_pointsx=self.pointsx
                self.temp_pointsy=self.pointsy
                self.temp_i=self.i
                self.i=0
                self.pointsx = [self.pose.x+(0.3*math.cos(self.target_angle))]
                self.pointsy = [self.pose.y+(0.3*math.sin(self.target_angle))]
            self.go_to_point_active= True
            if self.go_to_point_done:
                self.obstacle_avoidance_active=False
                self.go_to_point_active = False
                self.go_to_point_done = False
                self.once=True
                self.pointsx=self.temp_pointsx
                self.pointsy=self.temp_pointsy
                self.i=self.temp_i
                self.rotate_360_active=False

    def center_and_approach(self):
        #En esta función solo se avanza 
        #En realidad debería recibir si el objetivo esta centrado, a la izquierda o a la derecha y la distancia 
        # y debería ir al punto final
        if self.center_and_approach_active:
            print("Center and aproach")
            if self.once_center:
                self.once_center=False
                self.temp_pointsx=self.pointsx
                self.temp_pointsy=self.pointsy
                self.temp_i=self.i
                self.go_to_point_done=False
                self.i=0
                self.pointsx = [self.pose.x+0.3*math.cos(self.pose.theta)]
                self.pointsy = [self.pose.y+0.3*math.sin(self.pose.theta)]
                print(self.pointsx)
                print(self.pointsy)
            self.go_to_point_active= True
            if self.go_to_point_done:
                self.obstacle_avoidance_active=False
                self.go_to_point_active = False
                self.go_to_point_done = False
                self.once_center=True
                self.pointsx=self.temp_pointsx
                self.pointsy=self.temp_pointsy
                self.i=0
                self.rotate_360_active=False
                self.center_and_approach_done= True

    def callback_turtle_pose(self,msg):
        # Tópico de la posicion y orientación de la tortuga 
        self.pose=msg
    
    def callback_target_coords(self,msg):
        #Recibe las coordenadas
        print("Target recieved")
        if self.target_x is None or self.target_y is None:
            self.target_x=msg.data[0]
            self.target_y=msg.data[1]
    def callback_target_type(self,msg):
        #Recibe el tipo de objetivo
        print("Target recieved")
        if self.target_type is None:
            self.target_type=msg.data

    def my_map(self,in_min, in_max, out_min, out_max, x):
        #Una regla de 3
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    def velocity_calculator(self,v):
        # Calcula la velocidad que debe tener la tortuga para llegar a su objetivo
        if v>3:
            return 1.0
        else:
            return self.my_map(0.1,3,0.2,1,v)
    def velocity_rotation(self, v):
        # Calcula la velocidad con la que la tortuga debe rotar para llegar al ángulo objetivo
        if v> 3*math.pi/4:
            return 1.0
        else:
            return self.my_map(0.0001,3*math.pi/4,0.1,1.0,v)
        
    def signo(self,n):
        #Da el signo del valor
        if n>=0:
            return 1
        else:
            return -1
        
    def callback_obstacle(self,msg):
        #Tópico de la deteccion de obstáculo 
        if self.target_type is None or self.rotate_360_active: return
        self.obstacle_avoidance_active=msg.data

    def callback_objective(self,msg):
        #Topico de la detección del objetivo
        if not self.rotate_360_active: return
        self.center_and_approach_active=msg.data

def main(args=None):
     rclpy.init(args=args)
     nodeh = TurtleController()
     try: rclpy.spin(nodeh)
     except Exception as error: print(error)
     except KeyboardInterrupt: print("Node Terminated!")
  
  
if __name__=="__main__":
  main()