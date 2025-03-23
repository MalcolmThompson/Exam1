#!/usr/bin/env python3

# Importamos las librerías necesarias para trabajar con ROS y turtlesim
import rospy
from geometry_msgs.msg import Twist  # Para controlar la velocidad de la tortuga
from turtlesim.msg import Pose  # Para obtener la posición de la tortuga
from turtlesim.srv import Spawn, Kill  # Para crear y eliminar tortugas
from std_srvs.srv import Empty  # Para limpiar el área de trabajo
import time  # Para manejar pausas en la ejecución

class Figuras:
    def __init__(self):
        # Configuración inicial
        self.setup()

        # Eliminar la tortuga por defecto
        self.borrar()

    def setup(self):
        """Configuración inicial del nodo y variables."""
        rospy.init_node('tortugas_figuras')

        # Variables para el control de la tortuga
        self.vel_pub = None  # Publicador para enviar comandos de velocidad
        self.pos_x = 0  # Posición actual en X de la tortuga
        self.pos_y = 0  # Posición actual en Y de la tortuga
        self.tasa = rospy.Rate(10)  # Frecuencia de actualización (10 Hz)

        # Dimensiones del área de trabajo
        self.lim_x = 11
        self.lim_y = 11

    def borrar(self):
        """Eliminar la tortuga inicial."""
        rospy.wait_for_service('/kill')
        try:
            eliminar = rospy.ServiceProxy('/kill', Kill)
            eliminar('turtle1')
            rospy.loginfo("La tortuga inicial 'turtle1' fue eliminada.")
        except rospy.ServiceException as e:
            rospy.logwarn(f"No se pudo eliminar la tortuga inicial: {e}")

    def limpiar(self):
        """Limpiar el área de trabajo."""
        rospy.wait_for_service('/clear')
        try:
            limpiar = rospy.ServiceProxy('/clear', Empty)
            limpiar()
            rospy.loginfo("Área de trabajo limpiada.")
        except rospy.ServiceException as e:
            rospy.logerr(f"No se pudo limpiar el área de trabajo: {e}")

    def pos_actual(self, pose):
        """Actualizar la posición actual de la tortuga."""
        self.pos_x = pose.x
        self.pos_y = pose.y

    def sub(self, nombre):
        """Suscribirse al tópico de posición."""
        rospy.Subscriber(f'/{nombre}/pose', Pose, self.pos_actual)

    def pub(self, nombre):
        """Configurar el publicador de velocidad."""
        self.vel_pub = rospy.Publisher(f'/{nombre}/cmd_vel', Twist, queue_size=10)

    def mover(self, x_dest, y_dest):
        """Mover la tortuga a un punto."""
        Kp = 1.0  # Ganancia proporcional

        while not rospy.is_shutdown():
            error_x = x_dest - self.pos_x
            error_y = y_dest - self.pos_y

            # Si la tortuga está cerca del punto, detenerse
            if abs(error_x) < 0.01 and abs(error_y) < 0.01:
                rospy.loginfo(f"Llegó al punto: ({x_dest}, {y_dest})")
                break

            # Calcular velocidades proporcionales
            vel_x = Kp * error_x
            vel_y = Kp * error_y

            # Limitar las velocidades
            vel_x = max(min(vel_x, 2.5), -2.5)
            vel_y = max(min(vel_y, 2.5), -2.5)

            # Publicar el mensaje de velocidad
            msg = Twist()
            msg.linear.x = vel_x
            msg.linear.y = vel_y
            self.vel_pub.publish(msg)

            self.tasa.sleep()

        # Detener la tortuga
        self.vel_pub.publish(Twist())
        rospy.loginfo("Tortuga detenida.")

    def validar(self, puntos):
        """Validar si los puntos están dentro del área de trabajo."""
        for x, y in puntos:
            if x < 0 or x > self.lim_x or y < 0 or y > self.lim_y:
                return False
        return True

    def mini_menu(self):
        """Menú que aparece después de realizar una figura."""
        while True:
            print("\nPresiona 'r' para regresar al menú principal y limpiar el área de trabajo.")
            opc = input("Selecciona una opción: ").strip().lower()

            if opc == 'r':
                self.limpiar()
                return  # Regresa al menú principal
            else:
                print("Opción no válida. Intenta de nuevo.")

    def cuadrado(self, x_inicio, y_inicio):
        """Dibujar un cuadrado."""
        while True:
            rospy.wait_for_service('/spawn')
            rospy.wait_for_service('/kill')

            try:
                # Coordenadas del cuadrado
                puntos = [(x_inicio, y_inicio)]
                puntos.append((puntos[-1][0] + 2, puntos[-1][1]))
                puntos.append((puntos[-1][0], puntos[-1][1] + 2))
                puntos.append((puntos[-1][0] - 2, puntos[-1][1]))
                puntos.append((puntos[0][0], puntos[0][1]))

                if not self.validar(puntos):
                    rospy.logwarn("El cuadrado no cabe en el área de trabajo. Selecciona otras coordenadas.")
                    x_inicio = float(input("Nueva coordenada x inicial: "))
                    y_inicio = float(input("Nueva coordenada y inicial: "))
                    continue

                # Crear la tortuga
                spawn = rospy.ServiceProxy('/spawn', Spawn)
                spawn(x_inicio, y_inicio, 0.0, 'turtle_square')
                rospy.loginfo(f"Tortuga 'turtle_square' creada en ({x_inicio}, {y_inicio}).")

                # Configurar la tortuga
                self.sub('turtle_square')
                self.pub('turtle_square')

                for x, y in puntos:
                    self.mover(x, y)

                rospy.loginfo("¡Cuadrado terminado!")

                # Eliminar la tortuga
                kill = rospy.ServiceProxy('/kill', Kill)
                kill('turtle_square')
                rospy.loginfo("Tortuga 'turtle_square' eliminada.")

                # Mostrar el menú pequeño
                self.mini_menu()
                break

            except rospy.ServiceException as e:
                rospy.logerr(f"Error al manejar la tortuga 'turtle_square': {e}")
                break

    def romboide(self, x_inicio, y_inicio):
        """Dibujar un romboide."""
        while True:
            rospy.wait_for_service('/spawn')
            rospy.wait_for_service('/kill')

            try:
                # Coordenadas del romboide
                puntos = [(x_inicio, y_inicio)]
                puntos.append((puntos[-1][0] + 3, puntos[-1][1] + 1))
                puntos.append((puntos[-1][0], puntos[-1][1] + 2))
                puntos.append((puntos[-1][0] - 3, puntos[-1][1] - 1))
                puntos.append((puntos[0][0], puntos[0][1]))

                if not self.validar(puntos):
                    rospy.logwarn("El romboide no cabe en el área de trabajo. Selecciona otras coordenadas.")
                    x_inicio = float(input("Nueva coordenada x inicial: "))
                    y_inicio = float(input("Nueva coordenada y inicial: "))
                    continue

                # Crear la tortuga
                spawn = rospy.ServiceProxy('/spawn', Spawn)
                spawn(x_inicio, y_inicio, 0.0, 'turtle_rhombus')
                rospy.loginfo(f"Tortuga 'turtle_rhombus' creada en ({x_inicio}, {y_inicio}).")

                # Configurar la tortuga
                self.sub('turtle_rhombus')
                self.pub('turtle_rhombus')

                for x, y in puntos:
                    self.mover(x, y)

                rospy.loginfo("¡Romboide terminado!")

                # Eliminar la tortuga
                kill = rospy.ServiceProxy('/kill', Kill)
                kill('turtle_rhombus')
                rospy.loginfo("Tortuga 'turtle_rhombus' eliminada.")

                # Mostrar el menú pequeño
                self.mini_menu()
                break

            except rospy.ServiceException as e:
                rospy.logerr(f"Error al manejar la tortuga 'turtle_rhombus': {e}")
                break

    def triangulo(self, x_inicio, y_inicio):
        """Dibujar un triángulo equilátero."""
        while True:
            rospy.wait_for_service('/spawn')
            rospy.wait_for_service('/kill')

            try:
                # Coordenadas del triángulo equilátero
                puntos = [(x_inicio, y_inicio)]
                puntos.append((puntos[-1][0] + 2, puntos[-1][1]))
                puntos.append((puntos[0][0] + 1, puntos[0][1] + 2))
                puntos.append((puntos[0][0], puntos[0][1]))

                if not self.validar(puntos):
                    rospy.logwarn("El triángulo no cabe en el área de trabajo. Selecciona otras coordenadas.")
                    x_inicio = float(input("Nueva coordenada x inicial: "))
                    y_inicio = float(input("Nueva coordenada y inicial: "))
                    continue

                # Crear la tortuga
                spawn = rospy.ServiceProxy('/spawn', Spawn)
                spawn(x_inicio, y_inicio, 0.0, 'turtle_triangle')
                rospy.loginfo(f"Tortuga 'turtle_triangle' creada en ({x_inicio}, {y_inicio}).")

                # Configurar la tortuga
                self.sub('turtle_triangle')
                self.pub('turtle_triangle')

                for x, y in puntos:
                    self.mover(x, y)

                rospy.loginfo("¡Triángulo terminado!")

                # Eliminar la tortuga
                kill = rospy.ServiceProxy('/kill', Kill)
                kill('turtle_triangle')
                rospy.loginfo("Tortuga 'turtle_triangle' eliminada.")

                # Mostrar el menú pequeño
                self.mini_menu()
                break

            except rospy.ServiceException as e:
                rospy.logerr(f"Error al manejar la tortuga 'turtle_triangle': {e}")
                break

    def trapecio(self, x_inicio, y_inicio):
        """Dibujar un trapecio equilátero."""
        while True:
            rospy.wait_for_service('/spawn')
            rospy.wait_for_service('/kill')

            try:
                # Coordenadas del trapecio equilátero
                puntos = [(x_inicio, y_inicio)]  # Punto inferior izquierdo
                puntos.append((puntos[-1][0] + 4, puntos[-1][1]))  # Punto inferior derecho
                puntos.append((puntos[-1][0] - 1.5, puntos[-1][1] + 2))  # Punto superior derecho
                puntos.append((puntos[0][0] + 1.5, puntos[0][1] + 2))  # Punto superior izquierdo
                puntos.append((puntos[0][0], puntos[0][1]))  # Volver al punto inicial

                if not self.validar(puntos):
                    rospy.logwarn("El trapecio no cabe en el área de trabajo. Selecciona otras coordenadas.")
                    x_inicio = float(input("Nueva coordenada x inicial: "))
                    y_inicio = float(input("Nueva coordenada y inicial: "))
                    continue

                # Crear la tortuga
                spawn = rospy.ServiceProxy('/spawn', Spawn)
                spawn(x_inicio, y_inicio, 0.0, 'turtle_trapezoid')
                rospy.loginfo(f"Tortuga 'turtle_trapezoid' creada en ({x_inicio}, {y_inicio}).")

                # Configurar la tortuga
                self.sub('turtle_trapezoid')
                self.pub('turtle_trapezoid')

                for x, y in puntos:
                    self.mover(x, y)

                rospy.loginfo("¡Trapecio terminado!")

                # Eliminar la tortuga
                kill = rospy.ServiceProxy('/kill', Kill)
                kill('turtle_trapezoid')
                rospy.loginfo("Tortuga 'turtle_trapezoid' eliminada.")

                # Mostrar el menú pequeño
                self.mini_menu()
                break

            except rospy.ServiceException as e:
                rospy.logerr(f"Error al manejar la tortuga 'turtle_trapezoid': {e}")
                break

    def rombo(self, x_inicio, y_inicio):
        """Dibujar un rombo."""
        while True:
            rospy.wait_for_service('/spawn')
            rospy.wait_for_service('/kill')

            try:
                # Coordenadas del rombo
                puntos = [(x_inicio, y_inicio)]  # Punto inferior
                puntos.append((puntos[-1][0] + 2, puntos[-1][1] + 2))  # Punto derecho
                puntos.append((puntos[-1][0] - 2, puntos[-1][1] + 2))  # Punto superior
                puntos.append((puntos[-1][0] - 2, puntos[-1][1] - 2))  # Punto izquierdo
                puntos.append((puntos[0][0], puntos[0][1]))  # Volver al punto inicial

                if not self.validar(puntos):
                    rospy.logwarn("El rombo no cabe en el área de trabajo. Selecciona otras coordenadas.")
                    x_inicio = float(input("Nueva coordenada x inicial: "))
                    y_inicio = float(input("Nueva coordenada y inicial: "))
                    continue

                # Crear la tortuga
                spawn = rospy.ServiceProxy('/spawn', Spawn)
                spawn(x_inicio, y_inicio, 0.0, 'turtle_rhombus')
                rospy.loginfo(f"Tortuga 'turtle_rhombus' creada en ({x_inicio}, {y_inicio}).")

                # Configurar la tortuga
                self.sub('turtle_rhombus')
                self.pub('turtle_rhombus')

                for x, y in puntos:
                    self.mover(x, y)

                rospy.loginfo("¡Rombo terminado!")

                # Eliminar la tortuga
                kill = rospy.ServiceProxy('/kill', Kill)
                kill('turtle_rhombus')
                rospy.loginfo("Tortuga 'turtle_rhombus' eliminada.")

                # Mostrar el menú pequeño
                self.mini_menu()
                break

            except rospy.ServiceException as e:
                rospy.logerr(f"Error al manejar la tortuga 'turtle_rhombus': {e}")
                break

    def pentagono(self, x_inicio, y_inicio):
        """Dibujar un pentágono."""
        while True:
            rospy.wait_for_service('/spawn')
            rospy.wait_for_service('/kill')

            try:
                # Coordenadas del pentágono (calculadas usando ángulos)
                import math
                radio = 2  # Distancia desde el centro a los vértices
                puntos = []
                for i in range(5):
                    angulo = math.radians(72 * i)  # Ángulo en radianes (72° entre vértices)
                    x = x_inicio + radio * math.cos(angulo)
                    y = y_inicio + radio * math.sin(angulo)
                    puntos.append((x, y))
                puntos.append(puntos[0])  # Volver al punto inicial

                if not self.validar(puntos):
                    rospy.logwarn("El pentágono no cabe en el área de trabajo. Selecciona otras coordenadas.")
                    x_inicio = float(input("Nueva coordenada x inicial: "))
                    y_inicio = float(input("Nueva coordenada y inicial: "))
                    continue

                # Crear la tortuga
                spawn = rospy.ServiceProxy('/spawn', Spawn)
                spawn(x_inicio, y_inicio, 0.0, 'turtle_pentagon')
                rospy.loginfo(f"Tortuga 'turtle_pentagon' creada en ({x_inicio}, {y_inicio}).")

                # Configurar la tortuga
                self.sub('turtle_pentagon')
                self.pub('turtle_pentagon')

                for x, y in puntos:
                    self.mover(x, y)

                rospy.loginfo("¡Pentágono terminado!")

                # Eliminar la tortuga
                kill = rospy.ServiceProxy('/kill', Kill)
                kill('turtle_pentagon')
                rospy.loginfo("Tortuga 'turtle_pentagon' eliminada.")

                # Mostrar el menú pequeño
                self.mini_menu()
                break

            except rospy.ServiceException as e:
                rospy.logerr(f"Error al manejar la tortuga 'turtle_pentagon': {e}")
                break

        """Dibujar un pentágono."""
        while True:
            rospy.wait_for_service('/spawn')
            rospy.wait_for_service('/kill')

            try:
                # Coordenadas del pentágono
                puntos = [(x_inicio, y_inicio)]
                puntos.append((puntos[-1][0] + 2, puntos[-1][1]))
                puntos.append((puntos[-1][0] + 1, puntos[-1][1] + 2))
                puntos.append((puntos[-1][0] - 2, puntos[-1][1] + 2))
                puntos.append((puntos[-1][0] - 2, puntos[-1][1]))
                puntos.append((puntos[0][0], puntos[0][1]))

                if not self.validar(puntos):
                    rospy.logwarn("El pentágono no cabe en el área de trabajo. Selecciona otras coordenadas.")
                    x_inicio = float(input("Nueva coordenada x inicial: "))
                    y_inicio = float(input("Nueva coordenada y inicial: "))
                    continue

                # Crear la tortuga
                spawn = rospy.ServiceProxy('/spawn', Spawn)
                spawn(x_inicio, y_inicio, 0.0, 'turtle_pentagon')
                rospy.loginfo(f"Tortuga 'turtle_pentagon' creada en ({x_inicio}, {y_inicio}).")

                # Configurar la tortuga
                self.sub('turtle_pentagon')
                self.pub('turtle_pentagon')

                for x, y in puntos:
                    self.mover(x, y)

                rospy.loginfo("¡Pentágono terminado!")

                # Eliminar la tortuga
                kill = rospy.ServiceProxy('/kill', Kill)
                kill('turtle_pentagon')
                rospy.loginfo("Tortuga 'turtle_pentagon' eliminada.")

                # Mostrar el menú pequeño
                self.mini_menu()
                break

            except rospy.ServiceException as e:
                rospy.logerr(f"Error al manejar la tortuga 'turtle_pentagon': {e}")
                break

    def menu(self):
        """Menú principal."""
        while not rospy.is_shutdown():
            print("\nHacer Figuras con Turtlesim")
            print("1 -> Cuadrado")
            print("2 -> Romboide")
            print("3 -> Triángulo")
            print("4 -> Trapecio")
            print("5 -> Rombo")
            print("6 -> Pentágono")
            print("7 -> Salir")

            opc = input("Selecciona una opción: ")

            if opc == '7':
                print("¡Hasta luego!")
                break

            elif opc == '1':
                print("Opción seleccionada: Dibujar Cuadrado")
                x_inicio = float(input("Coordenada x inicial: "))
                y_inicio = float(input("Coordenada y inicial: "))
                self.cuadrado(x_inicio, y_inicio)

            elif opc == '2':
                print("Opción seleccionada: Dibujar Romboide")
                x_inicio = float(input("Coordenada x inicial: "))
                y_inicio = float(input("Coordenada y inicial: "))
                self.romboide(x_inicio, y_inicio)

            elif opc == '3':
                print("Opción seleccionada: Dibujar Triángulo")
                x_inicio = float(input("Coordenada x inicial: "))
                y_inicio = float(input("Coordenada y inicial: "))
                self.triangulo(x_inicio, y_inicio)

            elif opc == '4':
                print("Opción seleccionada: Dibujar Trapecio")
                x_inicio = float(input("Coordenada x inicial: "))
                y_inicio = float(input("Coordenada y inicial: "))
                self.trapecio(x_inicio, y_inicio)

            elif opc == '5':
                print("Opción seleccionada: Dibujar Rombo")
                x_inicio = float(input("Coordenada x inicial: "))
                y_inicio = float(input("Coordenada y inicial: "))
                self.rombo(x_inicio, y_inicio)

            elif opc == '6':
                print("Opción seleccionada: Dibujar Pentágono")
                x_inicio = float(input("Coordenada x inicial: "))
                y_inicio = float(input("Coordenada y inicial: "))
                self.pentagono(x_inicio, y_inicio)

if __name__ == '__main__':
    try:
        control = Figuras()
        control.menu()
    except rospy.ROSInterruptException:
        pass