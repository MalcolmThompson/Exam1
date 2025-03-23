#!/usr/bin/env python3

# Este script utiliza ROS para eliminar la tortuga inicial del simulador turtlesim
# y luego crear 5 tortugas nuevas en posiciones aleatorias dentro del área visible.

import rospy  # type: ignore # Se importa la librería principal de ROS en Python
from turtlesim.srv import Kill, Spawn  # type: ignore # Se importan los servicios necesarios
import random  # Se importa la librería para generar números aleatorios

# Se inicializa el nodo de ROS. Esto es necesario para que el programa funcione dentro del sistema de ROS.
rospy.init_node('controlador_de_tortugas')

# En esta parte se espera a que el servicio para eliminar tortugas esté disponible.
rospy.wait_for_service('/kill')

# Aquí se espera también al servicio para crear nuevas tortugas.
rospy.wait_for_service('/spawn')

# Se crean los proxies que permiten llamar a los servicios.
eliminar_tortuga = rospy.ServiceProxy('/kill', Kill)     # Proxy para eliminar una tortuga
agregar_tortuga = rospy.ServiceProxy('/spawn', Spawn)    # Proxy para crear una nueva tortuga

# Se intenta eliminar la tortuga que aparece por defecto al iniciar turtlesim (turtle1).
try:
    eliminar_tortuga('turtle1')  # Se solicita eliminar la tortuga llamada 'turtle1'
    rospy.loginfo("La tortuga por defecto fue eliminada correctamente.")  # Se informa que se eliminó
except rospy.ServiceException as e:
    # Si hay un error al eliminar, se muestra una advertencia y el error
    rospy.logwarn("No fue posible eliminar la tortuga por defecto.")
    rospy.logwarn(e)

# A partir de aquí se crean 5 tortugas nuevas en posiciones aleatorias, una por una.

#TORTUGA 1
x1 = random.uniform(1.0, 10.0)  # Se genera la posición X aleatoria
y1 = random.uniform(1.0, 10.0)  # Se genera la posición Y aleatoria
try:
    agregar_tortuga(x1, y1, 0.0, 'nueva_tortuga_1')  # Se crea la tortuga con su nombre
    rospy.loginfo(f"Tortuga 1 creada correctamente en x={x1:.2f}, y={y1:.2f}")  # Se muestra la posición
except rospy.ServiceException as e:
    rospy.logerr("No se pudo crear la tortuga 1.")  # Se reporta error si algo falla
    rospy.logerr(e)

#TORTUGA 2
x2 = random.uniform(1.0, 10.0)
y2 = random.uniform(1.0, 10.0)
try:
    agregar_tortuga(x2, y2, 0.0, 'nueva_tortuga_2')
    rospy.loginfo(f"Tortuga 2 creada correctamente en x={x2:.2f}, y={y2:.2f}")
except rospy.ServiceException as e:
    rospy.logerr("No se pudo crear la tortuga 2.")
    rospy.logerr(e)

#TORTUGA 3
x3 = random.uniform(1.0, 10.0)
y3 = random.uniform(1.0, 10.0)
try:
    agregar_tortuga(x3, y3, 0.0, 'nueva_tortuga_3')
    rospy.loginfo(f"Tortuga 3 creada correctamente en x={x3:.2f}, y={y3:.2f}")
except rospy.ServiceException as e:
    rospy.logerr("No se pudo crear la tortuga 3.")
    rospy.logerr(e)

#TORTUGA 4
x4 = random.uniform(1.0, 10.0)
y4 = random.uniform(1.0, 10.0)
try:
    agregar_tortuga(x4, y4, 0.0, 'nueva_tortuga_4')
    rospy.loginfo(f"Tortuga 4 creada correctamente en x={x4:.2f}, y={y4:.2f}")
except rospy.ServiceException as e:
    rospy.logerr("No se pudo crear la tortuga 4.")
    rospy.logerr(e)

#TORTUGA 5
x5 = random.uniform(1.0, 10.0)
y5 = random.uniform(1.0, 10.0)
try:
    agregar_tortuga(x5, y5, 0.0, 'nueva_tortuga_5')
    rospy.loginfo(f"Tortuga 5 creada correctamente en x={x5:.2f}, y={y5:.2f}")
except rospy.ServiceException as e:
    rospy.logerr("No se pudo crear la tortuga 5.")
    rospy.logerr(e)

# En este punto ya se eliminó la tortuga original y se agregaron 5 nuevas tortugas con posiciones aleatorias.
