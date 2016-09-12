.. _introducción:

==================
Introducción Copter
==================

Copter es un sistema avanzado de autopiloto avanzado para multirotores,
helicopteros, y otros vehiculos con rotores.

Overview
========

Copter es una solución open-source de auto-piloto para vehiculos
multi-rotor, que ofrece tanto el vuelo mejorado a traves de control remoto
(a traves de modos de vuelo inteligentes) y la ejecución de misiones
totalmente autónomas.

Como parte de la plataforma de software AuduPilot, funciona perfectamente 
con el software de Estación Terrena, que puede monitorear la telemetría del
vehículo y realizar actividades de planificación de misiones de vuelo. También
se beneficia de otras partes de la plataforma DroneCode, incluyendo simuladores,
herramienta de análisis de log, y APIs de alto nivel para manejo y control 
del vehiculo.

Copter está a la vanguardia de la robótica aérea y diseñada para aquellas 
personas que quieran probar la tecnología avanzada, técnicas de última generación
y nuevos estilos de vuelo. Es la plataforma preferida de numerosos vehiculos
disponibles en el mercado listos para volar (Ready-To-Fly), y se puede integrar
facilmente para mejorar sus vehiculos multirotores hechos por usted mismo (DIY)

.. image:: ../images/copter-introduction-diagram.jpg
    :target: ../_images/copter-introduction-diagram.jpg

Principales Características
============

Las características principales incluyen:

-  *Nivelación automática y Control automático de altitud de alta calidad*: Fly level and
   Vuelo nivelado o utilize el modo "Simple Flight", que hace que Copter sea uno 
   de los mas faciles para volar Vehiculos Multirotores

   No se preocupe de mantener la vista en al orientación del vehículo -
   solamente empuje el stick a donde quiere ir, y el autopiloto reconocerá
   lo que eso significa sin importar la orientación que tenga el vehículo, utilizando
   la brújula abordo. "Adelante", "Atras" ... A quien le importa!
   
-  *Despegue y Aterrizaje Autónomo*: Accionar un interrupto y vea el Vehículo Copter 
   ejectur su misión completamente autónomo, su regreso a casa y aterrizaje 
   por sí mismo cuando este listo.
-  *Modo "Loiter" *: El vehículo "Copter" mantendra su posición usando su GPS
   y sensores de altitud.
-  *Regreso a donde despegó (Return to launch)*: Accionar un interruptor para que  
   el vehiculo "Copter" vuele de regerso al lugar de despegue automaticamente.
-  *Seguridad Contra Fallos:* Automaticamente detecta cuando el vehículo pierda
   contacto de transmisión (o este por fuera de una GeoValla definida) y 
   regresará al punto de despegue. También tratará de aterrizar con seguridad si
   se detectan fallos de hardware.
-  *No se requiere programación*: Use el software *Mission Planner* 
   para conectarse al autopiloto (con solo un "click") y configure "Copter". 
   El software Mission Planner (y otras estaciones terrenas compatibles) 
   proporcionan información visual del estado del vehículo, configuración y 
   telemetría, incluyendo una interfaz de planificación "point-and-click"
-  *Misiones con cientos de puntos GPS*: Solamente de click y se generara un
   punto de destino (WayPoint) en el software "Mission Planner", y Copter volará
   a este punto por si mismo. Usted puede automatizar totalmente la misión, 
   incluyendo control de camara! Los únicos limites de distancia son la 
   fuente de alimentación del vehículo.
-  *Planificación de la misión durante el vuelo*: Usando una conexión inalámbrica
   de dos vías, punto de destino(WayPoints), cambio de modos, incluso se pueden hacer  
   cambio de todos los valores de los parámetros de control desde su portatil o 
   dispositivo movíl - incluso mientras el vehículo está en el aire!

Empezando
===============

Si está utilizando "Copter" en un vehículo listo para volar (RTF), es probable 
que ya este listo, configurado y ajustado, listo para su primer vuelo. Nosotros
recomendamos que *lea las instrucciones del fabricante*
en particular lo relacionado con seguridad, antes de volar.

Una vez que este familiarizado con la configuración por defecto de sus vehiculos
es posible que desee configurar su Radio Control y el transmisor del Vehículo
:ref:`Modos de Vuelo <flight-modes>`, 
o :ref:`Elegir una Estación Terrena <common-choosing-a-ground-station>`
y empezar a realizar vuelos con misiones autónomas.

.. tip::

   Sea que utilize un vehículo listo para volar (RTF) o hecho por usted mismo (DIY), 
   los vehículos autónomos son potencialmente peligroso! Siempre siga 
   :ref:`Mejores Practicas de seguridad <safety-multicopter>` y ponga mucha atención
   a todas las alarmas de seguridad.

Si esta trabajando en un proyecto DIY, este wiki tiene todo lo que usted necesita!
Debe empezar por leer esta sección con el fin de entender lo que puede hacer
un multirotor, como seleccionar una estructura, Tarjeta del controlador de Vuelo,
y otros componentes esenciales. Puede seguir con :ref:`Configuración por Primera Vez <initial-setup>` 
para aprender como ensamblar un "Copter" y despues 
:ref:`Primer Vuelo <flying-arducopter>` para aprender como configurar y ajustar su vehículo.

El Equipo de Desarrollo
====================

Copter es desarrollado y mantenido por un grupo de voluntarios 
de la comunidad de código abierto. Siga sus esfuerzos continuos y lea sobre 
los nuevos proyectos de desarrollo en `DIYDrones.com <http://diydrones.com>`__.

*Todos los que participamos en este proyecto nos preocupamos mucho
acerca de la privacidad y seguridad de lo que compartimos con el planeta.
Por favor, administre bien esta tecnología. Es un producto de muchos atardeceres y fines 
de semana, lo hacemos disponible para un uso benévolo.

Mas información acerca de Copter
=======================

Para mas información acerca de Copter y sus principales configuraciónes
por favor lea los siguientes temas:


.. toctree::
    :maxdepth: 1

    Como funciona un Multirotor <what-is-a-multicopter-and-how-does-it-work>
    Que necesita <what-you-need>
    Seguridad del MultiCopter <safety-multicopter>
    Elegir una estructura de MultiCopter <choosing-a-frame>
    Elegir un Controlador de Vuelo <common-choosing-a-flight-controller>
    Elegir una Estación Terrena <common-choosing-a-ground-station>
    Casos de Uso  <copter-use-case-overview>
