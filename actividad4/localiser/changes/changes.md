# Actividad 4
Cambios que ha hecho el bueno de Álvaro siguiendo las notas del drive de Robótica. Todo lo he hecho esta mañana del martes mientras atendía a mi sobrina y vigilaba a los obreros, un brasileño y un colombiano. Fíate tú de esta gente y fíate tú de mi código, suerte :).

He metido también al principio del specificworker.h unos métodos para controlar al robot por teclado (no tengo joystick).
Se supone que hay un componente para hacer eso pero no he sido capaz de instalarlo bien.

El resto de cambios son cosas siguiendo el drive. He hecho los 2 primeros puntos. A partir del 3º es el trato de las multi-puertas. No prometo que lo anterior funcione bien :)

ESTO ES IMPORTANTE :D

Ahora nominal_room tiene otro vector de puertas.
Este vector guarda las puertas desde la perspectiva de la habitación, NO desde la del robot.
Para eso tenemos el vector que teníamos en el .h

## Fase 1 - Filtrado de puntos más allá de las puertas
Creo que está casi perfecto. Solo he visto 1 caso en el que no filtra bien y es un punto muy concreto al estar cerca de una esquina y cerca de una de las puertas.
Fotito del caso:

<image src="./caso1.png" height=200>

Esto pasa en ese sitio con **ese ángulo exacto**, tanto hacia la derecha como hacia la izquierda.
No sé 100% que pasa, pero si os fijáis es un punto en el que no detecta las puertas (no se pintan).
Si lanzáis la máquina de estados normal, al ser un único punto con un ángulo en concreto es difícil que se de esta situación.
No sé si pasa con más posiciones-ángulos.

### Cambios en **door_detector.cpp**:

#### *detect()*:
He probado con un threshold diferente dependiendo de la distancia. 
Más estricto si están cerca y más permisivo si están lejos. 
Antes solo ponía 1000.f.

```C++
float threshold = (((p1.distance2d + p2.distance2d) / 2.0f) > 3000.f) ? 700.f : 1000.f;
```

He probado a meter otra vez la condición para ordenar los puntos al meterlos.
Lo de meter p1 y luego p2 o p2 y luego p1 en función del ángulo.

```C++
std::get<1>(p1) <= std::get<1>(p2)
```
#### *filter_points()*:
Solucionar esta parte era una chorrada, era poner bien el emplace final.
Lo he metido en el bucle de fuera.
Estaba en el bucle de dentro y eso generaba los problemas que teníamos antes.
El offset no sé como lo voy a quedar al final, he ido probando diferentes combinaciones. Cambiadlo como queráis.
```C++
if(!erased)
    filtered.emplace_back(p);
```

## Fase 2 - Pintar las puertas en el viewer_room
OJO - Hay un método "draw_doors" justo debajo de "turn". Es para pintar las puertas con las nuevas coordenadas p1_globa y p2_global justo al localizarse (al terminal el turn).

OJO2 - Hay que tener claro cuando se trabaja con puertas desde la perspectiva del robot y con puertas desde la perspectiva de la habitación.
CREO que las puertas desde la perspectiva de la habitación es solo para pintar.

### a. El vector de puertas pasarlo a la clase de Room.
He metido en NominalRoom el nuevo vector de puertas y un getter/setter del vector.

En **goto_door** y **orient_to_door** se hacen cosas con el nuevo vector de puertas, cuidado ahí.
Hay que tocar cosas ahí seguramente.

### b. Añadir Wall y Walls a common_types
Pues eso, he añadido los nuevos tipos:
```C++
using Wall = std::tuple<Eigen::ParametrizedLine<float, 2>, int, Corner, Corner>;
using Walls = std::vector<Wall>;
```
### c. Añadir los nuevos métodos a NominalRoom
Pues eso, he metido los nuevos métodos.
Creo que están bien pero no sé al 100%.

### d. El nuevo content dentro de TURN
El código se explica mejor que yo, por lo que checken :)

### e. y f.
El 'e.' está done, es añadir unos parámetros al struct Door.

EL 'f.' no sé 100% a lo que se refiere. Dice "add the element attribute to the class".
No sé si se refiere añadir la puerta al vector de puertas de nominal room, o al de specificworker.h.


FALTA en goto_door y en orient_to_door tratar múltiples puertas :). Además de todo lo que sigue después la 'g.'














