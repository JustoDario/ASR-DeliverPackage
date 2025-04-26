# ASR-DeliverPackage
Repo for ASR final project, our project consists in making the kobuki deliver packages to people.

## Behavior Tree
_\*\***NOTE:** This is a general idea of how the final behavior tree will look like.\*\*_

![schemaBT](https://github.com/user-attachments/assets/4aff3e78-ae81-4dd2-96fe-426bb0ed0d31)

### BT's Description
This is the kobuki's delivery workflow:
1) **Check for Summons**: The kobuki first waits until it's summoned (_isSummoned_ condition)
2) **Setup Delivery**: The kobuki receives all the infomation neccessary (_destination_ and _receiver_) and initializes a delivery operation
3) **Execute Delivery**: Carries out the actual delivery process through two main phases:
  - **Travel Phase:** The robot attempts to travel to the specified destination. If that fails, returns to home base
  - **Search and Deliver Phase:** After reaching the destination, the robot:
    - Tries up to 5 times (_num\_attempts_) to:
      - Search a person
      - Verify it's the actual receiver and then deliver the package (all failures are expected to occur here)
      - Return to home base
    - If all attempts fail (receiver not found), it returns to home base

### _Notas:_
- _Sobre **Check for Summons**..._
  - _Se da por hecho que cuando lo invocan (dandole a un botón por ejemplo), el paquete ya se está posado encima del kobuki. Lo he planteado de esa manera porque así puedes dejar el paquete encima del kobuki, hasta que creas la hora de entregar el paquete. Esto también me hace pensar sobre una posible funcionalidad adicional: programar una hora de salida._
- _Sobre **Setup Delivery**..._
  - _Hay que verificar si el nombre del destino que nos dicen está registrado en la lista de waypoints._
  - _La información que se espera del receptor, podrían ser su nombre y su apellido._
  - _Se me ocurre una funcionalidad adicional: que se le pueda decir varios destinos (destinos posibles en los que pueda estar el receptor). Así, si el kobuki no encuentra al receptor en el primer destino, irá al siguiente en vez de volver directamente a la base. Se le podría concretar el orden de los destinos, o que lo decida él mismo por las distancias (primero al más cercano)._
- _Sobre **Travel Phase**..._
  - _El waypoint debería de estár en la entrada del destino, para cuando llegue a la entrada, cambie al estado de busqueda inmediatamente._
- _Sobre **Search and Deliver Phase**..._
  - _Yo la visión que planteo, es que haga lo siguiente:_
    - _Cuando llega al destino, empieza a preguntar en alto por el nombre del receptor esperado, mientras se adentra a la sala poco a poco (podríamos tener un Waypoint adicional que determine el centro de la sala)._
    - _Mientras, busca por personas que se estén acercando a él (suponiendo que el receptor se va a acercar a por el paquete)._
    - _Cuando detecte a alguien acercarse, además de poder acercarse él también a la persona, cuando la persona esté lo suficientemente cerca, le preguntará por su apellido, para verificar si es el receptor. En caso de no serlo o de ser ignorado o negado (por malinterpretación), repetirá el proceso. He puesto que repita el proceso máximo 5 veces, pero se podría hacer que repita el proceso indefinidamente: mientras busca al receptor, sigue avanzando al centro de la sala, entonces cuando llegue al centro, se dirigirá de vuelta a la salida, siguiendo buscando al receptor mientras, y si llega a la salida, vuelve a la base directamente._
    - _Cuando encuentre al receptor, le dará luz verde para coger el paquete, y entonces volverá a la base._
