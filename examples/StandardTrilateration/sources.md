Cet exemple permet de positionner deux tags en 2D avec 3 ancres au sol. 
Il est possible d'ajouter d'avantage de tags en : 
- ajoutant un code "tag" avec un personal_short_address et EUI différent
- ajoutant cet identifiant à la liste tags[] du code main


### Échanges de messages : 

0. - 1 : data from Main (transmitStartRanging) -> indique au premier tag de commencer le TWR avec la première ancre

1. - Poll from the tag 
2. - ACTIVITY_CONTROL from the Anchor with ACK request (transmitResponseToPollACK) -> envoie un message au tag avec un champ spécifique de manière à ce qu'il réponde automatiquement. Cette réponse automatique évite un échange sur le bus SPI, et nécessite des conditions spécifiées dans la documentation. 
3. - Automatic ACK response from the Tag 
4. - ranging confirm from the Anchor, with timing data (transmitRangingConfirmExtended)


### Notes d'implémentation de l'ACK automatique : 
On implémente l'envoi automatique de messages par le module en lui-même.

Il faut l'activer dans le filtrage du récepteur (enableFrameFilteringACK)
Il faut envoyer un message dont le premier octet est DATA mais avec le 5ème bit à 1 (DATA_ACK) (transmitResponseToPollACK)
A la réception, l'ACK est automatique, donc il suffit de récup le timestamp de réception et d'émission en même temps.
La réception de l'ACK est plus tendue. On utilise DW1000Ng::setWait4Response(10) pour passer rapidement (10µs) de l'émission à la réception. Il ne faut pas utiliser la fonction de base DW1000NgRTLS::receiveFrame() car elle appelle DW1000Ng::startReceive() qui reset le buffer (qui pourrait être déjà remplit tellement la réponse est arrivée rapidement). On a donc créé receiveFrameACK(). Le message est donc légèrement différent : longueur de 3 octet.


