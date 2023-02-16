# INF3990 - ÉQUIPE 02 - PROJET INTÉGRATEUR 3

### Prérequis pour faire marcher le projet :
    - Docker 
    - Docker-Compose
    - 2 robots Limos

### Interface 

L'interface contient le serveur statique qui va distribuer le visuel de la page Web et le serveur qui va traiter les requêtes de l'utilisateur et qui va communiquer avec les robots 

##### Variables d'environnement 
    - ***LIMO_IP_1*** : Addresse IP du Limo avec l'identifiant 1 (*LIMO_ID=1*)
    - ***LIMO_IP_2*** : Addresse IP du Limo avec l'identifiant 2 (*LIMO_ID=2*)

##### Utilisation 

    1. Aller dans le dossier **Interface**
    2. Repérer le variables d'environnement
    3. Lancer la commande `./start-interface ADDRESSE IP DU LIMO 1> <ADDRESSE IP DU LIMO 2>`
Une interface est disponible sur `localhost:4200` pour pouvoir interagir avec les Limos. 
Le server roule sur le port 9330.

### Limo 

Le dossier Limo contient le code Docker qui va rouler sur les machines Limos. Ce code sert essentiellement à ouvrir une connextion entre le robot Limo et le serveur de la station au sol. De cette façon, l'utilisateur pourra envoyer des requêtes aux robots et ceux-ci pourront les traiter. Le code dépense du pacquet [Rosbridge](http://wiki.ros.org/rosbridge_suite) et de [ROS noetic](http://wiki.ros.org/noetic). Il faut s'assurer que les Limos possèdes ces progiciels.

##### Variables d'environnement 
    - ***LIMO_IP*** : Addresse IP du Limo
    - ***LIMO_ID** : Identifiant du Limo ; Chaque Limo doit possèder un identifiant différent des autres Limos et doit être 1 ou 2 (Max 2 Limos ici)

##### Utilisation 

    1. Aller dans le dossier **Limo/ros-server**
    2. Lancer la commande `docker build -t ros-server-102 .` pour construire le container
    3. Rouler les commandes pour rouler le **ROS MASTER** et **ROSBRIDGE**
    3. Repérer le variables d'environnement
    4. Lancer la commande `docker run -p 9332:9332 -e LIMO_ID=<IDENTIFIANT DU LIMO> -e LIMO_IP=<AdDRESSE IP DI LIMO> ros-server-102`

Le containteur roule sur le Limo et une connexion est établie avec les serveurs sur le port 9332.





