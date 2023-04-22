# INF3990 - ÉQUIPE 02 - PROJET INTÉGRATEUR 3

### Prérequis pour faire marcher le projet :
    - Docker 
    - Docker-Compose
    - 2 robots Limos
    - Rouler l'interface dans un environnement Ubuntu 20.04
    - Ros Noetic avec la configuration complète

### Interface 

L'interface contient le serveur statique qui va distribuer le visuel de la page Web et le serveur qui va traiter les requêtes de l'utilisateur et qui va communiquer avec les robots 


##### Utilisation 

1. Aller dans le dossier **Interface**
2. Lancer la commande `sudo ./start-interface`

Une interface est disponible sur `localhost:4200` pour pouvoir interagir avec les Limos. 
Le server roule sur le port 9330.

### Limo 

Le dossier Limo contient le code Docker qui va rouler sur les machines Limos. Ce code sert essentiellement à ouvrir une connextion entre le robot Limo et le serveur de la station au sol. De cette façon, l'utilisateur pourra envoyer des requêtes aux robots et ceux-ci pourront les traiter. Le code dépense du pacquet [Rosbridge](http://wiki.ros.org/rosbridge_suite) et de [ROS noetic](http://wiki.ros.org/noetic). Il faut s'assurer que les Limos possèdes ces progiciels.

##### Utilisation 

1. Aller dans le dossier **Limo/ros-server**
2.  Lancer la commande `sudo ./start-limo`

Le containteur roule sur le Limo et une connexion est établie avec les serveurs sur le port 9332.

### Simulation 

#### Prérequis à la simulation 

1. [La librairie limo_ros](https://github.com/agilexrobotics/limo_ros)
2. [La librairie ugv_gazebo_sim](https://github.com/agilexrobotics/ugv_gazebo_sim)
3. [La librairie Move_Base](http://wiki.ros.org/move_base)
4. [La librairie Rosbridge](http://wiki.ros.org/rosbridge_suite)
5. [La librairie Gmapping](http://wiki.ros.org/gmapping) 
6. [La librairie AMCL](http://wiki.ros.org/amcl)
7. [La librairie multirobot_map_merge](http://wiki.ros.org/action/fullsearch/multirobot_map_merge?action=fullsearch&context=180&value=linkto%3A%22multirobot_map_merge%22)

Pour la simulation, il est important d’avoir un dossier avec toutes les dépendances du
projet et de les avoir construit avec la commande suivante : `catkin_make`

#### Lancement des Limos 

Pour lancer les conteneur des robots en simulation, c’est la même commande que pour
les robots physiques sauf qu’il faut ajouter à celle-ci le paramètre « 1 » et le chemin
d’accès vers le fichier « setup.bash » des librairies de dépendances (e.g./home/user/Document/simu/lib/devel/setup.bash). 
La commande est la suivante : `sudo ./start-limo.sh 1 <chemin du fichier setup.bash des dépendances>`

#### Lancement de l'interface 

Pour l’interface utilisateur, il faut se diriger dans le dossier Interface depuis la racine du
projet. Il exécuter le fichier « start-interface.sh » et lui passer le paramètre « 1 ». 
La commande est la suivante : `sudo ./start-interface.sh 1`
Les adresses IP à rentrer sont ceux de la machine qui a lancé la simulation.


Avec cela, la simulation Gazebo va se lancer et l'interface sera disponible sur le port 4200. 
Note qu'il est très important de laisser les serveurs du coté robot se déployer compeltement avant de se connecter avec l'interface

### Tests 

Les tests unitaires ont été fait sur une machine Ubuntu 20.04 sur Firefox. 

#### Prérequis

1. Node.js verison 18.03 
2. Ubuntu 20.4 avec Firefox comme navigateur 

#### Test interface

Des tests unitaires sont disponibles pour le client dans le dossier /Interface/client depuis la racine du projet. 
Il faut simplement exécuter: 

1. `npm i` pour installer les dépendances
2. `npm run test` pour lancer les tests client

Des tests pour le backend sont aussi disponible. Il faut galler dans le dossier /Interface/server depuis la racine du projet. 
Il faut simplement exécuter: 

1. `npm i` pour installer les dépendances
2. `npm run test` pour lancer les tests serveurs

#### Test Limo

Des tests pour le code Typescript est disponible dans le dossier /Limo/ros-server depuis la racine. 
Il faut exécuter les commandes suivante :

1. `npm i` pour installer les dépendances
2. `npm run test` pour lancer les tests

###### Un fichier Tests.pdf est disponible à la racine pour expliquer les tests qui ont été faits sur les noeuds qui n'ont pas pu être tester unitairement. 







