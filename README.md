# Upozornění

Jedná se o repozitář sloužící především k otestování řídích algoritmů, které byli vytvořeny během bakalářské práce.  Repozitář obsahuje zároveň použítý [f1tenth_simulator](https://github.com/f1tenth/f1tenth_simulator) od [F1TENTH](https://f1tenth.org/), který zde slouží k otestování právě navržených algoritmů.

# Nastavení pro otestování v simulátoru

Ke zprovoznění je zapotřebí mít na počítači nainstalovaný operační systém Linux. Simulátor byl odzkoušen konkrétně na Ubuntu 20.04 a Ubuntu 18.04, proto by bylo nejvhodnější použít jednu z těchto distribucí. Dále je zapotřebí mít na počítači nainstalovaný softwarový rámec [ROS](https://www.ros.org/), nejlépe však Melodic. Dále je zapotřebí nainstalovat balíčky, které jsou nutné pro spuštění simulátoru. Nejdříve tak aktualizujeme dostupné balíčky a jejich verze pomocí příkazu:
```
$ sudo apt-get update
```
Poté se balíčky jednoduše nainstalují konkrétně pro ROS Melodic pomocí příkazů v terminálu:
```
$ sudo apt-get install ros-melodic-tf2-geometry-msgs
$ sudo apt-get install ros-melodic-ackermann-msgs
$ sudo apt-get install ros-melodic-joy
$ sudo apt-get install ros-melodic-map-server
```
Dále je potřeba stáhnout Python knihovny  [scipy](https://scipy.org/) a [scikit-image](https://scikit-image.org/) pro spuštění algoritmu vygenerování trajektorie. Všechny potřebné knihovny se dají nainstalovat pomocí dvou příkazů: 
```
$ sudo apt-get install python-skimage
$ sudo apt-get install python-scipy
```
Poslední části, která je důležitá pro otestování řídícího algoritmu je nainstalování dříve zmíněných knihoven [OSQP](https://osqp.org/), [Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page) a [osqp-eigen](https://github.com/robotology/osqp-eigen). Celý proces instalace knihovny [OSQP](https://osqp.org/) je popsán přímo v [návodu](https://osqp.org/docs/get_started/sources.html). Stejně tak nainstalování knihovny [osqp-eigen](https://github.com/robotology/osqp-eigen),  kde návod na instalaci je popsán rovnou v repositáři. Nakonec knihovna [Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page) se nainstaluje pomocí příkazu: 
```
$ sudo apt-get install libeigen3-dev
```
Po nainstalovaní všech těchto knihoven se bude následně dát celý workspace přeložit pomocí následujících příkazů:
```
$ cd ~/simulation_ws
$ catkin_make
$ source devel/setup.bash
```
Po úspěšném přeložení je už možné vyzkoušet simulátor, generování trajektorie a řídící algoritmus pomocí sady příkazů, kde každý příkaz bude zadán v jiném terminálu: 
```
$ roslaunch f1tenth_simulator simulator.launch
$ rosrun navigation line.py
$ roslaunch navigation controller.launch
```
První příkaz spouští simulátor, druhý generování trajektorie a třetí spouští řídící algoritmus. Jelikož řídící algoritmus vysílá řídící vstupy na topik `/nav` je potřeba v terminálu ve kterém je spuštěn simulátor napsat písmeno **n** , které zajistí že autíčko bude přijímat zprávy z topiku `/nav`, kde by následně autíčko by mělo sledovat trajektorii.

