14 mai 2018
Tentative de comprendre pourquoi on n'observe aucun frémissement sur la IHM08
La lecture des intensités et courant a l'air de fonctionner mais c'est comme si aucun courant n'était envoyé au moteur.
Je reviens alors à la IHM07 en me disant qu'il faut expliquer :
- pourquoi la troisième itensité est différente des autres à l'échantillonage temps réél (quasiment plate). 
Est-ce un problème matériel de la carte F446 (En échangeant les pattes programmatiquement, il semble que ça ne vienne pas de l'IHM07)?  
Le moteur ne fonctionnerait-il que sur deux phases ?
- pouquoi la tension de la phase 3 est différente à vide ? 
Tentative d'ajout d'un ajustement sur cette phase. Ne change rien au fonctionnement.

29 mars 2018
Bout de code pour lire la valeur du potentiomètre inclus sur les IHM07m1 et assigner au rapport cyclique. 
Test à la porte d'embarquement à Stansted que ça fonctionne (j'ai trouvé une prise murale !), mais le démarrage n'est pas propre et ne fonctionne que sur une plage réduite.

avantages liés au portage en c++ envisagés :
- strong/named types (std::chrono, duty cycle, voltage, intensité, radians etc.)
- templates + constexpr pour zero/negative cost init?
- constexpr + namespace pour moins de macro de définition des constantes
- templates pour moins de #ifdef
- namespaces (foc et bldc par ex., pour la configuration)
- RAII pour des blocs de contextes (DCCALL_ON/DCCALL_OFF, désactivation du WDG etc.)
- réduction du nombre de macros, plutôt des fonctions inline et des templates
- zero cost lambdas?
- strong enums
- utilisation de references plutôt que des pointeurs
- classes : moins de static partout, volatile sur des struct entières
- classes : meilleure structuration du code ?
- meilleurs outils d'analyse statique (clang tidy etc.) ?

27 mars 2018
- parviens à faire tourner proprement en BLDC sur f446 + ChibiOS 4
- La calibration BLDC fonctionne pour la première fois : j'ai regardé de plus près les instructions de la bulle d'aide et 
augmenté sensiblement ERPM (1600) et duty cycle (19%), I reste autour de 1A.

26 mars 2018
découvre une incohérence entre le canal ADC assigné à la phase 3 et la capacité ADC de la patte correspondante (qui n'est reliée qu'à ADC1,2) en regardant de plus près la datasheet.
Ne peut tester en vrai avant demain car j'ai oublié au bureau l'adaptateur de prise britanique.

