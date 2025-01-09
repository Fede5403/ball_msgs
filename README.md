# Controllo Automatico delle Braccia del Robot DARwIn-OP tramite Motor Map

## Descrizione

Questo progetto riguarda l'automazione del movimento delle braccia del robot DARwIn-OP. Inizialmente, le braccia del robot venivano controllate tramite un joystick PlayStation 4. La fase successiva ha previsto l'implementazione di un sistema autonomo di controllo basato su una motor map. Questo approccio permette al robot di regolare autonomamente le braccia per bilanciare una palla all'interno di un tubo riempito d'acqua.

## Caratteristiche Principali

- **Nodo ROS `op2_motormap`**: Sviluppato per gestire l'automazione del movimento delle braccia.
- **Messaggio Personalizzato `ball_msgs/ball`**: Contiene informazioni sulla distanza e la velocità della palla rilevate dal sistema di visione del robot.
- **Integrazione con il Sistema di Visione**: Il sistema di visione pubblica dati in tempo reale che vengono utilizzati per calcolare gli angoli articolari delle braccia tramite l'algoritmo della motor map.

## Struttura del Progetto

### Definizione del Messaggio Personalizzato

Per facilitare la comunicazione tra il sistema di visione e il nodo di motor map, è stato definito un messaggio ROS personalizzato `ball_msgs/ball`:

```plaintext
float32 distance
float32 velocity

LISTING 3.4: Definizione di ball_msgs/ball.msg

Pubblicazione dei Dati e Integrazione con ROS

I dati elaborati — specificamente la distanza della palla dal centro e la sua velocità — vengono incapsulati nel messaggio ROS personalizzato ball_msgs/ball. Il sistema di visione pubblica questo messaggio sul topic /ball_topic a una frequenza sincronizzata con il rate di cattura dei frame. In questo modo, le informazioni sono immediatamente disponibili per gli altri nodi all’interno della rete ROS, in particolare per il sistema di controllo della motor map, che si sottoscrive a questo topic per ricevere dati sensoriali in tempo reale.

Requisiti
	•	ROS (Robot Operating System): Versione compatibile con il nodo sviluppato.
	•	DARwIn-OP Robot: Configurato e funzionante.
	•	Sistema di Visione Integrato: Per la rilevazione della palla.
	•	Joystick PlayStation 4 (solo per la fase iniziale di controllo manuale).

Installazione
	1.	Clonare il Repository

git clone https://github.com/alessandonicosia/darwin-op-motormap-control.git


	2.	Navigare nella Directory del Progetto

cd darwin-op-motormap-control


	3.	Installare le Dipendenze Necessarie
Assicurarsi di avere tutte le dipendenze ROS installate. È possibile utilizzare rosdep per installarle automaticamente:

rosdep install --from-paths src --ignore-src -r -y


	4.	Compilare il Progetto

catkin_make


	5.	Sorgere l’Ambiente di Lavoro

source devel/setup.bash



Utilizzo
	1.	Avviare il Sistema di Visione
Assicurarsi che il sistema di visione sia attivo e pubblica correttamente i messaggi ball_msgs/ball sul topic /ball_topic.
	2.	Avviare il Nodo Motor Map

rosrun op2_motormap op2_motormap_node


	3.	Monitorare il Movimento delle Braccia
Il nodo op2_motormap elaborerà i dati ricevuti e controllerà automaticamente le braccia del robot per bilanciare la palla all’interno del tubo.

Autore
	•	Alessandro Nicosia

Nota

Questo README è stato redatto utilizzando ChatGPT, un modello di linguaggio sviluppato da OpenAI.

