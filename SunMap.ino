#include <TaskScheduler.h>
#include <TaskSchedulerDeclarations.h>
#include <TaskSchedulerSleepMethods.h>
#include <Servo.h>
#include <Smoothed.h>

#define MAX_ITERATION 4     //numero iterazioni per ciclo (preferibilmente multipli di 4);
#define MAX_CICLE 5         //numero cicli
#define STEP 10         //numero gradi utilizzati come singolo passo per il movimento dei servo 
#define INTERVAL 20000   //intervallo temporale tra i cicli in ms
#define PANEL A0        //pin al quale è collegato il pannelo solare

Servo base;
Servo arm;
Scheduler runner;
Smoothed <double> smooth_exp;

int target[2] = {0, 0};   //target dei servo (0 = base, 1 = arm)
double actualVoltage = 0;
double prevVoltage = 0;
int actualTarget = 0;   //indice del servo che si sta ottimizzando
int actualDirection = +1;
bool baseOnTarget = false;
bool armOnTarget = false;
int count_iteration = 0;
int count_cicle = 0;
int optimal[2] = {0, 0};  //angoli migliori attuali ( 0 = base, 1 = arm) calcolati come media pesata sul voltaggio
long voltage_sum = 0;      //somma dei voltaggi delle posizioni migliori di ogni ciclo ( utilizzato per il calcolo delgi angoli)                                

void moveBase();
void moveArm();
void readVoltage();
void optimize();
void revertTarget();
void updateTraget();
void changeDirection();
bool isTargetIllegal();

Task moveBaseTask(TASK_MILLISECOND * 10, TASK_FOREVER, moveBase);
Task moveArmTask(TASK_MILLISECOND * 10, TASK_FOREVER, moveArm);
Task readVoltageTask(TASK_MILLISECOND*2, 500, readVoltage);
Task optimizeTask(TASK_MILLISECOND, TASK_FOREVER, optimize);

void moveBase() {
  if (target[0] <= 180 && target[0] >= 0) {
    //Serial.println("Move base");
    if (base.read() < target[0]) {
      base.write(base.read() + 1);
    }
    if (base.read() > target[0]) {
      base.write(base.read() - 1);
    }
    if (base.read() == target[0]) {
      //Serial.print("Target base raggiunto: ");
      baseOnTarget = true;
      moveBaseTask.disable();
      if(!readVoltageTask.isEnabled() && armOnTarget){
        readVoltageTask.restart();
      }
    }
  } else {
    Serial.println("Out of bound base target");
  }
}

void moveArm() {
  if (target[1] <= 180 && target[1] >= 0) {
    //Serial.println("Move arm");
    if (arm.read() < target[1]) {
      arm.write(arm.read() + 1);
    }
    if (arm.read() > target[1]) {
      arm.write(arm.read() - 1);
    }
    if (arm.read() == target[1]) {
      //Serial.print("Target arm raggiunto: ");
      armOnTarget = true;
      moveArmTask.disable();
      if(!readVoltageTask.isEnabled() && baseOnTarget){
        readVoltageTask.restart();
      }
    }
  } else {
    Serial.println("Out of bound arm target");
  }
}

void readVoltage() {
  if(readVoltageTask.isFirstIteration()){
    smooth_exp.clear();
  }
  int value = analogRead(PANEL);
  //Serial.println(value);
  smooth_exp.add(value);
  if(readVoltageTask.isLastIteration()){
    prevVoltage = actualVoltage;
    actualVoltage = smooth_exp.get();
    Serial.print("Base: ");
    Serial.println(base.read());
    Serial.print("Arm: ");
    Serial.println(arm.read());
    Serial.print("Voltage attuale: ");
    Serial.println(actualVoltage);
    Serial.print("Voltage precedente: ");
    Serial.println(prevVoltage);
    Serial.println();
    optimizeTask.enable();
  }
}

void optimize(){
  if(actualVoltage <= prevVoltage){   //se il voltaggio non è migliorato cambio direzione di ottimizzazione
    revertTarget();
    actualVoltage = prevVoltage;
    changeDirection();
  }
  if(count_iteration < MAX_ITERATION){     //se ho ancora delle direzioni da ottimizzare avvio il movimento
    updateTarget();
    baseOnTarget = false;
    armOnTarget = false;
    moveBaseTask.enable();
    moveArmTask.enable();
  }else{                            // altrimenti significa che ho concluso l'ottimizzazione e quindi stampo lo stato attuale, aggiuorno i valori ottimali assoluti e resetto i dati
    Serial.println("Posizione ottimale raggiunta");
    Serial.print("Base: ");
    Serial.println(target[0]);
    optimal[0] = (long)(optimal[0] * voltage_sum + target[0] * actualVoltage)/(voltage_sum + actualVoltage);
    Serial.print("Arm: ");
    Serial.println(target[1]);
    optimal[1] = (long)(optimal[1] * voltage_sum + target[1] * actualVoltage)/(voltage_sum + actualVoltage);
    Serial.print("Voltage: ");
    Serial.println(actualVoltage);
    voltage_sum += actualVoltage;
    Serial.println();
    count_iteration = 0;
    count_cicle++;
    actualVoltage = 0;
    prevVoltage = 0;
    if(count_cicle < MAX_CICLE){
      moveBaseTask.enableDelayed(INTERVAL);
      moveArmTask.enableDelayed(INTERVAL);
    }
    else{
      Serial.println("Posizione ottimale assoluta calcolata");
      Serial.print("Base: ");
      Serial.println(optimal[0]);
      Serial.print("Arm: ");
      Serial.println(optimal[1]);
    }
  }
  optimizeTask.disable();
}

void setup() {
  Serial.begin(9600);
  base.attach(5);
  arm.attach(6);
  runner.init();
  runner.addTask(moveBaseTask);
  runner.addTask(moveArmTask);
  runner.addTask(readVoltageTask);
  runner.addTask(optimizeTask);
  smooth_exp.begin(SMOOTHED_EXPONENTIAL, 20);
  moveBaseTask.enable();
  moveArmTask.enable();
  
}

void loop() {
  runner.execute();
}

void revertTarget(){
  target[actualTarget] = (target[actualTarget] - STEP * actualDirection + 180 + STEP) % (180 + STEP);
}

void updateTarget(){
  target[actualTarget] = (target[actualTarget] + STEP * actualDirection + 180 + STEP) % (180 + STEP);
}

void changeDirection(){
  if(actualDirection < 0){
    actualTarget = (actualTarget + 1) % 2; 
  }
  actualDirection = -actualDirection;
  count_iteration++;
}
