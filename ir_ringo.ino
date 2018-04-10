#include "IRremote.h"
#include "ir_command_codes.h"
#include <Servo.h>

// Аналоговый вход контроллера, к которму подключен ИК-приёмник.
const int IR_PIN = A0;

// Цифровые выводы контролера, к которым подключены серводвигатели.
const int LEFT_SERVO_PIN = 2;
const int CENTRAL_SERVO_PIN = 4;
const int RIGHT_SERVO_PIN = 7;

// Центральное ("нулевое") положение серводвигателей в градусах.
const long LEFT_SERVO_ZERO_VALUE = 90;
const long RIGHT_SERVO_ZERO_VALUE = 90;
const long CENTRAL_SERVO_ZERO_VALUE = 90;

// Амплитула левого и правого серводвигателей.
const long SIDE_SERVOS_FULL_AMPLITUDE = 30;
// Уменьшенная амплитула левого и правого серводвигателей. Используется
//     при поворотах совмещённых с движением вперёд или назад.
const long SIDE_SERVOS_HALF_AMPLITUDE = 15;
// Амплитула центрального серводвигателя.
const long CENTRAL_SERVO_AMPLITUDE = 20;

// Периоды колебаний для различных скоростей.
const long STEP_PERIOD_VERY_SLOW = 2000;
const long STEP_PERIOD_SLOW = 1500;
const long STEP_PERIOD_FAST = 1000;
const long STEP_PERIOD_VERY_FAST = 500;

long lastMillis;
long globalPhase;
float angleShiftLeftServo;
float angleShiftRightServo;
float angleShiftCentralServo;
long stepPeriod;
long amplitudeLeftServo;
long amplitudeRightServo;
boolean isAttached;
boolean isStopped;

// Создаём объект ИК-приёмник. Этот объект принимает и декодирует ИК-сигналы от пульта.
IRrecv irrecv(IR_PIN);

Servo LeftServo;
Servo RightServo;
Servo CentralServo;

void attachServos() {
  if (!isAttached) {
    LeftServo.attach(LEFT_SERVO_PIN);
    RightServo.attach(RIGHT_SERVO_PIN);
    CentralServo.attach(CENTRAL_SERVO_PIN);
    isAttached = true;
  }
}

// В некоторых положениях серводвигатели могут вибрировать и шуметь.
//     Чтобы это избежать во время остановок робота, сервы надо отключать.
void detachServos() {
  if (isAttached) {
    LeftServo.detach();
    RightServo.detach();
    CentralServo.detach();
    isAttached = false;
  }
}

void setup() {
  // Начинаем прослушивание ИК-сигналов.
  irrecv.enableIRIn();
 
  attachServos();
  isStopped = true;
  lastMillis = millis();

  angleShiftLeftServo = 0;
  angleShiftRightServo = 0;
  angleShiftCentralServo = 0;
  
  stepPeriod = STEP_PERIOD_FAST;
}

// Получение угла для серводвигателя.
//     Параметр amplitude - амплитуда колебаний,
//     Параметр phaseMillis - текущая продолжительность колебаний,
//     Параметр shiftAndle - фаза колебаний.
int getAngle(long amplitude, long phaseMillis, float shiftAngle) {
  float alpha = 2 * PI * phaseMillis / stepPeriod + shiftAngle;
  float angle = amplitude * sin(alpha);
  return (int)angle;
}

template<typename T,size_t N>
boolean hasCode(T (&commandCodes)[N], long code) {
  for (int i = 0; i < N; i++) {
    if (commandCodes[i] == code) {
      return true;
    }
  }
  return false;
}

void loop() {
  long millisNow = millis();
  long millisPassed = millisNow - lastMillis;
  if (isStopped) {
    // Ждём полсекунды, чтобы серводвигатели вышли в нулевое положение и отключаем их.
    if (millisPassed >= 500) {
      lastMillis = 0;
      detachServos();
    }

    globalPhase = 0;
  } else {
    lastMillis = millisNow;
    
    globalPhase += millisPassed;
    globalPhase = globalPhase % stepPeriod;
  }
  
  // Описываем структуру results, в которую будут помещаться
  //     принятые и декодированные ИК-команды:
  decode_results results;
  
  // Если команда успешно принята и декодирована, то мы можем её обрабатывать.
  if (irrecv.decode(&results)) {
    
    if (hasCode(IR_COMMAND_FORWARD_CODES, results.value) ||
        hasCode(IR_COMMAND_FORWARD_LEFT_CODES, results.value) ||
        hasCode(IR_COMMAND_FORWARD_RIGHT_CODES, results.value)) {
    
      attachServos();
      isStopped = false;
      angleShiftLeftServo = 0;
      angleShiftRightServo = 0;
      angleShiftCentralServo = PI/2;
        
      amplitudeLeftServo = SIDE_SERVOS_FULL_AMPLITUDE;
      amplitudeRightServo = SIDE_SERVOS_FULL_AMPLITUDE;
      if (hasCode(IR_COMMAND_FORWARD_LEFT_CODES, results.value)) {
        amplitudeLeftServo = SIDE_SERVOS_HALF_AMPLITUDE;
      } else if (hasCode(IR_COMMAND_FORWARD_RIGHT_CODES, results.value)) {
        amplitudeRightServo = SIDE_SERVOS_HALF_AMPLITUDE;
      }
    } else if(hasCode(IR_COMMAND_BACKWARD_CODES, results.value) ||
              hasCode(IR_COMMAND_BACKWARD_LEFT_CODES, results.value) ||
              hasCode(IR_COMMAND_BACKWARD_RIGHT_CODES, results.value)) {

      attachServos();
      isStopped = false;
      angleShiftLeftServo = 0;
      angleShiftRightServo = 0;
      angleShiftCentralServo = -PI/2;

      amplitudeLeftServo = SIDE_SERVOS_FULL_AMPLITUDE;
      amplitudeRightServo = SIDE_SERVOS_FULL_AMPLITUDE;
      if (hasCode(IR_COMMAND_BACKWARD_LEFT_CODES, results.value)) {
        amplitudeRightServo = SIDE_SERVOS_HALF_AMPLITUDE;
      } else if (hasCode(IR_COMMAND_BACKWARD_RIGHT_CODES, results.value)) {
        amplitudeLeftServo = SIDE_SERVOS_HALF_AMPLITUDE;
      }
    } else if (hasCode(IR_COMMAND_TURN_LEFT_CODES, results.value)) {
      attachServos();
      isStopped = false;
      angleShiftLeftServo = 0;
      angleShiftRightServo = PI;
      angleShiftCentralServo = -PI/2;
      amplitudeLeftServo = SIDE_SERVOS_FULL_AMPLITUDE;
      amplitudeRightServo = SIDE_SERVOS_FULL_AMPLITUDE;
    } else if (hasCode(IR_COMMAND_TURN_RIGHT_CODES, results.value)) {
      attachServos();
      isStopped = false;
      angleShiftLeftServo = 0;
      angleShiftRightServo = PI;
      angleShiftCentralServo = PI/2;
      amplitudeLeftServo = SIDE_SERVOS_FULL_AMPLITUDE;
      amplitudeRightServo = SIDE_SERVOS_FULL_AMPLITUDE;
    } else if (hasCode(IR_COMMAND_STOP_CODES, results.value)) {
      attachServos();
      isStopped = true;
      angleShiftLeftServo = 0;
      angleShiftRightServo = 0;
      angleShiftCentralServo = 0;
      amplitudeLeftServo = SIDE_SERVOS_FULL_AMPLITUDE;
      amplitudeRightServo = SIDE_SERVOS_FULL_AMPLITUDE;
    } else if (hasCode(IR_COMMAND_VERY_SLOW_CODES, results.value)) {
      // Корректировка globalPhase чтобы сохранить положение серв при смене периода колебаний ног.
      globalPhase = globalPhase * STEP_PERIOD_VERY_SLOW / stepPeriod;
      stepPeriod = STEP_PERIOD_VERY_SLOW;
    } else if (hasCode(IR_COMMAND_SLOW_CODES, results.value)) {
      globalPhase = globalPhase * STEP_PERIOD_SLOW / stepPeriod;
      stepPeriod = STEP_PERIOD_SLOW;
    } else if (hasCode(IR_COMMAND_FAST_CODES, results.value)) {
      globalPhase = globalPhase * STEP_PERIOD_FAST / stepPeriod;
      stepPeriod = STEP_PERIOD_FAST;
    } else if (hasCode(IR_COMMAND_VERY_FAST_CODES, results.value)) {
      globalPhase = globalPhase * STEP_PERIOD_VERY_FAST / stepPeriod;
      stepPeriod = STEP_PERIOD_VERY_FAST;
    }
    // После декодирования кода для продолжения приёма должен вызаваться метод resume().
    irrecv.resume();
  }
  
  if (isAttached) {
    LeftServo.write(LEFT_SERVO_ZERO_VALUE + getAngle(amplitudeLeftServo, globalPhase, angleShiftLeftServo));
    RightServo.write(RIGHT_SERVO_ZERO_VALUE + getAngle(amplitudeRightServo, globalPhase, angleShiftRightServo));
    CentralServo.write(CENTRAL_SERVO_ZERO_VALUE + getAngle(CENTRAL_SERVO_AMPLITUDE, globalPhase, angleShiftCentralServo));
  }
}

