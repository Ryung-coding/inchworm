#include <Servo.h>
#define SWEEP_DELAY 10  // 서보 모터 이동 속도 조절 (ms)

Servo base_grip;
Servo ee_grip;

// 초기 명령은 별도로 받기 전까지는 설정되지 않은 상태
int base_command = 0;
int ee_command = 0;
// 이전 명령을 추적 (첫 루프에서 무조건 변화로 인식하도록 -1로 초기화)
int prev_base_command = -1;
int prev_ee_command = -1;

// 서보의 현재 위치 (초기값: 35도)
int base_pos = 35;
int ee_pos = 35;
int single_movement = 35;


// 서보 모터를 부드럽게 이동시키는 함수
void moveServoSmoothly(Servo &servo, int &current_pos, int target_pos) {
  while (current_pos != target_pos) {
    if (current_pos < target_pos)
      current_pos++;
    else if (current_pos > target_pos)
      current_pos--;
    servo.write(current_pos);
    delay(SWEEP_DELAY);
  }
}

void setup() {
  Serial.begin(115200);
  base_grip.attach(9);
  ee_grip.attach(10);
  
  // 초기 위치 설정
  base_grip.write(base_pos);
  ee_grip.write(ee_pos);
}

void loop() {
   if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();  // ← 안전하게 문자열 정리
  
    int commaIndex = input.indexOf(',');
    if (commaIndex >= 0) {
      base_command = input.substring(0, commaIndex).toInt();
      ee_command = input.substring(commaIndex + 1).toInt();
    } else {
      int single_command = input.toInt();
      if (single_command == 3 && single_movement < 70) {
        single_movement++;
        ee_grip.write(single_movement);
        delay(SWEEP_DELAY);
      } else if (single_command == 4 && single_movement > 0) {
        single_movement--;
        ee_grip.write(single_movement);
        delay(SWEEP_DELAY);
      }
    }
  }

  if (base_command != prev_base_command) {
    if (prev_base_command == 0 && base_command == 1) {
      moveServoSmoothly(ee_grip, ee_pos, 70);
      moveServoSmoothly(base_grip, base_pos, 0);
    } else if (prev_base_command == 1 && base_command == 0) {
      moveServoSmoothly(base_grip, base_pos, 70);
      moveServoSmoothly(ee_grip, ee_pos, 0);
    }
    prev_base_command = base_command;
  }

  // 🔧 이 부분 추가
  if (ee_command != prev_ee_command) {
    if (ee_command == 1) {
      moveServoSmoothly(ee_grip, ee_pos, 70);
    } else if (ee_command == 0) {
      moveServoSmoothly(ee_grip, ee_pos, 0);
    }
    prev_ee_command = ee_command;
  }
}
