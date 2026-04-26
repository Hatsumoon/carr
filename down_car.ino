#include <Arduino.h>
#include "bsp.h"
#include "motion.h"
#include "trackline.h"  // 【修复3】补充缺失的循迹头文件，确保 Forward() 可用

// ====== 这里填“右后轮”那一路的两根输入脚 ======
static const uint32_t RH_IN1 = PB0;
static const uint32_t RH_IN2 = PB1;
// =========================================

static const uint32_t START_DELAY_MS = 5000;
static const int8_t   CMD = 60;     // 给 Motors_Set 的命令
static const uint8_t  PWM = 180;    // 给 analogWrite 的 pwm (0..255)

static const uint16_t RUN_MS  = 2500;
static const uint16_t STOP_MS = 1200;

enum ActionType {
  ACTION_FORWARD,
  ACTION_LEFT_TURN,
  ACTION_RIGHT_TURN,
  ACTION_BACK_TURN,
  ACTION_PUSH_BLOCKS,
  ACTION_WAIT
};

struct RouteStep {
  ActionType action;
  uint16_t value;
};

// 【修复1】手动声明函数原型，专门用来对付 Arduino IDE 找不到自定义结构体的 Bug
static void ExecuteStep(const struct RouteStep &step);

static const RouteStep selectionRoute[] = {
  //{ACTION_FORWARD, 2},
  //ACTION_WAIT, 100},
  //{ACTION_RIGHT_TURN, 0},
  //{ACTION_WAIT, 100},
  //{ACTION_FORWARD, 2},
  //{ACTION_WAIT, 100},
  {ACTION_LEFT_TURN, 0},
  /*{ACTION_WAIT, 100},
  {ACTION_FORWARD, 1},
  {ACTION_WAIT, 100},
  {ACTION_RIGHT_TURN, 0},
  {ACTION_WAIT, 100},
  {ACTION_PUSH_BLOCKS, 0}*/
};

static const uint8_t selectionRouteLength = sizeof(selectionRoute) / sizeof(selectionRoute[0]);

static void StopAll(uint16_t ms) {
  Stop();
  delay(ms);
}

static void RawStop() {
  digitalWrite(RH_IN1, LOW);
  digitalWrite(RH_IN2, LOW);
}

static void ExecuteStep(const RouteStep &step) {
  switch (step.action) {
    case ACTION_FORWARD:      Forward(step.value); break;
    case ACTION_LEFT_TURN:    LeftTurn(); break;
    case ACTION_RIGHT_TURN:   RightTurn(); break;
    case ACTION_BACK_TURN:    BackTurn(); break;
    case ACTION_PUSH_BLOCKS:  PushBlocks(); break;
    case ACTION_WAIT:         delay(step.value); break;
    default: break;
  }
}

void setup() {
  Serial.begin(9600);

  // 初始化底层硬件
  Motor_Init();
  Trackline_Init(); // 【修复3配套】调用 Forward 前必须初始化循迹

  // 初始化手动测试引脚
  pinMode(RH_IN1, OUTPUT);
  pinMode(RH_IN2, OUTPUT);
  RawStop();

  Serial.println("\n[DOWNCAR TEST] start delay...");
  Stop();
  delay(START_DELAY_MS);
}

void loop() {
  // 【修复2】定义并检查 done 变量，跑完一遍就彻底锁死
  /*static bool done = false;
  if (done) { 
    Stop(); 
    return; 
  }

  for (uint8_t i = 0; i < selectionRouteLength; ++i) {
    ExecuteStep(selectionRoute[i]);   
  }
  
  Stop();
  done = true; // 标记流程结束*/
  //Velocity_Controller(180,80,0,false);*/
  RightTurn();
  Velocity_Controller(0,80,0,false);
  delay(200);
  LeftTurn();
}