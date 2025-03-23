#include "stm32f10x.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_rcc.h"

#include "misc.h"

// 초음파 센서
#define US_TIMER TIM3
#define US_TRIG_PORT GPIOB
#define US_TRIG_PIN GPIO_Pin_0 //TIM Ch3 (trig output)
#define US_ECHO_PORT GPIOA
#define US_ECHO_PIN GPIO_Pin_6 //TIM Ch1 (echo input)
#define US_TIMER_TRIG_SOURCE TIM_TS_TI1FP1

// 부저
#define BUZZER_PIN GPIO_Pin_3
#define BUZZER_PORT GPIOA
#define BUZZER_TIM TIM2
#define BUZZER_TIM_CHANNEL TIM_Channel_4


/* function prototype */
void RCC_Configure(void);
void GPIO_Configure(void);
void EXTI_Configure(void);
void USART1_Init(void);
void USART2_Init(void);
void NVIC_Configure(void);
void Delay(void);
void Motor_Control(uint16_t motor, uint16_t direction, uint16_t speed);

// Ultra Sonic
void InitHCSR04();
int32_t HCSR04GetDistance();
static void initMeasureTimer();

// piezo
void TIM_Config(void);
void Start_PWM(uint16_t frequency);
void Stop_PWM(void);

//variable
int smartphone_to_putty = 1;
TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
TIM_OCInitTypeDef TIM_OCInitStructure;
uint16_t motorDir = 0;
uint16_t motorSpeed = 0;
int state = 0; // 안 위험 =0, 위험 = 1
int stop = 0; // 긴급정지 안했음 =0 , 했음 = 1


//---------------------------------------------------------------------------------------------------

void InitHCSR04() {
initMeasureTimer();
}

static void initMeasureTimer() {
RCC_ClocksTypeDef RCC_ClocksStatus;
RCC_GetClocksFreq(&RCC_ClocksStatus);
uint16_t prescaler = RCC_ClocksStatus.SYSCLK_Frequency / 1000000 - 1; //1 tick = 1us (1 tick = 0.165mm resolution)

TIM_DeInit(US_TIMER);
TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
TIM_TimeBaseInitStruct.TIM_Prescaler = prescaler;
TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
TIM_TimeBaseInitStruct.TIM_Period = 0xFFFF;
TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
TIM_TimeBaseInit(US_TIMER, &TIM_TimeBaseInitStruct);

TIM_OCInitTypeDef TIM_OCInitStruct;
TIM_OCStructInit(&TIM_OCInitStruct);
TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
TIM_OCInitStruct.TIM_Pulse = 15; //us
TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;
TIM_OC3Init(US_TIMER, &TIM_OCInitStruct);

TIM_ICInitTypeDef TIM_ICInitStruct;
TIM_ICInitStruct.TIM_Channel = TIM_Channel_1;
TIM_ICInitStruct.TIM_ICPolarity = TIM_ICPolarity_Rising;
TIM_ICInitStruct.TIM_ICSelection = TIM_ICSelection_DirectTI;
TIM_ICInitStruct.TIM_ICPrescaler = TIM_ICPSC_DIV1;
TIM_ICInitStruct.TIM_ICFilter = 0;

TIM_PWMIConfig(US_TIMER, &TIM_ICInitStruct);
TIM_SelectInputTrigger(US_TIMER, US_TIMER_TRIG_SOURCE);
TIM_SelectMasterSlaveMode(US_TIMER, TIM_MasterSlaveMode_Enable);

TIM_CtrlPWMOutputs(US_TIMER, ENABLE);

TIM_ClearFlag(US_TIMER, TIM_FLAG_Update);
}

int32_t HCSR04GetDistance() {
(US_TIMER)->CNT = 0;
TIM_Cmd(US_TIMER, ENABLE);
while(!TIM_GetFlagStatus(US_TIMER, TIM_FLAG_Update));
TIM_Cmd(US_TIMER, DISABLE);
TIM_ClearFlag(US_TIMER, TIM_FLAG_Update);
return (TIM_GetCapture2(US_TIMER)-TIM_GetCapture1(US_TIMER))*165/1000;
}

void RCC_Configure(void)
{
// TODO: Enable the APB2 peripheral clock using the function 'RCC_APB2PeriphClockCmd'

/* UART TX/RX port clock enable */
RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

/* USART1 USART2 clock enable */
RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

/* Alternate Function IO clock enable */
RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

// RCC setup and and TIM4 CH3, CH4
RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

// 타이머 클럭 활성화
RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
}

void GPIO_Configure(void)
{
GPIO_InitTypeDef GPIO_InitStructure;

/* USART1 Pin setting */
//TX
GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
GPIO_Init(GPIOA, &GPIO_InitStructure);

//RX
GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
GPIO_Init(GPIOA, &GPIO_InitStructure);

/* USART2 Pin setting */
//TX
GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
GPIO_Init(GPIOA, &GPIO_InitStructure);

//RX
GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
GPIO_Init(GPIOA, &GPIO_InitStructure);

//TIM4 CH1, CH2, CH3,CH4 setup - PB0, PB1 - right motor
GPIO_InitTypeDef GPIO_InitStructure2;
GPIO_InitStructure2.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
GPIO_InitStructure2.GPIO_Mode = GPIO_Mode_AF_PP;
GPIO_InitStructure2.GPIO_Speed = GPIO_Speed_50MHz;
GPIO_Init(GPIOB, &GPIO_InitStructure2);

uint16_t prescale = 0;
prescale = (uint16_t) (SystemCoreClock / 1000000);
TIM_TimeBaseStructure.TIM_Period = 10000-1;
TIM_TimeBaseStructure.TIM_Prescaler = prescale - 1;
TIM_TimeBaseStructure.TIM_ClockDivision = 0;
TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
TIM_OCInitStructure.TIM_Pulse = motorSpeed; // us

TIM_OC1Init(TIM4, &TIM_OCInitStructure); // TIM4 CH1 init
TIM_OC2Init(TIM4, &TIM_OCInitStructure); // TIM4 CH2 init
TIM_OC3Init(TIM4, &TIM_OCInitStructure); // TIM4 CH3 init
TIM_OC4Init(TIM4, &TIM_OCInitStructure); // TIM4 CH4 init

TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); // timer init

TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable); // TIM4 CH1 setup and start
TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable); // TIM4 CH2 setup and start
TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable); // TIM4 CH3 setup and start
TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable); // TIM4 CH4 setup and start

TIM_ARRPreloadConfig(TIM4, ENABLE); // APR preload enable
TIM_Cmd(TIM4, ENABLE); // timer enable


GPIO_InitStructure.GPIO_Pin = US_TRIG_PIN;
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
GPIO_Init(US_TRIG_PORT, &GPIO_InitStructure);

GPIO_InitStructure.GPIO_Pin = US_ECHO_PIN;
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
GPIO_Init(US_ECHO_PORT, &GPIO_InitStructure);

TIM_TimeBaseInitTypeDef TIM_TimeBaseStruct;
TIM_OCInitTypeDef TIM_OCStruct;
// 타이머 기본 설정
uint16_t prescale2 = 0;
uint16_t period = 1000;
prescale2 = (uint16_t) (SystemCoreClock / 1000000);

TIM_TimeBaseStruct.TIM_Period = period - 1; // 초기 주파수: 1kHz
TIM_TimeBaseStruct.TIM_Prescaler = prescale2 - 1; // 72MHz -> 1MHz 클럭
TIM_TimeBaseStruct.TIM_ClockDivision = 0;
TIM_TimeBaseStruct.TIM_CounterMode = TIM_CounterMode_Down;
TIM_TimeBaseInit(BUZZER_TIM, &TIM_TimeBaseStruct);

// PWM 모드 설정
TIM_OCStruct.TIM_OCMode = TIM_OCMode_PWM1;
TIM_OCStruct.TIM_OutputState = TIM_OutputState_Enable;
TIM_OCStruct.TIM_Pulse = period / 2; // Duty Cycle 50%
TIM_OCStruct.TIM_OCPolarity = TIM_OCPolarity_High;
TIM_OC4Init(BUZZER_TIM, &TIM_OCStruct);
TIM_OC4PreloadConfig(BUZZER_TIM, TIM_OCPreload_Enable);

// 타이머 시작
TIM_ARRPreloadConfig(BUZZER_TIM, ENABLE);
TIM_Cmd(BUZZER_TIM, ENABLE);
}

// PWM 시작 (주파수 설정)
void Start_PWM(uint16_t frequency) {

GPIO_InitTypeDef GPIO_InitStruct;

RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

// PWM 출력 핀 설정 (PA1)
GPIO_InitStruct.GPIO_Pin = BUZZER_PIN;
GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;

GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
GPIO_Init(BUZZER_PORT, &GPIO_InitStruct);

uint16_t period = 1000000 / frequency; // 1MHz 타이머 기준
TIM_SetAutoreload(BUZZER_TIM, period - 1);
TIM_SetCompare4(BUZZER_TIM, period / 2); // Duty Cycle 50%
TIM_Cmd(BUZZER_TIM, ENABLE);
}

// PWM 정지
void Stop_PWM(void) {
GPIO_InitTypeDef GPIO_InitStruct;

RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

// PWM 출력 핀 설정 (PA3)
GPIO_InitStruct.GPIO_Pin = BUZZER_PIN;
GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
GPIO_Init(BUZZER_PORT, &GPIO_InitStruct);

}

void Motor_Control(uint16_t motor, uint16_t direction, uint16_t speed) {
  if (motor == 0) { // left - PA6, PA7
    if (direction == 0) { // move front
    TIM4->CCR1 = speed; // a-1a
    TIM4->CCR2 = 0; // a-1b
    } else { // move back
    TIM4->CCR1 = 0; // a-1a
    TIM4->CCR2 = speed; // a-1b
    }
  } else if (motor == 1) { // right - PB0, PB1
      if (direction == 0) { // move front
      TIM4->CCR3 = speed; // b-1a
      TIM4->CCR4 = 0; // b-1b
      } else { // move back
      TIM4->CCR3 = 0; // b-1a
      TIM4->CCR4 = speed; // b-1b
    }
  }
}

void USART1_Init(void)
{
USART_InitTypeDef USART1_InitStructure;

// Enable the USART1 peripheral
USART_Cmd(USART1, ENABLE);

// TODO: Initialize the USART using the structure 'USART_InitTypeDef' and the function 'USART_Init'
USART1_InitStructure.USART_BaudRate = 9600;
USART1_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
USART1_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
USART1_InitStructure.USART_Parity = USART_Parity_No;
USART1_InitStructure.USART_StopBits = USART_StopBits_1_5;
USART1_InitStructure.USART_WordLength = USART_WordLength_8b;
USART_Init(USART1, &USART1_InitStructure);

// TODO: Enable the USART1 RX interrupts using the function 'USART_ITConfig' and the argument value 'Receive Data register not empty interrupt'
USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
}

void USART2_Init(void)
{
USART_InitTypeDef USART2_InitStructure;

// Enable the USART1 peripheral
USART_Cmd(USART2, ENABLE);

// TODO: Initialize the USART using the structure 'USART_InitTypeDef' and the function 'USART_Init'
USART2_InitStructure.USART_BaudRate = 9600;
USART2_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
USART2_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
USART2_InitStructure.USART_Parity = USART_Parity_No;
USART2_InitStructure.USART_StopBits = USART_StopBits_1_5;
USART2_InitStructure.USART_WordLength = USART_WordLength_8b;
USART_Init(USART2, &USART2_InitStructure);

// TODO: Enable the USART1 RX interrupts using the function 'USART_ITConfig' and the argument value 'Receive Data register not empty interrupt'
USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
}

void NVIC_Configure(void) {

NVIC_InitTypeDef NVIC_InitStructure;

// TODO: fill the arg you want
NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

// UART1
// 'NVIC_EnableIRQ' is only required for USART setting
NVIC_EnableIRQ(USART1_IRQn);
NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3; // TODO
NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; // TODO
NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
NVIC_Init(&NVIC_InitStructure);

// UART2
// 'NVIC_EnableIRQ' is only required for USART setting
NVIC_EnableIRQ(USART2_IRQn);
NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3; // TODO
NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1; // TODO
NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
NVIC_Init(&NVIC_InitStructure);
}

void USART1_IRQHandler() {
uint16_t word;
if(USART_GetITStatus(USART1,USART_IT_RXNE)!=RESET){
// the most recent received data by the USART1 peripheral
word = USART_ReceiveData(USART1);
if (word=='1' && state) {
  motorDir = 0;
  motorSpeed = 0;
  stop = 1;
} else if (word=='2' && !stop) { // Acceleration
  motorDir = 0;
  if (motorSpeed < 7000)  {
    Motor_Control(0,motorDir,9999);
    Delay();
    motorSpeed = 7000;
  }
  motorSpeed += 300;
  if (motorSpeed > 9999)
    motorSpeed = 9999;
} else if (word=='3') { // stop
  motorDir = 0;
  motorSpeed = 0;
}
Motor_Control(0,motorDir,motorSpeed);
Motor_Control(1,motorDir,motorSpeed);
// TODO implement
while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
USART_SendData(USART2, word);

// clear 'Read data register not empty' flag
USART_ClearITPendingBit(USART1,USART_IT_RXNE);
}
}

void USART2_IRQHandler() {
uint16_t word;
if(USART_GetITStatus(USART2,USART_IT_RXNE)!=RESET){
// the most recent received data by the USART2 peripheral
  
word = USART_ReceiveData(USART2);


// TODO implement
while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
USART_SendData(USART1, word);

// clear 'Read data register not empty' flag
USART_ClearITPendingBit(USART2,USART_IT_RXNE);
}
}

void Delay(void) {
int i;

for (i = 0; i < 10000; i++) {}
}

int main(void)
{  
    SystemInit();
    RCC_Configure();
    GPIO_Configure();
    USART1_Init();
    USART2_Init();      // bluetooth
    InitHCSR04();
    NVIC_Configure();
    while (1) {
      int32_t dist = HCSR04GetDistance();
      if ((dist < 300) && !state) {
        state = 1;
        Start_PWM(440);
      }
      else if ((dist >= 300) && state) {
        state = 0;
        Stop_PWM();
      }
    }
    return 0;
}