/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include<stdbool.h>
#include<string.h>
#include<stdio.h>

#include "bno055_stm32.h"
#include "stm32roboclaw.h"

#include<math.h>
#include "hrishikesh.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_UART4_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int target=0;
float error= 0;
float preverror=0;
float P=8;
float I=0.0005;
float D=70;
float intg;

int16_t vFL, vFR, vBL, vBR;
int16_t vMAX = 0;
int16_t maxspeed = 32000;

int16_t PID(float angz){
	error=target-angz;
	intg += error;
	int16_t output = (error*P)+(intg*I)+((error-preverror)*D);
	if(output>100) output = 100;
	if(output<-100) output = -100;
	preverror=error;
	//printf("%d\r\n",error);
	return output;
}

uint8_t rx_byte;
volatile uint8_t rx_count=0;
volatile bool ready= false;
volatile uint8_t idx=0;
uint8_t packet_buffer[11];

//data
int8_t LX;
int8_t LY;
int8_t RX;
int8_t RY;
uint8_t L2,R2;
uint16_t buttons;
uint16_t prev_buttons;
int mode=0,sign=0;

//FOD variables
float driveLX, driveLY;

#define EVENT_QUEUE_SIZE 8
volatile uint16_t event_queue[8];
volatile uint8_t event_head =0;
volatile uint8_t event_tail =0;
volatile uint16_t prev_buttons_isr = 0;

#define NUM_BUTTONS 10
volatile uint8_t butt[NUM_BUTTONS] = {0};

//cytron definitions
#define KFSvertical 0x00
#define KFShorizontal 0x08
#define leadscrew 0x01
#define powerwindow 0x09

//power window rotation flags
bool CW = false;
bool CCW = false;

bool gripper = true;

//kfs intake flags
bool kfsintake = false;
bool kfsexit = false;
bool kfshold = true;

bool kfsLeadscrewVertUp = false;
bool kfsLeadscrewVertDown = false;

bool kfsLeadscrewHoriFW = false;
bool kfsLeadscrewHoriBW = false;

//staff picking leadscrew flags
bool sexPosition = false;
int Encodertarget = 0;
float encerror;
float encpreverror;
float encP = 1.0;
float encI = 0.0;
float encD = 0.0;
float encintg;
int encOut;
bool manualControl = true;
volatile uint8_t servoflag = 0;      // target state (0° / 90°)
static uint8_t lastServoState = 255; // tracks previous state

int16_t encPID(float input){
	encerror=Encodertarget-input;
	encintg += encerror*encI;
	int16_t output = (encerror*encP)+encintg+((encerror-encpreverror)*encD);
	if(output>100) output = 100;
	if(output<-100) output = -100;
	preverror=error;
	//printf("%d\r\n",error);
	return output;
}

//encoder counts
volatile int32_t KFSencoderCount = 0;
volatile int32_t StaffencoderCount = 0;

static inline void enqueue_button_event(uint16_t pressed)
{
	uint8_t next = (event_head + 1) % EVENT_QUEUE_SIZE;

	if (next != event_tail) {   // queue not full
		event_queue[event_head] = pressed;
		event_head = next;
	}
}

bool check_packet(uint8_t *buf){
	if(buf[0]!=0xAA || buf[10]!=0x55) return false;
	uint8_t sum=0;
	for(int i=0;i<=8;i++) sum+=buf[i];
	return ((uint8_t)(0-sum)==buf[9]);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin==ENC2A_Pin){
		if(HAL_GPIO_ReadPin(ENC2B_GPIO_Port, ENC2B_Pin)==1){
			StaffencoderCount++;
		}
		else{
			StaffencoderCount--;
		}
	}

	if(GPIO_Pin==ENC1A_Pin){
		if(HAL_GPIO_ReadPin(ENC1B_GPIO_Port, ENC1B_Pin)==1){
			KFSencoderCount++;
		}
		else{
			KFSencoderCount--;
		}
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == UART4)
	{
		if (idx == 0)
		{
			if (rx_byte == 0xAA)
			{
				packet_buffer[idx++] = rx_byte;
			}
		}
		else
		{
			packet_buffer[idx++] = rx_byte;

			if (idx == 11)
			{
				if (check_packet(packet_buffer))
				{
					ready=true;
					uint16_t buttons =
							packet_buffer[7] |
							(packet_buffer[8] << 8);

					uint16_t pressed = buttons & ~prev_buttons;

					if (pressed)
						enqueue_button_event(pressed);

					prev_buttons = buttons;

					HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
				}

				idx = 0;
			}
		}

		HAL_UART_Receive_IT(&huart4, &rx_byte, 1);
	}
}

void MDDS60_SendPacket(uint8_t addr, int8_t cmd)
{
	cmd = map(cmd, -100, 100, 0, 255);

	uint8_t packet[4];

	packet[0] = 0x55;
	packet[1] = addr;
	packet[2] = cmd;
	packet[3] = packet[0] + packet[1] + packet[2];

	HAL_UART_Transmit(&huart3, packet, 4, 100);
}

void MDDS60_Autobaud(void)
{
	uint8_t b = 0x55;

	HAL_Delay(2000);
	HAL_UART_Transmit(&huart3, &b, 1, 100);
	HAL_Delay(1000);
}

void MDDS60_StopAll(void){
	MDDS60_SendPacket(0x00,0);
	HAL_Delay(20);
	MDDS60_SendPacket(0x08,0);
}

void process_packet(void){
	if(ready){
		uint8_t local_buf[11];
		__disable_irq();
		memcpy(local_buf, packet_buffer, 11);
		ready = false;
		__enable_irq();

		if(check_packet(local_buf)){
			//			LX = (int8_t)local_buf[1];
			//			LY = (int8_t)local_buf[2];
			//			RX = (int8_t)local_buf[3];
			//			RY = (int8_t)local_buf[4];
			LX = ((int16_t)local_buf[1] - 50) * 2;
			LY = ((int16_t)local_buf[2] - 50) * 2;
			RX = ((int16_t)local_buf[3] - 50) * 2;
			RY = ((int16_t)local_buf[4] - 50) * 2;
			R2 = local_buf[5];
			L2 = local_buf[6];
		}
	}
}



void print_pots(void)
{
	printf("LX:%d  LY:%d  RX:%d  RY:%d  L2:%d  R2:%d\r\n", LX, LY, RX, RY, L2, R2);
}

// Send printf to uart2
extern "C" int _write(int fd, char* ptr, int len) {
	HAL_StatusTypeDef hstatus;

	if (fd == 1 || fd == 2) {
		hstatus = HAL_UART_Transmit(&huart2, (uint8_t *) ptr, len, HAL_MAX_DELAY);
		if (hstatus == HAL_OK)
			return len;
		else
			return -1;
	}
	return -1;
}

stm32RoboClaw motor(&huart1,100);
stm32RoboClaw motor2(&huart6,100);

void inv_kin(float x, float y, float z){
	if(mode==2){
		x=-x*0.35;
		y=-y*0.35;
	}

	else if(mode==4){
		x=-x*0.80;
		y=-y*0.80;
	}
	else{
		x=-x*0.5;
		y=-y*0.5;
	}

	//z=-z;
	z = map(z, -100, 100, -35000, 35000); //maps value of output with range of speed of motors // 100
	x = map(x, -100, 100, -35000, 35000); //maps xj1 values to max speed of motors
	y = map(y, -100, 100, -35000, 35000); //maps yj1 values to max speed of motors

	//inverse kinematics equation for each motor
	vFL = -0.35*x - 0.35*y + 0.25*z;
	vFR = -0.35*x + 0.35*y + 0.25*z;
	vBL = 0.35*x - 0.35*y + 0.25*z;
	vBR = 0.35*x + 0.35*y + 0.25*z;

	vMAX = fmaxf(fmaxf(abs(vFL), abs(vBL)),fmaxf(abs(vFR), abs(vBR)));

	if (vMAX > maxspeed) {
		vFL = (vFL / vMAX) * maxspeed;
		vBL = (vBL / vMAX) * maxspeed;
		vFR = (vFR / vMAX) * maxspeed;
		vBR = (vBR / vMAX) * maxspeed;
	}

	vFL=constrain(vFL,-maxspeed,maxspeed);
	vFR=constrain(vFR,-maxspeed,maxspeed);
	vBL=constrain(vBL,-maxspeed,maxspeed);
	vBR=constrain(vBR,-maxspeed,maxspeed);

	//printf("vfl: %d, vfr: %d, , vbl: %d, , vbr: %d\r\n",vFL, vFR, vBL, vBR);
	motor.DutyM1(129, vFR);
	motor.DutyM2(129, vFL);
	motor2.DutyM1(129, vBL);
	motor2.DutyM2(129, -vBR);

}

void motor_test(void){
	motor.DutyM1(129, 2000);	//vFR
	motor.DutyM2(129, 2000);	//vFL
	motor2.DutyM1(129, 2000);	//vBL
	motor2.DutyM2(129, -2000);	//vBR
}

void PotDeadzone(){
	if(LX<20 && LX>-20) LX=0;
	if(LY<20 && LY>-20) LY=0;
	if(RX<20 && RX>-20) RX=0;
	if(RY<20 && RY>-20) RY=0;

}

void ModeSet(){
	if(RX> 75) mode=1;
	if(RX<-75) mode=2;
	if(RY> 75) mode=3;
	if(RY<-75) mode=4;
}

//void FOD(float x){
//	float pi= 3.14159;
//	float angle = ((-x*pi)/180);
//	int8_t tempLX,tempLY;
//
//	tempLX = (LY*sin(angle)) + (LX*cos(angle));
//	tempLY = (LY*cos(angle)) -(LX*sin(angle));
//
//	LX = tempLX;
//	LY = tempLY;
//}

void FOD(float heading, float *outLX, float *outLY)
{
	float angle = heading * (3.14159265f / 180.0f);

	float x = (float)LX;
	float y = (float)LY;

	*outLX = (x * cosf(angle)) - (y * sinf(angle));
	*outLY = (x * sinf(angle)) + (y * cosf(angle));
}

void colorSet(int mode){
	if(mode==1){ //green
		HAL_GPIO_WritePin(LEDR_GPIO_Port, LEDR_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LEDG_GPIO_Port, LEDG_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LEDB_GPIO_Port, LEDB_Pin, GPIO_PIN_RESET);
	}

	else if(mode==2){ //yellow
		HAL_GPIO_WritePin(LEDR_GPIO_Port, LEDR_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LEDG_GPIO_Port, LEDG_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LEDB_GPIO_Port, LEDB_Pin, GPIO_PIN_RESET);
	}

	else if(mode==3){ //blue
		HAL_GPIO_WritePin(LEDR_GPIO_Port, LEDR_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LEDG_GPIO_Port, LEDG_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LEDB_GPIO_Port, LEDB_Pin, GPIO_PIN_SET);
	}

	else if(mode==4){ //red
		HAL_GPIO_WritePin(LEDR_GPIO_Port, LEDR_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(LEDG_GPIO_Port, LEDG_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(LEDB_GPIO_Port, LEDB_Pin, GPIO_PIN_RESET);
	}
}



void StaffPickingLeadscrew(){
	if(mode==2){
		if(L2>70){
			MDDS60_SendPacket(leadscrew, -50);
		}
		else if(R2>70){
			MDDS60_SendPacket(leadscrew, 50);
		}
		else{
			MDDS60_SendPacket(leadscrew, 0);
		}
	}
}

void PowerWindowRotation(){
	if(mode==2){
		if(CW){
			//printf("moving cw\r\n");
			// red side pwm = 20 for blue side pwm  = -20
			int bump = HAL_GPIO_ReadPin(BUMP2_GPIO_Port, BUMP2_Pin);
			if(bump!=0){
				MDDS60_SendPacket(powerwindow, 20);
			}
			else{
				MDDS60_SendPacket(powerwindow, 0);
			}

		}

		else if(CCW){
			//printf("moving ccw\r\n");
			MDDS60_SendPacket(powerwindow, -20);
		}

		else{
			//printf("stopped\r\n");
			MDDS60_SendPacket(powerwindow, 0);
		}
	}
}

void gripperRotation(){
	if(gripper==false){
		HAL_GPIO_WritePin(SOL1_GPIO_Port, SOL1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(SOL2_GPIO_Port, SOL2_Pin, GPIO_PIN_RESET);
	}
	else{
		HAL_GPIO_WritePin(SOL1_GPIO_Port, SOL1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(SOL2_GPIO_Port, SOL2_Pin, GPIO_PIN_SET);
	}
}

void KFSPicking(){
	if(mode==3){
		if(kfsintake){
			//printf("intake\r\n");
			TIM1->CCR3 = 0;
			TIM1->CCR4 = 32767;
		}
		else if(kfsexit){
			//printf("exit\r\n");
			TIM1->CCR3 = 6000;
			TIM1->CCR4 = 0;
		}
		else{
			//printf("stopped\r\n");
			TIM1->CCR3 = 0;
			TIM1->CCR4 = 0;
		}
	}
}

void KFShold(){
	if(mode==3){
		TIM1->CCR1 = 0;
		TIM1->CCR2 = 0;
		if(kfshold){
			printf("open position\r\n");
			if(KFSencoderCount<-13){
				TIM1->CCR1 = 15000;
				TIM1->CCR2= 0;
			}
			//			else{
			//				TIM1->CCR1 = 0;
			//				TIM1->CCR2 = 0;
			//			}
		}
		else{
			printf("close position\r\n");
			if(KFSencoderCount>-50){
				TIM1->CCR1 = 0;
				TIM1->CCR2= 15000;
			}
			else{
				TIM1->CCR1 = 0;
				TIM1->CCR2 = 900;
			}
		}
	}
}

void KFSLeadscrewVert(){
	if(mode==3){
		if(kfsLeadscrewVertUp){
			//printf("Lead up\r\n");
			MDDS60_SendPacket(KFSvertical, 30);
		}
		else if(kfsLeadscrewVertDown){
			//printf("Lead down\r\n");
			MDDS60_SendPacket(KFSvertical, -30);
		}
		else{
			//printf("stopped\r\n");
			MDDS60_SendPacket(KFSvertical, 0);
		}
	}
}

void KFSLeadscrewHori(){
	if(mode==3){
		if(kfsLeadscrewHoriBW){
			//printf("Lead back\r\n");
			MDDS60_SendPacket(KFShorizontal, 70);
		}
		else if(kfsLeadscrewHoriFW){
			//printf("Lead forw\r\n");
			int bump1 = HAL_GPIO_ReadPin(BUMP1_GPIO_Port, BUMP1_Pin);
			if(bump1== 1){
				MDDS60_SendPacket(KFShorizontal, 0);
			}
			else{
				MDDS60_SendPacket(KFShorizontal, -70);
			}
		}
		else{
			//printf("stopped\r\n");
			MDDS60_SendPacket(KFShorizontal, 0);
		}
	}
}

void StaffPickingLeadscrewPID(){
	if(sexPosition){
		Encodertarget = 19500;
	}
	else{
		Encodertarget = 0;
	}

	encOut = encPID(StaffencoderCount);
	MDDS60_SendPacket(leadscrew, encOut);
}

void ServoControl()
{
	if(lastServoState != servoflag){

		if(servoflag){   // Move 0 -> 90
			for(int angle = 0; angle <= 100; angle++){
				uint16_t pulse = 500 + (angle * 2000 / 180);

				__HAL_TIM_SET_COMPARE(
						&htim2,
						TIM_CHANNEL_1,
						pulse
				);

				HAL_Delay(10);
			}
		}
		else{            // Move 90 -> 0
			for(int angle = 90; angle >= 0; angle--){

				uint16_t pulse = 500 + (angle * 2000 / 180);

				__HAL_TIM_SET_COMPARE(
						&htim2,
						TIM_CHANNEL_1,
						pulse
				);

				HAL_Delay(10);
			}
		}

		lastServoState = servoflag;
	}
}

void process_button_events(void)
{
	while (event_tail != event_head) {

		uint16_t evt = event_queue[event_tail];
		event_tail = (event_tail + 1) % EVENT_QUEUE_SIZE;

		if(evt & (1 << 0)){
			//printf("Cross\r\n");
			if(mode==1){
				target+=(sign*180);
				P = 5;
				I = 0.0007;
				D = 80;
			}

			else if(mode==3){
				kfsLeadscrewVertUp = !kfsLeadscrewVertUp;
				kfsLeadscrewVertDown=false;
			}

		}

		if(evt & (1 << 1)){
			//printf("Circle\r\n");
			if(mode==1){
				target+=(sign*90);
				P = 1.5;
				I = 0.0005;
				D = 120;
			}

			else if(mode==3){
				kfsLeadscrewHoriFW = !kfsLeadscrewHoriFW;
				kfsLeadscrewHoriBW = false;
			}

		}
		if(evt & (1 << 2)){
			//printf("Triangle\r\n");
			if(mode==1){
				target+=(sign*45);
				P = 4;
				I = 0.0005;
				D = 75;
			}

			else if(mode==2){
				servoflag = !servoflag;
			}
			else if(mode==3){
				kfsLeadscrewVertDown = !kfsLeadscrewVertDown;
				kfsLeadscrewVertUp=false;
			}

		}
		if(evt & (1 << 3)){
			//printf("Square\r\n");
			if(mode==1){
				target+=(sign*5);
				P = 8;
				I = 0.0005;
				D = 50;
			}

			else if(mode==2){
				manualControl = !manualControl;
			}

			else if(mode==3){
				kfsLeadscrewHoriBW = !kfsLeadscrewHoriBW;
				kfsLeadscrewHoriFW = false;
			}

		}
		if(evt & (1 << 4)){
			//printf("L1\r\n");
			if(mode==2){
				gripper =!gripper;
			}

			else if(mode==3){
				kfsintake =!kfsintake;
				kfsexit = false;
			}
		}
		if(evt & (1 << 5)){
			//printf("R1\r\n");
			if(mode==3){
				kfsexit =!kfsexit;
				kfsintake = false;
			}

			//			else if(mode==2){
			//				sexPosition = !sexPosition;
			//			}
		}
		if(evt & (1 << 6)) {
			//printf("Up\r\n");
			if(mode==2){
				CW=!CW;
				CCW=false;
			}
		}
		if(evt & (1 << 7)){
			//printf("Down\r\n");
			if(mode==2){
				CCW=!CCW;
				CW=false;
			}
		}
		if(evt & (1 << 8)){
			//printf("Left\r\n");
			sign=-1;
			if(mode==3){
				kfshold = !kfshold;
			}
		}
		if(evt & (1 << 9)){
			//printf("Right\r\n");
			sign=1;
		}
	}
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{

	/* USER CODE BEGIN 1 */
	motor.DutyM1(129, -0);
	motor.DutyM2(129, 0);
	motor2.DutyM1(129, 0);
	motor2.DutyM2(129, 0);
	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_USART2_UART_Init();
	MX_I2C1_Init();
	MX_USART1_UART_Init();
	MX_USART6_UART_Init();
	MX_UART4_Init();
	MX_USART3_UART_Init();
	MX_TIM1_Init();
	MX_TIM2_Init();
	/* USER CODE BEGIN 2 */
	printf("STM32 initializing\r\n");
	HAL_GPIO_WritePin(LEDR_GPIO_Port, LEDR_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LEDG_GPIO_Port, LEDG_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LEDB_GPIO_Port, LEDB_Pin, GPIO_PIN_SET);

	HAL_NVIC_SetPriority(UART4_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(UART4_IRQn);
	HAL_UART_Receive_IT(&huart4, &rx_byte, 1);

	HAL_Delay(100);

	bno055_assignI2C(&hi2c1);
	bno055_setup();
	bno055_setOperationModeNDOF();

	HAL_Delay(1000);
	printf("Stm32 setup complete\r\n");
	HAL_GPIO_WritePin(LEDR_GPIO_Port, LEDR_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LEDG_GPIO_Port, LEDG_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LEDB_GPIO_Port, LEDB_Pin, GPIO_PIN_RESET);

	MDDS60_Autobaud();
	MDDS60_StopAll();
	HAL_Delay(500);

	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		process_packet();
		process_button_events();
		PotDeadzone();
		ModeSet();
		colorSet(mode);
		//		//print_pots();
		//
		float v = bno055_getHeadingContinuous();
		printf("%.2f\r\n",v);
		FOD(v,&driveLX,&driveLY);
		inv_kin(driveLX, driveLY, PID(v));
		//printf("VFL:%d\tVFR:%d\tVBL:%d\tVBR:%d\r\n",vFL,vFR,vBL,vBR);
		//printf("%.2f,-8,8\r\n",v);

		if(manualControl){
			StaffPickingLeadscrew();
		}
		else{
			//StaffPickingLeadscrewPID();
			if(StaffencoderCount>=-98900){
				MDDS60_SendPacket(leadscrew, -70);
			}
			else{
				MDDS60_SendPacket(leadscrew, 0);
			}
		}
		PowerWindowRotation();
		gripperRotation();
		KFSPicking();
		KFShold();
		KFSLeadscrewVert();
		KFSLeadscrewHori();
		ServoControl();

		//printf("%d\r\n",StaffencoderCount);


		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 150;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 2;
	RCC_OscInitStruct.PLL.PLLR = 2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV16;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void)
{

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = 400000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void)
{

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 0;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 32767;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
	{
		Error_Handler();
	}
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */
	HAL_TIM_MspPostInit(&htim1);

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void)
{

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 15;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 19999;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 1500;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */
	HAL_TIM_MspPostInit(&htim2);

}

/**
 * @brief UART4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_UART4_Init(void)
{

	/* USER CODE BEGIN UART4_Init 0 */

	/* USER CODE END UART4_Init 0 */

	/* USER CODE BEGIN UART4_Init 1 */

	/* USER CODE END UART4_Init 1 */
	huart4.Instance = UART4;
	huart4.Init.BaudRate = 115200;
	huart4.Init.WordLength = UART_WORDLENGTH_8B;
	huart4.Init.StopBits = UART_STOPBITS_1;
	huart4.Init.Parity = UART_PARITY_NONE;
	huart4.Init.Mode = UART_MODE_TX_RX;
	huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart4.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart4) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN UART4_Init 2 */

	/* USER CODE END UART4_Init 2 */

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void)
{

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void)
{

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart2) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void)
{

	/* USER CODE BEGIN USART3_Init 0 */

	/* USER CODE END USART3_Init 0 */

	/* USER CODE BEGIN USART3_Init 1 */

	/* USER CODE END USART3_Init 1 */
	huart3.Instance = USART3;
	huart3.Init.BaudRate = 9600;
	huart3.Init.WordLength = UART_WORDLENGTH_8B;
	huart3.Init.StopBits = UART_STOPBITS_1;
	huart3.Init.Parity = UART_PARITY_NONE;
	huart3.Init.Mode = UART_MODE_TX_RX;
	huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart3.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart3) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART3_Init 2 */

	/* USER CODE END USART3_Init 2 */

}

/**
 * @brief USART6 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART6_UART_Init(void)
{

	/* USER CODE BEGIN USART6_Init 0 */

	/* USER CODE END USART6_Init 0 */

	/* USER CODE BEGIN USART6_Init 1 */

	/* USER CODE END USART6_Init 1 */
	huart6.Instance = USART6;
	huart6.Init.BaudRate = 230400;
	huart6.Init.WordLength = UART_WORDLENGTH_8B;
	huart6.Init.StopBits = UART_STOPBITS_1;
	huart6.Init.Parity = UART_PARITY_NONE;
	huart6.Init.Mode = UART_MODE_TX_RX;
	huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart6.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart6) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART6_Init 2 */

	/* USER CODE END USART6_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	/* USER CODE BEGIN MX_GPIO_Init_1 */

	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, LEDR_Pin|SOL2_Pin|SOL1_Pin|LEDG_Pin
			|LEDB_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : ENC1A_Pin ENC2A_Pin */
	GPIO_InitStruct.Pin = ENC1A_Pin|ENC2A_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : ENC2B_Pin ENC1B_Pin */
	GPIO_InitStruct.Pin = ENC2B_Pin|ENC1B_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : BUMP2_Pin */
	GPIO_InitStruct.Pin = BUMP2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(BUMP2_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : LEDR_Pin SOL2_Pin SOL1_Pin LEDG_Pin
                           LEDB_Pin */
	GPIO_InitStruct.Pin = LEDR_Pin|SOL2_Pin|SOL1_Pin|LEDG_Pin
			|LEDB_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : BUMP1_Pin */
	GPIO_InitStruct.Pin = BUMP1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(BUMP1_GPIO_Port, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI0_IRQn);

	HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI1_IRQn);

	/* USER CODE BEGIN MX_GPIO_Init_2 */

	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
	}
	/* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
