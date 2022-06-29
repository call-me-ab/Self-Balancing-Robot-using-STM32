#include "stm32f4079xx.h"
#include <math.h>
#define MPU6050_ADDR 0xD0
#define SMPLRT_DIV_REG 0x19
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define PWR_MGMT_1_REG 0x6B
#define WHO_AM_I_REG 0x75
#define GYRO_XOUT_H_REG 0x43
float ref = 0;
float Kp=0.05,Kd=40,Ki=40;
float sum_of_errors=0;
float dt = 0;
float deg=57.3;
float Acc_angle=0;
float Gyro_angle=0;
float angle=0;
float error=0,duty=0;
#define INT_ENABLE_REG 0x38
int16_t x_read=0,y_read=0,z_read=0;
int16_t Gyro_X_RAW = 0;
int16_t Gyro_Y_RAW = 0;
int16_t Gyro_Z_RAW = 0;
int16_t Accel_X_RAW = 0;
int16_t Accel_Y_RAW = 0;
int16_t Accel_Z_RAW = 0;
float Ax, Ay, Az, Gx, Gy, Gz;
uint8_t check,temp;
int flag=0;
void EXT_Init(void)
{
//1. Enable TIM@ and GPIO clock
RCC->APB1ENR |= (1<<0); // enable PORTA clock
RCC->APB2ENR |=(1<<14);
RCC->APB2ENR |= (1<<0);
SYSCFG->EXTICR[0] &= ~(0xf<<0);
EXTI->RTSR |= (1<<0); // Enable Rising Edge Trigger for PA0
EXTI->FTSR &= ~(1<<0); // Disable Falling Edge Trigger for PA0
EXTI->IMR |=(1<<0);
NVIC->ISER[0] |= 1<<6;
NVIC_SetPriority (EXTI0_IRQn, 1); // Set Priority
NVIC_EnableIRQ (EXTI0_IRQn); // Enable Interrupt
}
void TIM_Config()
{
// Basic timer configuration
RCC->APB1ENR |= (1UL << 2); //Enable TIM4 clock
TIM4->CR1 &= 0x1101; //Set the mode to Count up
TIM4->CR1 |= 1UL; //Start the timer
}
void PWM_configuration()
{ TIM4->PSC = 1-1; //PRESCALER = sys_clck/required_freq = 16MHz/16Mhz=1;
TIM4->ARR = 1000; // for a 16kHz PWM signal for no noise
//Channel 1 config
TIM4->CCMR1 |= (6UL << 4); //Set OutputCompare mode to PWM
TIM4->CCMR1 &= ~(3UL << 0); //CC to output
TIM4->CCER &= ~(1UL << 1); //Output Compare polarity to active high
TIM4->CCER |= (1UL << 0); //Capture compare output enable
//Channel 2 config
TIM4->CCMR1 |= (6UL << (4+8)); //Set OutputCompare mode to PWM
TIM4->CCMR1 &= ~(3UL << 8); //CC to output
TIM4->CCER &= ~(1UL << (1+4)); //Output Compare polarity to active high
TIM4->CCER |= (1UL << 4); //Capture compare output enable
//CHANNEL 3
TIM4->CCMR2 |= (6UL << (4)); //Set OutputCompare mode to PWM
TIM4->CCMR2 &= ~(3UL << 0); //CC to output
TIM4->CCER &= ~(1UL << (1)); //Output Compare polarity to active high
TIM4->CCER |= (1UL << 0); //Capture compare output enable
//CHANNEL 4
TIM4->CCMR2 |= (6UL << (4+8)); //Set OutputCompare mode to PWM
TIM4->CCMR2 &= ~(3UL << 8); //CC to output
TIM4->CCER &= ~(1UL << (1+4)); //Output Compare polarity to active high
TIM4->CCER |= (1UL << 4); //Capture compare output enable
}
void GPIO_configuration()
{ // GPIOB - for I2C connection with MPU6050
RCC->AHB1ENR |=(1UL<<1);//Enable clock for port B
GPIOB->MODER |=(2UL<<12);//PB6 to alternate function pin
GPIOB->MODER |=(2UL<<14);//PB7 to alternate function pin
GPIOB->PUPDR |=(0x5UL<<12);//set PB6 and 7 as pull up pins
GPIOB->OTYPER |=(0x3UL<<6);//Set PB6 and 7 as open drain
GPIOB->OSPEEDR |=(0xAUL<<12);//Set PB6 and 7 as high speed
GPIOB->AFR[0] |= (0x44<<24);//Set PB6 and 7 to alternate function 4
// GPIOA - for direction control
RCC->AHB1ENR|=RCC_AHB1ENR_GPIOAEN; //ENABLED THE CLOCK FOR GPIOA
 GPIOA->MODER |=(0x55UL<<2); // GPIOA pins - 1,2,3,4 mode selected as general purpose
output
 GPIOA->PUPDR|=(0xAA<<2);// GPIOA pins-> 1,2,3,4 SET TO PULL down MODE
 GPIOA->OSPEEDR|=(0xAA<<2);//GPIOA pins 0,1,2,3 set to high speed
 // GPIOD - for PWM output
 RCC->AHB1ENR|=RCC_AHB1ENR_GPIODEN;
 GPIOD->MODER|=(0xAAUL<<24); //MODE OF PINS 12,13,14,15 of GPIOD SET TO AF
 GPIOD->PUPDR|=(0x55UL<<24);// GPIOD 12 and 13,14,15 set to pull up mode
GPIOD->OSPEEDR|=(0xAAUL<<24); // speed of GPIOD 12 and 13,14,15 set to high speed
}
void I2C_Config(void)
{
RCC->APB1ENR |=(1UL<<21);//Enable I2C clock
I2C1->CR1 |= (1UL<<15);//Reset I2C
I2C1->CR1 &= ~(1UL<<15);
I2C1->CR2 |=(16UL<<0);//Set peripheral clock at 16MHz
I2C1->OAR1 |=(1UL<<14);//Should be set high
I2C1->CCR |=(0x50UL<<0);//Set SCL as 100KHz
I2C1->TRISE |=(17UL<<0);//Configure maximum rise time
I2C1->CR1 |= (1UL<<0);//Enable I2C
}
void I2C_Start (void)
{
I2C1->CR1 |= (1<<10);//Enable the ACK Bit
I2C1->CR1 |= (1<<8);//Send the start bit
while (!(I2C1->SR1 & (1<<0)));//Wait for SB bit to set
}
void I2C_Write(uint8_t data)
{
while (!(I2C1->SR1 & (1<<7)));//Wait till TX buffer is empty
I2C1->DR = data;//Write data to I2C slave
while (!(I2C1->SR1 & (1<<2)));//Wait till Byte transfer is completed
}
void I2C_Address (uint8_t Address)
{
I2C1->DR = Address; // send the slave address
while (!(I2C1->SR1 & (1<<1))); // wait for ADDR bit to set
temp = I2C1->SR1 | I2C1->SR2; // read SR1 and SR2 to clear the ADDR bit
}
void I2C_Read (uint8_t Address, uint8_t *buffer, uint8_t size)
{
int remaining = size;
if (size == 1)
{I2C1->DR = Address; // send the address
while (!(I2C1->SR1 & (1<<1))); // wait for ADDR bit to set
I2C1->CR1 &= ~(1<<10); // clear the ACK bit
temp = I2C1->SR1 | I2C1->SR2; // read SR1 and SR2 to clear the ADDR bit.... EV6
condition
while (!(I2C1->SR1 & (1<<6))); // wait for RxNE to set
buffer[size-remaining] = I2C1->DR; // Read the data from the DATA REGISTER
}
else
{
I2C1->DR = Address; // send the address
while (!(I2C1->SR1 & (1<<1))); // wait for ADDR bit to set
temp = I2C1->SR1 | I2C1->SR2; // read SR1 and SR2 to clear the ADDR bit
while (remaining>2)
{
while (!(I2C1->SR1 & (1<<6))); // wait for RxNE to set
buffer[size-remaining] = I2C1->DR; // copy the data into the buffer
I2C1->CR1 |= 1<<10; // Set the ACK bit to Acknowledge the data received
remaining--;
}
while (!(I2C1->SR1 & (1<<6))); // wait for RxNE to set
buffer[size-remaining] = I2C1->DR;
I2C1->CR1 &= ~(1<<10); // clear the ACK bit
I2C1->CR1 |= (1<<9); // Stop I2C
remaining--;
while (!(I2C1->SR1 & (1<<6))); // wait for RxNE to set
buffer[size-remaining] = I2C1->DR; // copy the data into the buffer
}
}
void I2C_Stop (void)
{
I2C1->CR1 |= (1<<9); // Stop I2C
}
void MPU_Write (uint8_t Address, uint8_t Reg, uint8_t Data)
{
I2C_Start ();
I2C_Address (Address);
I2C_Write (Reg);
I2C_Write (Data);
I2C_Stop ();
}
void MPU_Read (uint8_t Address, uint8_t Reg, uint8_t *buffer, uint8_t size)
{
I2C_Start ();
I2C_Address (Address);
I2C_Write (Reg);
I2C_Start ();
I2C_Read (Address+0x01, buffer, size);//To read, set LSB to 1
I2C_Stop ();
}
void MPU6050_Init (void)
{
uint8_t check;
uint8_t Data;
MPU_Read (MPU6050_ADDR,WHO_AM_I_REG, &check, 1);//Check device ID
if (check == 104) // 0x68 will be returned by the sensor
{
Data = 0;
MPU_Write (MPU6050_ADDR, PWR_MGMT_1_REG, Data);//Power up the sensor
Data = 0x07;
MPU_Write(MPU6050_ADDR, SMPLRT_DIV_REG, Data);//Sampling rate of 1KHz
Data = 0x00;
MPU_Write(MPU6050_ADDR, ACCEL_CONFIG_REG, Data);//accelerometer
range=+/- 2g
Data=0x01;
MPU_Write(MPU6050_ADDR, INT_ENABLE_REG, Data);//Enable interrupt when
data ready
}
}
void read_acc(void)
{
uint8_t Buf[6];
// Read 6 BYTES of data starting from ACCEL_XOUT_H register
MPU_Read (MPU6050_ADDR, ACCEL_XOUT_H_REG, Buf, 6);
Accel_X_RAW= (int16_t)(Buf[0] << 8 | Buf[1]);
Accel_Y_RAW = (int16_t)(Buf[2] << 8 | Buf[3]);
Accel_Z_RAW = (int16_t)(Buf[4] << 8 | Buf[5]);
Ax = Accel_X_RAW/16384.0;
Ay = Accel_Y_RAW/16384.0;
Az = Accel_Z_RAW/16384.0;
}
void read_gyro()
{
uint8_t Rx_data[6];
MPU_Read(MPU6050_ADDR, GYRO_XOUT_H_REG, Rx_data, 6);
Gyro_X_RAW =(int16_t)(Rx_data[0]<<8 | Rx_data[1]);
Gyro_Y_RAW =(int16_t)(Rx_data[2]<<8 | Rx_data[3]);
Gyro_Z_RAW =(int16_t)(Rx_data[4]<<8 | Rx_data[5]);
Gx = Gyro_X_RAW/131.0; // FS_SEL = 0. So dividing by 131.0
Gy = Gyro_Y_RAW/131.0;
Gz= Gyro_Z_RAW/131.0;
}
void EXTI0_IRQHandler( )
{EXTI->PR |= (1<<0);//Clear interrupt flag
flag=1;
}
void Delay(uint32_t number)
{
for(uint32_t i=0;i<number*500;i++)
{
__NOP();
}
}
int main()
{
TIM_Config();
GPIO_configuration();
PWM_configuration();
I2C_Config ();
MPU6050_Init ();
EXT_Init();
while (1)
{
if(flag==1){
Delay(40);// require a delay of 40ms because time constant of filter is set to
40ms
dt = 40; // time constant of complimentary filter= 40ms
read_acc();
Acc_angle = atan2(Ay,Az)*deg;
read_gyro();
Gyro_angle=Gz*dt*deg ;
angle=0.99*(Gyro_angle)+0.01*(Acc_angle);
error=ref-angle;
sum_of_errors+=error*dt;
duty=(Kp*error + (Kd*1000*(error)/dt)+Ki*sum_of_errors/1000)/100000;
if(duty<0)
duty=-duty;
if(error>0)// robot falling forward
{
// move robot forward
GPIOA->BSRR |=(0x1<<18)|(0x1<<20); // clr pins A2 and A4 i.e IN1
and IN3 of L289n
GPIOA->BSRR|=((1<<1)|(1<<3)); // set pins A1 and A3 i.e IN1
//and IN3 of L289N // IN1=1 and IN2=0
// IN3=1 and IN4=0
TIM4->CCR2=0; //13....away from arrow.Motor 2
TIM4->CCR4=0; //15....along arrow....Motor1
TIM4->CCR3=duty; //14...along arrow...Motor2
TIM4->CCR1=duty+250; //12...away from arrow.Motor 1
}
else if(error<0) // robot falling backwards
{
// move robot backwards
GPIOA->BSRR |=(0x1<<17)|(0x1<<19); // clr pins A1 and A3 i.e IN1
and IN3 of L289n
GPIOA->BSRR |=(0x1<<2)|(0x1<<4); // set pins A2 and pins A4 i.2
IN2 and IN4 of L289n
// IN1=0 and IN2=1
// IN3=0 and IN4=1
TIM4->CCR2=duty; //13....away from arrow.Motor 2
TIM4->CCR4=duty+250; //15....along arrow....Motor1
TIM4->CCR3=0; //14...along arrow...Motor2
TIM4->CCR1=0; //12...away from arrow.Motor 1
}
else if(error==0){ // robot in mean position
// stop the motor
TIM4->CCR1=0;
TIM4->CCR2=0;
TIM4->CCR3=0;
TIM4->CCR4=0;
GPIOA->BSRR=0;
GPIOA->BSRR=0;
}
flag=0;
}
//Delay(1000);
}
}
