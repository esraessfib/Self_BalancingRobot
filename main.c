#include "stm32f4xx.h"
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

// ----- MPU6050 I2C definitions -----
#define MPU_ADDR         (0x68 << 1)
#define WHO_AM_I         0x75
#define PWR_MGMT_1       0x6B
#define CONFIG           0x1A
#define GYRO_CONFIG      0x1B
#define ACCEL_CONFIG     0x1C
#define ACCEL_XOUT_H     0x3B
#define XA_OFFSET_H      0x06
#define XG_OFFSET_H      0x13
#define ACCEL_FULL_SCALE_2_G     0x00
#define GYRO_FULL_SCALE_250_DPS  0x00
// ----- Filter & PID -----
#define PITCH_OFFSET  0.0f    
#define RAD_TO_DEG   57.295779513f
#define ALPHA        0.95f
#define DT           0.01f   // 100 Hz
#define KP           10.0f 
#define KI           0.05f 
#define KD           0.1f  

// ----- Safety Thresholds -----
#define MAX_I_TERM   1.0f  // Integral anti-windup limit
#define MAX_POWER    500.0f  // Maximum motor power
#define MIN_POWER    100.0f 
//#define ANGLE_DEADBAND 0.5f

// ----- Globals -----
float filtered_derivative = 0.0f;
volatile int flag = 0;
int16_t accel_raw[3], gyro_raw[3];
int16_t accel_offset[3], gyro_offset[3];
uint8_t axis_map[3]= {0,1,2};
int8_t  axis_dir[3]= {1,1,1};
float pitch = 0.0f, I_term = 0.0f, sum_of_error = 0.0f;
float gyro_angle_x = 0, gyro_angle_y = 0, gyro_angle_z = 0;
float prev_error = 0.0f;  // Store previous error for derivative calculation
uint8_t control_enabled = 0;  // Flag to enable/disable control system

// ----- Prototypes -----
void hse_clk(void);
void config_I2C1(void);
uint8_t I2C1_read_byte(uint8_t, uint8_t);
void I2C1_write_byte(uint8_t, uint8_t, uint8_t);
void I2C1_read_bytes(uint8_t, uint8_t, uint8_t*, uint8_t);
void config_USART3(void);
void Sendchar_USART3(char c);
void SendString_USART3(char *pt);
void config_TIM3_PWM(void);
void config_TIM2_Int(void);
void TIM2_IRQHandler(void);
void MPU6050_init(void);
void MPU6050_calibrate(uint16_t);
void MPU6050_set_axes(uint8_t, int8_t, uint8_t, int8_t, uint8_t, int8_t);
void MPU6050_read_data(void);
void calculate_accel_angles(float*, float*);
void update_angles(void);
void control_motors(float control);
void delay_ms(uint32_t ms);
uint8_t check_mpu_connection(void);


// ----- Implémentations -----

void hse_clk(void) {
    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR & RCC_CR_HSERDY));
    RCC->CFGR = RCC_CFGR_SW_0;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_0);
}
//------------------------------
// USART3 Configuration (9600 baud)
// Using PB10 (TX) and PB11 (RX) with AF7
//------------------------------
void config_USART3(void) {
       // Enable clock for GPIOB and USART3
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
    
    // Configure PB10 and PB11 as Alternate Function mode

    GPIOB->MODER |=  ((2 << 20) | (2 << 22));
    
    // Set AF7 (USART3) for PB10 and PB11
 
    GPIOB->AFR[1] |=  ((7 << 8) | (7 << 12));
    
    // Set high speed for these pins
    GPIOB->OSPEEDR |= ((3 << 20) | (3 << 22));
    
    // Calculate baud rate for 9600 baud:
	  // HSE = 8 MHz
    // USARTDIV = 8,000,000 / (16 * 9600) = 52,08
    USART3->BRR = 0x341; // 9600
    USART3->CR1 |= USART_CR1_TE | USART_CR1_RE;
    
    // Enable USART3
    USART3->CR1 |= USART_CR1_UE;   
}
// Function to send a single character via USART3
void Sendchar_USART3(char c)
{
    while (!(USART3->SR & USART_SR_TXE)); 
    USART3->DR = c;
}

// Function to send a string via USART3
void SendString_USART3(char *pt)
{
    while (*pt)
    {
        Sendchar_USART3(*pt++);
    }
}

void config_TIM3_PWM(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    // Configure PB0 (CH1), PB5 (CH2), PB4 (CH3), PB1 (CH4) as alternate function (AF2)
    GPIOB->MODER |= (2<<0) | (2<<2) | (2<<8) | (2<<10);
    GPIOB->AFR[0] |= (2<<0) | (2<<4) | (2<<16) | (2<<20);
    // PD12/13 outputs direction
    GPIOD->MODER |= (1<<(12*2))|(1<<(13*2));
    // Timer
    TIM3->PSC = 7;    
    TIM3->ARR = 99;   // 1MHz/1000=1kHz //999
    // Configure channel 1
    TIM3->CCMR1 |= (6<<4);
	   // Configure Channel 2 (PB5)
    TIM3->CCMR1 |= (6<<12);          
    // Configure channel 3
    TIM3->CCMR2 |= (6<<4);
	  // Configure Channel 4 (PB1)
    TIM3->CCMR2 |= (6<<12);
		
    TIM3->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;		
    TIM3->CCR1 = 0;
    TIM3->CCR2 = 0;
    TIM3->CCR3 = 0;
    TIM3->CCR4 = 0;
	  TIM3->CR1 |= TIM_CR1_CEN;
}
//------------------------------
// I2C1 Configuration (Standard Mode 100 kHz)
//------------------------------
void config_I2C1(void)
{
    // Enable GPIOB clock (for PB6=SCL, PB7=SDA)
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    
    // Configure PB6 and PB7 for Alternate Function mode:
    
    GPIOB->MODER |=  GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1;
    
    // Configure as open-drain and enable pull-up resistors
    GPIOB->OTYPER |= GPIO_OTYPER_OT6 | GPIO_OTYPER_OT7;
    
    // Set high speed
    GPIOB->OSPEEDR |= (3 << (6*2)) | (3 << (7*2));
    
    // Select Alternate Function AF4 for I2C1 on PB6 and PB7.
    GPIOB->AFR[0] |=  GPIO_AFRL_AFRL6_2 | GPIO_AFRL_AFRL7_2;
    
    // Enable I2C1 clock
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
    
    // Reset I2C1 peripheral
    RCC->APB1RSTR |= RCC_APB1RSTR_I2C1RST;
    RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C1RST;
    
    // Configure I2C1:
    // Set peripheral clock frequency in CR2 (APB1 clock = 8 MHz)
    I2C1->CR2 = 8;
    // Standard mode (100 kHz): clear FS bit
    I2C1->CCR &= ~I2C_CCR_FS;
    I2C1->CCR = 40; //CCR = APB1_clock / (2 * I2C_desired_frequency)
    I2C1->TRISE = 9; //TRISE = (max_rise_time_ns / 1000) * APB1_clock + 1= (1000 / 1000) * 8 + 1 = 9
    
    // Enable I2C1 peripheral
    I2C1->CR1 |= I2C_CR1_PE;
}
//------------------------------
// I2C1 Read Byte from Slave Register
//------------------------------
uint8_t I2C1_read_byte(uint8_t slave_addr, uint8_t reg_addr)
{
    uint8_t data;
    
    // Generate START condition
    I2C1->CR1 |= I2C_CR1_START;
    while (!(I2C1->SR1 & I2C_SR1_SB));
    
    // Send slave address with Write (last bit 0)
    I2C1->DR = slave_addr;
    while (!(I2C1->SR1 & I2C_SR1_ADDR));
    (void)I2C1->SR2;  // Clear ADDR flag
    
    // Send register address
    while (!(I2C1->SR1 & I2C_SR1_TXE));
    I2C1->DR = reg_addr;
    while (!(I2C1->SR1 & I2C_SR1_BTF));
    
    // Generate a repeated START condition
    I2C1->CR1 |= I2C_CR1_START;
    while (!(I2C1->SR1 & I2C_SR1_SB));
    
    // Send slave address with Read bit (bit0=1)
    I2C1->DR = slave_addr | 1;
    while (!(I2C1->SR1 & I2C_SR1_ADDR));
    (void)I2C1->SR2;
    
    // Disable ACK for single byte reception
    I2C1->CR1 &= ~I2C_CR1_ACK;
    
    // Wait for data reception
    while (!(I2C1->SR1 & I2C_SR1_RXNE));
    data = I2C1->DR;
    
    // Generate STOP condition
    I2C1->CR1 |= I2C_CR1_STOP;
    
    // Re-enable ACK for future receptions
    I2C1->CR1 |= I2C_CR1_ACK;
    
    return data;
}
//------------------------------
// Write single byte to I2C device
//------------------------------
void I2C1_write_byte(uint8_t slave_addr, uint8_t reg_addr, uint8_t data)
{
    // Start and send slave address (write mode)
    I2C1->CR1 |= I2C_CR1_START;
    while (!(I2C1->SR1 & I2C_SR1_SB));
    
    I2C1->DR = slave_addr;
    while (!(I2C1->SR1 & I2C_SR1_ADDR));
    (void)I2C1->SR2;
    
    // Send register address
    while (!(I2C1->SR1 & I2C_SR1_TXE));
    I2C1->DR = reg_addr;
    while (!(I2C1->SR1 & I2C_SR1_BTF));
    
    // Send data and stop
    I2C1->DR = data;
    while (!(I2C1->SR1 & I2C_SR1_BTF));
    
    I2C1->CR1 |= I2C_CR1_STOP;
}
//------------------------------
// Read multiple bytes from I2C device
//------------------------------
void I2C1_read_bytes(uint8_t slave_addr, uint8_t reg_addr, uint8_t *buffer, uint8_t length)
{
    // Start and send slave address (write mode)
    I2C1->CR1 |= I2C_CR1_START;
    while (!(I2C1->SR1 & I2C_SR1_SB));
    
    I2C1->DR = slave_addr;
    while (!(I2C1->SR1 & I2C_SR1_ADDR));
    (void)I2C1->SR2;
    
    // Send register address
    while (!(I2C1->SR1 & I2C_SR1_TXE));
    I2C1->DR = reg_addr;
    while (!(I2C1->SR1 & I2C_SR1_BTF));
    
    // Repeated start for read
    I2C1->CR1 |= I2C_CR1_START;
    while (!(I2C1->SR1 & I2C_SR1_SB));
    
    I2C1->DR = slave_addr | 1;  // Read mode
    while (!(I2C1->SR1 & I2C_SR1_ADDR));
    (void)I2C1->SR2;
    
    // Enable ACK for multi-byte
    I2C1->CR1 |= I2C_CR1_ACK;
    
    // Read all bytes
    for (uint8_t i = 0; i < length; i++) {
        if (i == length - 1) {
            I2C1->CR1 &= ~I2C_CR1_ACK;  // Last byte: no ACK
        }
        
        while (!(I2C1->SR1 & I2C_SR1_RXNE));
        buffer[i] = I2C1->DR;
        
        if (i == length - 1) {
            I2C1->CR1 |= I2C_CR1_STOP;
        }
    }
    
    I2C1->CR1 |= I2C_CR1_ACK;  // Re-enable ACK
}
//------------------------------
// delay fonction
//------------------------------
void delay_ms(uint32_t ms)
{
    volatile uint32_t i;
    for(i = 0; i < ms * 4000; i++) { }
}
//------------------------------
// Initialize MPU6050
//------------------------------
void MPU6050_init(void)
{
    // Reset the device
    I2C1_write_byte(MPU_ADDR, PWR_MGMT_1, 0x80);
    delay_ms(100);
    
    // Wake up and select clock source
	 //This wakes up the device from sleep mode and selects the PLL with X-axis gyroscope as the clock reference
    I2C1_write_byte(MPU_ADDR, PWR_MGMT_1, 0x01);
    delay_ms(10);
    
    // Configure digital low-pass filter
	  //This filters out high-frequency noise, providing smoother readings
    I2C1_write_byte(MPU_ADDR, CONFIG, 0x03);
    
    // Set gyro range (±250°/s)
	 //Lower range provides higher resolution but can't measure faster rotations
    I2C1_write_byte(MPU_ADDR, GYRO_CONFIG, GYRO_FULL_SCALE_250_DPS);
    
    // Set accel range (±2g)
	 //This is the most sensitive setting, best for detecting small movements
    I2C1_write_byte(MPU_ADDR, ACCEL_CONFIG, ACCEL_FULL_SCALE_2_G);
}
//------------------------------
// Read MPU6050 sensor data
//------------------------------
void MPU6050_read_data(void)
{
    uint8_t buffer[14];
    int16_t raw_data[6];
    
    // Read all data at once (accel + temp + gyro)
	  //6 for accelerometer, 2 for temperature, 6 for gyroscope
	
    I2C1_read_bytes(MPU_ADDR, ACCEL_XOUT_H, buffer, 14);
    
    // Convert to raw values (big endian)
	  // Each axis takes 2 bytes, with the high byte first (big endian format)
	 // converts the byte pairs into 16-bit integers
    for (int i = 0; i < 3; i++) {
        // Accelerometer X, Y, Z
        raw_data[i] = (buffer[i*2] << 8) | buffer[i*2+1];
    }
    
    for (int i = 0; i < 3; i++) {
        // Gyroscope X, Y, Z (skip temperature at idx 6,7)
        raw_data[i+3] = (buffer[i*2+8] << 8) | buffer[i*2+9];
    }
    
    // Apply offsets and mapping
    for (int i = 0; i < 3; i++) {
        // Apply offset
        int16_t accel_val = raw_data[i] + accel_offset[i];
        int16_t gyro_val = raw_data[i+3] + gyro_offset[i];
        
        // Apply mapping
        uint8_t accel_idx = axis_map[i];
        uint8_t gyro_idx = axis_map[i];
        
        // Apply direction (with correct mapping)(+1 or -1)
        accel_raw[accel_idx] = accel_val * axis_dir[i];
        gyro_raw[gyro_idx] = gyro_val * axis_dir[i];
    }
}
//------------------------------
// Set axis mapping
//------------------------------
void MPU6050_set_axes(uint8_t x_axis, int8_t x_dir, uint8_t y_axis, int8_t y_dir, uint8_t z_axis, int8_t z_dir)
{
    axis_map[0] = x_axis;
    axis_map[1] = y_axis;
    axis_map[2] = z_axis;
    
    axis_dir[0] = x_dir;
    axis_dir[1] = y_dir;
    axis_dir[2] = z_dir;
}
//------------------------------
// Calibrate MPU6050
//------------------------------
void MPU6050_calibrate(uint16_t samples)
{
  	//"samples":the number of individual readings that will be taken from the sensor during the calibration process
    // For storing sums
    int32_t accel_sum[3] = {0};
    int32_t gyro_sum[3] = {0};
    uint8_t buffer[14];
   
    
    SendString_USART3("Calibrating MPU6050...\r\n");
    delay_ms(1000);
    
    // Take multiple samples
    for (uint16_t i = 0; i < samples; i++) {
        // Read raw data directly
        I2C1_read_bytes(MPU_ADDR, ACCEL_XOUT_H, buffer, 14);
        
        // Process and sum accelerometer data (x,y,z)
        for (int j = 0; j < 3; j++) {
            int16_t accel_val = (buffer[j*2] << 8) | buffer[j*2+1];
            accel_sum[j] += accel_val;
        }
        
        // Process and sum gyro data (x,y,z)
        for (int j = 0; j < 3; j++) {
            int16_t gyro_val = (buffer[j*2+8] << 8) | buffer[j*2+9];
            gyro_sum[j] += gyro_val;
        }
        
        delay_ms(2);
    }
    
    // Calculate average offsets
    for (int i = 0; i < 3; i++) {
        accel_offset[i] = -(accel_sum[i] / samples);
    }
    
    // Remove gravity from Z axis (assuming +Z is up)
		//This makes Z read 0 when the device is level
    accel_offset[2] += 16384; // 1g at ±2g setting
    
    // Calculate gyro offsets
    for (int i = 0; i < 3; i++) {
        gyro_offset[i] = -(gyro_sum[i] / samples);
    }
    
     for(int i=0;i<3;i++){
        I2C1_write_byte(MPU_ADDR,XA_OFFSET_H+i*2,(accel_offset[i]>>8)&0xFF);
        I2C1_write_byte(MPU_ADDR,XA_OFFSET_H+i*2+1,accel_offset[i]&0xFF);
        I2C1_write_byte(MPU_ADDR,XG_OFFSET_H+i*2,(gyro_offset[i]>>8)&0xFF);
        I2C1_write_byte(MPU_ADDR,XG_OFFSET_H+i*2+1,gyro_offset[i]&0xFF);
		 }
     SendString_USART3("Calibration complete!\r\n");
}
//----------------------------
//Calculate angles from accelerometer data
//----------------------------
void calculate_accel_angles(float*ax,float*ay){
	  // Convert raw accelerometer values to g units
    // At +/-2g sensitivity, 16384 LSB/g
   
	  float X=accel_raw[0]/16384.0f, Y=accel_raw[1]/16384.0f, Z=accel_raw[2]/16384.0f;
 
	// Calculate pitch and roll using accelerometer data
    // atan2 returns angle in radians, so convert to degrees
    
	  *ax = atan2(Y,sqrt(X*X+Z*Z))*RAD_TO_DEG;
    *ay = atan2(-X,sqrt(Y*Y+Z*Z))*RAD_TO_DEG;
}
//-----------------------------
// Update angles using both accelerometer and gyroscope data
//------------------------------
void update_angles(void)
{
    // Get current sensor readings
    MPU6050_read_data();
    
    // Convert gyro values to degrees per second
    // At +/-250 deg/s sensitivity, 131 LSB/(deg/s)
   
  	float gx=gyro_raw[0]/131.0f;
    
	   // Integrate gyro rates to get angles
   
    	//gyro_angle_x += gx*DT;
   	// Get accelerometer angles
   
  	float ax,ay; 
	  calculate_accel_angles(&ax,&ay);
    
    // Complementary filter - combine accelerometer and gyroscope data
    // High pass filter on gyro, low pass filter on accelerometer
   
     pitch = ALPHA * (pitch + gx*DT) + (1-ALPHA) * ax;
}
//------------------------------
// Function to check if MPU6050 is connected
//------------------------------
uint8_t check_mpu_connection(void) {
     return (I2C1_read_byte(MPU_ADDR, WHO_AM_I) == 0x68);
 }
//------------------------------
// Function to format and display angles
//------------------------------

void config_TIM2_Int(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    TIM2->PSC = 1599;  // 84MHz/8400=10kHz
    TIM2->ARR = 99;    // 10kHz/100=100Hz
    TIM2->DIER |= TIM_DIER_UIE;
    NVIC_SetPriority(TIM2_IRQn, 2);
    NVIC_EnableIRQ(TIM2_IRQn);
}

void control_motors(float control)
{  
   // Stop immediately if control == 0
    if (control == 0.0f) {
        TIM3->CCR1 = TIM3->CCR2 = TIM3->CCR3 = TIM3->CCR4 = 0;
        return;
		}
    // Limit maximum power
    if (control > MAX_POWER) control = MAX_POWER;
    if (control < -MAX_POWER) control = -MAX_POWER;
     
		//  If too small to overcome friction, bump it
    if (fabsf(control) < MIN_POWER) {
        control = (control > 0)
          ?  MIN_POWER    // small +control ? +MIN_POWER
          : -MIN_POWER;   // small –control ? -MIN_POWER
    }
    
    // When robot tilts forward (positive pitch error), motors should go forward
    // When robot tilts backward (negative pitch error), motors should go backward
    if (control > 0) {
        TIM3->CCR1 = (uint16_t)(control);  
        TIM3->CCR2 = 0;
        TIM3->CCR3 = (uint16_t)(control);  
        TIM3->CCR4 = 0;  
			// Move forward to catch falling forward robot
       
    } else {
     // Move backward to catch falling backward robot
        TIM3->CCR1 = 0;
        TIM3->CCR2 = (uint16_t)(-control);    
        TIM3->CCR3 = 0;
        TIM3->CCR4 = (uint16_t)(-control);
    }
}
void TIM2_IRQHandler(void) {
    if (TIM2->SR & TIM_SR_UIF) {
        TIM2->SR &= ~TIM_SR_UIF;
        update_angles();
		
        
         if (control_enabled) {
            // Calculate error
            float error = pitch - (PITCH_OFFSET); 
           
            sum_of_error += error * DT;					 
            float derivative = (error - prev_error) / DT;
            prev_error = error;

						float out = KP * error + KD * derivative+ KI * sum_of_error ;
            
						 
            
            if (out > MAX_POWER) out = MAX_POWER;
            if (out < -MAX_POWER) out = -MAX_POWER;
            
            control_motors(out);
            
            char buffer[100];
            sprintf(buffer, "Pitch: %.2f, Err: %.2f, Out: %.2f\r\n", 
                    pitch, error, out);			
            SendString_USART3(buffer);
         }
    
}
}


void find_robot_pitch(void) 
{
    SendString_USART3("Finding balance point...\r\n");
    SendString_USART3("Hold robot vertically \r\n");
    
    // Wait for button press while showing current pitch
    
        update_angles();
        char buffer[50];
        sprintf(buffer, "Current pitch: %.2f\r\n", pitch);
        SendString_USART3(buffer);
}
			
				
int main(void) {
    hse_clk();
    config_I2C1();
    config_USART3();
    config_TIM3_PWM();
    delay_ms(700);
    SendString_USART3("Balancing Robot starting...\r\n");
    delay_ms(100);
    find_robot_pitch();
    // Initialize and check MPU6050
    if (check_mpu_connection()) {
        SendString_USART3("MPU6050 connected\r\n");
        MPU6050_init();
        MPU6050_calibrate(100);
        // Set axis mapping to match robot orientation
        
			  MPU6050_set_axes(1, -1, 0, 1, 2, 1);
			
			          // Configure timer for control loop
      config_TIM2_Int();
			SendString_USART3("System ready! Press button to start control.\r\n");
			control_enabled = 1;
      sum_of_error = 0.0f;  // Reset integral term on start
      pitch = 0.0f;         // Reset pitch angle
			prev_error = 0.0f;    // Reset derivative term
      SendString_USART3("Control enabled\r\n");
      TIM2->CR1 |= TIM_CR1_CEN;  // Start the control timer
    } else {
        SendString_USART3("mpu is not connected\r\n");
        while (1);
     }

    while (1) {
			

    }
}