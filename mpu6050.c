#include "mpu6050.h"
#include "i2c_f3.h"
#include "math.h"
#include "MadgwickAHRS.h"
#include "uart.h"
#include "spi_f3.h"

volatile float roll;
volatile float pitch;
volatile float yaw;
volatile float AccX;
volatile float AccY;
volatile float AccZ;
volatile float GyroX;
volatile float GyroY;
volatile float GyroZ;
volatile float mag_data[3];
volatile float Magnetometer_ASA[3];
u8 response[7];

#define M_PI		3.14159265358979323846f

extern volatile float q0, q1, q2, q3;

void Convert(void)
    {
    pitch = (-asinf(2.0f * (q1 * q3 - q0 * q2)))*(180.0f / M_PI);
    yaw   = (atan2f(2.0f * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3))*(180.0f / M_PI);
    roll  = (atan2f(2.0f * (q0 * q1 + q2 * q3), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3))*(180.0f / M_PI);
    }

u32 delay = 0;

#define K 0.8f
void MPU_get_data(void)
    {
    GPIOE->BSRR = GPIO_BSRR_BS_6;
    extern u8 i2c_data_get_buf[14];
    static float avg_pitch = 0;
    static float avg_roll = 0;
    AccX = ((float)((s16)((i2c_data_get_buf[0]<<8)|i2c_data_get_buf[1])))*(2.0/32768.0);
    AccY = ((float)((s16)((i2c_data_get_buf[2]<<8)|i2c_data_get_buf[3])))*(2.0/32768.0);
    AccZ = ((float)((s16)((i2c_data_get_buf[4]<<8)|i2c_data_get_buf[5])))*(2.0/32768.0);
    GyroX = ((float)((s16)((i2c_data_get_buf[8]<<8)|i2c_data_get_buf[9])))*(500.0/32768.0);
    GyroY = ((float)((s16)((i2c_data_get_buf[10]<<8)|i2c_data_get_buf[11])))*(500.0/32768.0);
    GyroZ = ((float)((s16)((i2c_data_get_buf[12]<<8)|i2c_data_get_buf[13])))*(500.0/32768.0);
    MadgwickAHRSupdateIMU(AccX,AccY,AccZ,GyroX*M_PI/180.0f,GyroY*M_PI/180.0f,GyroZ*M_PI/180.0f);//,mag_data[0],mag_data[1],-mag_data[2]);
    Convert();
    avg_pitch = (avg_pitch * (1 - K)) + (pitch * K);
    avg_roll = (avg_roll * (1 - K)) + (roll * K);
    //avg_yaw = (avg_yaw * 0.9) + (yaw * 0.1);
    if (delay < 5){delay++;}
    else
	{
	//for(u8 i = 0; i < 3; i++)
	//    {
	//    mag_data[i] = (float)(Magnetometer_ASA[i]*((float)((s16)(response[i*2+1]<<8|response[i*2]))));
	//    }
	delay = 0;
	Uart_FloatWrite(avg_pitch,3);
	Uart1_write(' ');
	Uart_FloatWrite(avg_roll,3);
	Uart1_write(' ');
	//Uart_FloatWrite(0,3);
	Uart_newLine();
	}
	//spi_writeData(MPU6050_RA_I2C_SLV0_ADDR,AK8963_I2C_ADDR|0x80);  // Set the I2C slave addres of AK8963 and set for read.
	//spi_writeData(MPU6050_RA_I2C_SLV0_REG, AK8963_HXL);            // I2C slave 0 register address from where to begin data transfer
	//spi_writeData(MPU6050_RA_I2C_SLV0_CTRL, 0x87);                 // Read 6 bytes from the magnetometer
    GPIOE->BSRR = GPIO_BSRR_BR_6;
    }

void Read_mpu9250_spi(void)
    {
    spi_readData_buf(MPU6050_RA_ACCEL_XOUT_H,i2c_data_get_buf,14);
    spi_readData_buf(MPU6050_RA_EXT_SENS_DATA_00,response,7);
    SetTask(MPU_get_data);
    //AK8963_readData();
    }


void EXTI9_5_IRQHandler()
{
 //   GPIOE->BSRR = GPIO_BSRR_BS_6;
    EXTI->PR |= EXTI_PR_PR6;
#if MPU6050_OR_MPU9250 == 1
    i2c_read_buf_int(MPU6050_ADDRESS,MPU6050_RA_ACCEL_XOUT_H,14);
#else
    SetTask(Read_mpu9250_spi);
#endif
 //   GPIOE->BSRR = GPIO_BSRR_BR_6;

}


void MPU_init(void)
    {
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
    EXTI_InitTypeDef exti;
    NVIC_InitTypeDef nvic;
    GPIOC->MODER &=~ GPIO_MODER_MODER6;

    RCC->AHBENR |= (RCC_AHBENR_GPIOEEN);
        //светодиоды на плате
    GPIOE->MODER |= (GPIO_MODER_MODER6_0);

    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource6);

    exti.EXTI_Line = EXTI_Line6;
    exti.EXTI_Mode = EXTI_Mode_Interrupt;
    exti.EXTI_Trigger = EXTI_Trigger_Rising;
    exti.EXTI_LineCmd = ENABLE;
    EXTI_Init(&exti);

    nvic.NVIC_IRQChannel = EXTI9_5_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 0;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
    //сбрасываем mpu
#if MPU6050_OR_MPU9250 == 1
    i2c_write(MPU6050_ADDRESS,MPU6050_RA_PWR_MGMT_1,0x80);
    for (volatile u32 i = 0;i<50000;i++);
    i2c_write(MPU6050_ADDRESS,MPU6050_RA_PWR_MGMT_1,0x01);
    for (volatile u32 i = 0;i<50000;i++);
    //i2c_write(MPU6050_ADDRESS,MPU6050_RA_PWR_MGMT_1,0x01);
    //500 гц частота работы
    i2c_write(MPU6050_ADDRESS,MPU6050_RA_SMPLRT_DIV,1);
    //включаем фильтр и делаем частоту тактирования гироскопа 1кгц
    i2c_write(MPU6050_ADDRESS,MPU6050_RA_CONFIG,MPU6050_DLPF_BW_188);
    //задаем диапазон гироскопа +-500 градусов в сек
    i2c_write(MPU6050_ADDRESS,MPU6050_RA_GYRO_CONFIG,MPU6050_GYRO_FS_500<<3);
    //задаем диапазон акселерометра +-2g в сек
    i2c_write(MPU6050_ADDRESS,MPU6050_RA_ACCEL_CONFIG,MPU6050_ACCEL_FS_2<<3);
    //включим прерывание
    i2c_write(MPU6050_ADDRESS,MPU6050_RA_INT_PIN_CFG,
	    1<<MPU6050_INTCFG_LATCH_INT_EN_BIT |
	    1<<MPU6050_INTCFG_INT_RD_CLEAR_BIT);
    i2c_write(MPU6050_ADDRESS,MPU6050_RA_INT_ENABLE,1<<MPU6050_INTERRUPT_DATA_RDY_BIT);
#else
    spi_writeData(MPU6050_RA_PWR_MGMT_1,0x80);
    for (volatile u32 i = 0;i<50000;i++);
    spi_writeData(MPU6050_RA_PWR_MGMT_1,0x01);
    for (volatile u32 i = 0;i<50000;i++);
    spi_writeData(MPU6050_RA_SMPLRT_DIV,1);
    //включаем фильтр и делаем частоту тактирования гироскопа 1кгц
    spi_writeData(MPU6050_RA_CONFIG,MPU6050_DLPF_BW_188);
    //задаем диапазон гироскопа +-500 градусов в сек
    spi_writeData(MPU6050_RA_GYRO_CONFIG,MPU6050_GYRO_FS_500<<3);
    //задаем диапазон акселерометра +-2g в сек
    spi_writeData(MPU6050_RA_ACCEL_CONFIG,MPU6050_ACCEL_FS_2<<3);
    AK8963_init();
    //включим прерывание
    spi_writeData(MPU6050_RA_INT_PIN_CFG,
	1<<MPU6050_INTCFG_LATCH_INT_EN_BIT |
	1<<MPU6050_INTCFG_INT_RD_CLEAR_BIT);
    spi_writeData(MPU6050_RA_INT_ENABLE,1<<MPU6050_INTERRUPT_DATA_RDY_BIT);

#endif
    }



void AK8963_calib(void)
    {
    u8 asa_raw[3];
    spi_writeData(MPU6050_RA_I2C_SLV0_ADDR,AK8963_I2C_ADDR);
    spi_writeData(MPU6050_RA_I2C_SLV0_REG,AK8963_CNTL1); // I2C slave 0 register address from where to begin data transfer
    spi_writeData(MPU6050_RA_I2C_SLV0_DO,0x0F);   // Register value to 100Hz continuous measurement in 16bit
    spi_writeData(MPU6050_RA_I2C_SLV0_CTRL,0x81);  //Enable I2C and set 1 byte
    for (volatile u32 i = 0;i<5000;i++){};
    spi_writeData(MPU6050_RA_I2C_SLV0_ADDR,AK8963_I2C_ADDR|0x80);   // Set the I2C slave    addres of AK8963 and set for read.
    spi_writeData(MPU6050_RA_I2C_SLV0_REG, AK8963_ASAX);            // I2C slave 0 register address from where to begin data transfer
    spi_writeData(MPU6050_RA_I2C_SLV0_CTRL, 0x83);                  // Read 3 bytes from the magnetometer
    for (volatile u32 i = 0;i<20000;i++){};
    spi_readData_buf(MPU6050_RA_EXT_SENS_DATA_00,asa_raw,3);
    for(u8 i = 0; i < 3; i++)
	{
        //Magnetometer_ASA[i] = (((float)(((s8)asa_raw[i])-128))/256.0+1.0);
        Magnetometer_ASA[i] = 1;
	}
}


void AK8963_init(void)
    {
    spi_writeData(MPU6050_RA_USER_CTRL,0x30);	// I2C Master mode and set I2C_IF_DIS to disable slave mode I2C bus
    spi_writeData(MPU6050_RA_I2C_MST_CTRL,0x0D);	// I2C configuration multi-master  IIC 400KHz
    //spi_writeData(MPU6050_RA_I2C_SLV0_ADDR,AK8963_I2C_ADDR|0x80); //Set the I2C slave addres of AK8963 and set for read.
    //spi_writeData(MPU6050_RA_I2C_SLV0_REG, AK8963_WIA); //I2C slave 0 register address from where to begin data transfer
    //spi_writeData(MPU6050_RA_I2C_SLV0_CTRL, 0x81); //Read 1 byte from the magnetometer
    //for(volatile u32 i = 0;i<50000;i++){};
    //return spi_readData(MPU6050_RA_EXT_SENS_DATA_00);    //Read I2C
    spi_writeData(MPU6050_RA_I2C_SLV0_ADDR,AK8963_I2C_ADDR); //Set the I2C slave addres of AK8963 and set for write.
    spi_writeData(MPU6050_RA_I2C_SLV0_REG,AK8963_CNTL2); // I2C slave 0 register address from where to begin data transfer
    spi_writeData(MPU6050_RA_I2C_SLV0_DO,0x01);  // Reset AK8963
    spi_writeData(MPU6050_RA_I2C_SLV0_CTRL,0x81); // Enable I2C and set 1 byte
    for(volatile u32 i = 0;i<1000;i++){};


    AK8963_calib();
    spi_writeData(MPU6050_RA_I2C_SLV0_ADDR,AK8963_I2C_ADDR); //Set the I2C slave addres of AK8963 and set for write.
    spi_writeData(MPU6050_RA_I2C_SLV0_REG,AK8963_CNTL1); // I2C slave 0 register address from where to begin data transfer
    spi_writeData(MPU6050_RA_I2C_SLV0_DO,0x00);   // Register value to 100Hz continuous measurement in 16bit
    spi_writeData(MPU6050_RA_I2C_SLV0_CTRL,0x81);  //Enable I2C and set 1 byte
    for(volatile u32 i = 0;i<1000;i++){};

    spi_writeData(MPU6050_RA_I2C_SLV0_ADDR,AK8963_I2C_ADDR); //Set the I2C slave addres of AK8963 and set for write.
    spi_writeData(MPU6050_RA_I2C_SLV0_REG,AK8963_CNTL2); // I2C slave 0 register address from where to begin data transfer
    spi_writeData(MPU6050_RA_I2C_SLV0_DO,0x01);  // Reset AK8963
    spi_writeData(MPU6050_RA_I2C_SLV0_CTRL,0x81); // Enable I2C and set 1 byte
    for(volatile u32 i = 0;i<1000;i++){};
    spi_writeData(MPU6050_RA_I2C_SLV0_REG,AK8963_CNTL1); // I2C slave 0 register address from where to begin data transfer
    spi_writeData(MPU6050_RA_I2C_SLV0_DO,0x16);   // Register value to 100Hz continuous measurement in 16bit
    spi_writeData(MPU6050_RA_I2C_SLV0_CTRL,0x81);  //Enable I2C and set 1 byte
    //SetTimerTask(AK8963_readData,2);
    }


void AK8963_readData(void)
    {
    spi_readData_buf(MPU6050_RA_EXT_SENS_DATA_00,response,7);
    for(volatile u32 i = 0; i < 20000; i++){}
    for(u8 i = 0; i < 3; i++)
	{
        mag_data[i] = (float)(Magnetometer_ASA[i]*((float)((s16)(response[i*2+1]<<8|response[i*2]))));
	}
    spi_writeData(MPU6050_RA_I2C_SLV0_ADDR,AK8963_I2C_ADDR|0x80);  // Set the I2C slave addres of AK8963 and set for read.
    spi_writeData(MPU6050_RA_I2C_SLV0_REG, AK8963_HXL);            // I2C slave 0 register address from where to begin data transfer
    spi_writeData(MPU6050_RA_I2C_SLV0_CTRL, 0x87);                 // Read 6 bytes from the magnetometer
    }





