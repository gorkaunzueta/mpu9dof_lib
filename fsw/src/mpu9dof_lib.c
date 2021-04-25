/*
 * MikroSDK - MikroE Software Development Kit
 * Copyright© 2020 MikroElektronika d.o.o.
 * 
 * Permission is hereby granted, free of charge, to any person 
 * obtaining a copy of this software and associated documentation 
 * files (the "Software"), to deal in the Software without restriction, 
 * including without limitation the rights to use, copy, modify, merge, 
 * publish, distribute, sublicense, and/or sell copies of the Software, 
 * and to permit persons to whom the Software is furnished to do so, 
 * subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be 
 * included in all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, 
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. 
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, 
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE 
 * OR OTHER DEALINGS IN THE SOFTWARE. 
 */

/*!
 * \file
 *
 */

#include "mpu9dof_lib.h"
#include "bcm2835_lib.h"

#include "cfe.h"

// ------------------------------------------------ PUBLIC FUNCTION DEFINITIONS

void mpu9dof_cfg_setup ( mpu9dof_cfg_t *cfg )
{
 
    cfg->i2c_address = MPU9DOF_XLG_I2C_ADDR_0;
    cfg->i2c_mag_address = MPU9DOF_M_I2C_ADDR_0;
}

MPU9DOF_RETVAL mpu9dof_init ( mpu9dof_t *ctx, mpu9dof_cfg_t *cfg )
{

    ctx->slave_address = cfg->i2c_address;
    ctx->magnetometer_address = cfg->i2c_mag_address;
    
    return MPU9DOF_OK;
}

void mpu9dof_default_cfg ( mpu9dof_t *ctx )
{
    uint8_t command;

    // Chip reset 
    command = MPU9DOF_BIT_H_RESET;
    mpu9dof_generic_write( ctx, MPU9DOF_PWR_MGMT_1, &command, 1 );
    delay(10);

    // Initialize accel & gyro 
    command = MPU9DOF_DEFAULT;
    mpu9dof_generic_write( ctx, MPU9DOF_SMPLRT_DIV, &command, 1 );
    delay(10);

    command = MPU9DOF_BITS_DLPF_CFG_42HZ;
    mpu9dof_generic_write( ctx, MPU9DOF_CONFIG, &command, 1 );
    delay(10);

    command = MPU9DOF_BITS_FS_1000DPS;
    mpu9dof_generic_write( ctx, MPU9DOF_GYRO_CONFIG, &command, 1 );
    delay(10);

    command = MPU9DOF_BITS_AFSL_SEL_8G;
    mpu9dof_generic_write( ctx, MPU9DOF_ACCEL_CONFIG, &command, 1 );
    delay(10);

    // Disable FIFOs
    command = MPU9DOF_BIT_FIFO_DIS;
    mpu9dof_generic_write( ctx, MPU9DOF_FIFO_EN , &command, 1 );
    delay(10);

    // Bypass mode enabled
    command = MPU9DOF_BIT_INT_PIN_CFG;
    mpu9dof_generic_write( ctx, MPU9DOF_INT_PIN_CFG , &command, 1 );
    delay(10);

    // Disable all interrupts
    command = MPU9DOF_DEFAULT;
    mpu9dof_generic_write( ctx, MPU9DOF_INT_ENABLE , &command, 1 );
    delay(10);

    // No FIFO and no I2C slaves
    command = MPU9DOF_DEFAULT;
    mpu9dof_generic_write( ctx, MPU9DOF_USER_CTRL , &command, 1 );
    delay(10);

    // No power management, internal clock source
    command = MPU9DOF_DEFAULT;
    mpu9dof_generic_write( ctx, MPU9DOF_PWR_MGMT_1, &command, 1 );
    delay(10);
    command = MPU9DOF_DEFAULT;
    mpu9dof_generic_write( ctx, MPU9DOF_PWR_MGMT_2, &command, 1 );
    delay(10);

    // Initialize magnetometer 
    mpu9dof_write_data_mag( ctx, MPU9DOF_MAG_CNTL, MPU9DOF_BIT_RAW_RDY_EN );
    delay(10);
}

void mpu9dof_generic_write ( mpu9dof_t *ctx, uint8_t reg, uint8_t *data_buf, uint8_t len )
{
    char tx_buf[ 256 ];
    uint8_t cnt;
    
    tx_buf[ 0 ] = reg;
    
    for ( cnt = 1; cnt <= len; cnt++ )
    {
        tx_buf[ cnt ] = data_buf[ cnt - 1 ]; 
    }
    
    bcm2835_i2c_setSlaveAddress(ctx->slave_address);
    bcm2835_i2c_write( tx_buf, len + 1 ); 
}

void mpu9dof_generic_read ( mpu9dof_t *ctx, uint8_t reg, char *data_buf, uint8_t len )
{
    char tx_buf[ 1 ];

    tx_buf [ 0 ] = reg;

    
    bcm2835_i2c_setSlaveAddress(ctx->slave_address);
    bcm2835_i2c_write_read_rs( tx_buf, 1, data_buf, len );
}

// Generic write data function MPU-9150 MAG 
void mpu9dof_write_data_mag ( mpu9dof_t *ctx, uint8_t address, uint8_t write_command )
{
    char tx_buf[ 2 ];

    tx_buf[ 0 ] = address;
    tx_buf[ 1 ] = write_command;
    
    bcm2835_i2c_setSlaveAddress(ctx->magnetometer_address);
    bcm2835_i2c_write( tx_buf, 2 ); 
}

// Generic read data function MPU-9150 MAG 
char mpu9dof_read_data_mag ( mpu9dof_t *ctx, uint8_t address )
{
    char write_reg[ 1 ];
    char read_reg[ 1 ];

    write_reg[ 0 ] = address;
    
    bcm2835_i2c_setSlaveAddress(ctx->magnetometer_address);
    bcm2835_i2c_write_read_rs( write_reg, 1, read_reg, 1 );

    return read_reg[ 0 ];
}

// Function get data from MPU-9150 XL G register 
int16_t mpu9dof_get_axis ( mpu9dof_t *ctx, uint8_t adr_reg_high )
{
    uint16_t result;
    char buffer[ 2 ];

    mpu9dof_generic_read( ctx, adr_reg_high, buffer, 2 );

    result = buffer[ 0 ];
    result <<= 8;
    result |= buffer[ 1 ];

    return result;
}

// Function get data from MPU-9150 MAG register
int16_t mpu9dof_get_axis_mag ( mpu9dof_t *ctx, uint8_t adr_reg_low )
{
    uint16_t result;
    uint8_t buffer[ 2 ];
    
    mpu9dof_write_data_mag( ctx, MPU9DOF_MAG_CNTL, 0x01);
    delay(10);

    buffer[ 0 ] = mpu9dof_read_data_mag( ctx, adr_reg_low + 1 );
    buffer[ 1 ] = mpu9dof_read_data_mag( ctx, adr_reg_low );

    result = buffer[ 0 ];
    result <<= 8;
    result |= buffer[ 1 ];

    return result;
}

// Function read Gyro X-axis, Y-axis and Z-axis axis
void mpu9dof_read_gyro ( mpu9dof_t *ctx, int16_t *gyro_x, int16_t *gyro_y, int16_t *gyro_z )
{
    *gyro_x = mpu9dof_get_axis( ctx, MPU9DOF_GYRO_XOUT_H );
    *gyro_y = mpu9dof_get_axis( ctx, MPU9DOF_GYRO_YOUT_H );
    *gyro_z = mpu9dof_get_axis( ctx, MPU9DOF_GYRO_ZOUT_H );
}

// Function read Accel X-axis, Y-axis and Z-axis 
void mpu9dof_read_accel ( mpu9dof_t *ctx, int16_t *accel_x, int16_t *accel_y, int16_t *accel_z )
{
    *accel_x = mpu9dof_get_axis( ctx, MPU9DOF_ACCEL_XOUT_H );
    *accel_y = mpu9dof_get_axis( ctx, MPU9DOF_ACCEL_YOUT_H );
    *accel_z = mpu9dof_get_axis( ctx, MPU9DOF_ACCEL_ZOUT_H );
}

// Function read Magnetometar X-axis, Y-axis and Z-axis 
void mpu9dof_read_mag ( mpu9dof_t *ctx, int16_t *mag_x, int16_t *mag_y, int16_t *mag_z )
{
    *mag_x = mpu9dof_get_axis_mag( ctx, MPU9DOF_MAG_XOUT_L );
    *mag_y = mpu9dof_get_axis_mag( ctx, MPU9DOF_MAG_YOUT_L );
    *mag_z = mpu9dof_get_axis_mag( ctx, MPU9DOF_MAG_ZOUT_L );
}

// Function read Temperature data from MPU-9150 XL G register
float mpu9dof_read_temperature ( mpu9dof_t *ctx )
{
    int16_t result;
    float temperature;
    temperature = 0.00;

    result = mpu9dof_get_axis( ctx, MPU9DOF_TEMP_OUT_H );
    delay(10);

    temperature =  ( float ) result;
    temperature -= 21;
    temperature /= 333.87;
    temperature += 21;

    return temperature;
}

// Function of initialization for the cFS
int32 MPU9DOF_LIB_Init(void)
{
    /*
     * Initialize the variables for the default configuration and read
     * the WHO AM I register. If the response is not the expected avoid 
     * implementation
     */
    
    // Definition of class and variables
    mpu9dof_cfg_t   mpu9dofconfig;
    mpu9dof_t       mpu9dofclass; 
    char            RxBuffer[1] = {0};
    
    // Initialize the i2c
    if(!bcm2835_i2c_begin()){
        OS_printf("MPU9DOF Lib: I2C begin failed \n");
        return CFE_STATUS_NOT_IMPLEMENTED;
    } 
    
    // Set the baudrate to the standard freq. 100 KHz
    bcm2835_i2c_set_baudrate(BAUDRATE);
    
    // Initialize classes for the mpu9dof config
    mpu9dof_cfg_setup ( &mpu9dofconfig );
    mpu9dof_init ( &mpu9dofclass, &mpu9dofconfig );
    mpu9dof_default_cfg ( &mpu9dofclass );
    
    // Read the WHO AM I register
    mpu9dof_generic_read ( &mpu9dofclass, MPU9DOF_WHO_AM_I_XLG, RxBuffer, 1 );
    
    // Check for the expected value
    if (RxBuffer[0] != 0x71){
        OS_printf("MPU9DOF Lib not implemented. Sensor not found. Response: %X", RxBuffer[0]);
        return CFE_STATUS_NOT_IMPLEMENTED;
    }
    
    OS_printf("MPU9DOF Lib Initialized.\n");

    return CFE_SUCCESS;

}

// ------------------------------------------------------------------------- END

