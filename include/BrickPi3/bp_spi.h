#ifndef BP_SPI_H
#define BP_SPI_H

#include <stdint.h>
#include <linux/spi/spidev.h>


#define LONGEST_SPI_TRANSFER 29 // longest possible message for configuring for an I2C sensor
#define LONGEST_I2C_TRANSFER 16 // longest possible I2C read/write

#define SPI_TARGET_SPEED 500000 // SPI target speed of 500kbps

#define SPIDEV_FILE_NAME "/dev/spidev0.1" // File name of SPI

// Error values
#define ERROR_NONE 0
#define ERROR_SPI_FILE -1
#define ERROR_SPI_RESPONSE -2
#define ERROR_WRONG_MANUFACTURER -3
#define ERROR_WRONG_DEVICE -4
#define ERROR_FIRMWARE_MISMATCH -5
#define ERROR_SENSOR_TYPE_MISMATCH -6

extern int spi_file_handle;                    // SPI file handle
extern struct spi_ioc_transfer spi_xfer_struct;     // SPI transfer struct
extern uint8_t spi_array_out[LONGEST_SPI_TRANSFER]; // SPI out array
extern uint8_t spi_array_in[LONGEST_SPI_TRANSFER];  // SPI in array

// SPI message type
enum BPSPI_MESSAGE_TYPE
{
    BPSPI_MESSAGE_NONE,

    BPSPI_MESSAGE_GET_MANUFACTURER, // 1
    BPSPI_MESSAGE_GET_NAME,
    BPSPI_MESSAGE_GET_HARDWARE_VERSION,
    BPSPI_MESSAGE_GET_FIRMWARE_VERSION,
    BPSPI_MESSAGE_GET_ID,
    BPSPI_MESSAGE_SET_LED,
    BPSPI_MESSAGE_GET_VOLTAGE_3V3,
    BPSPI_MESSAGE_GET_VOLTAGE_5V,
    BPSPI_MESSAGE_GET_VOLTAGE_9V,
    BPSPI_MESSAGE_GET_VOLTAGE_VCC,
    BPSPI_MESSAGE_SET_ADDRESS, // 11

    BPSPI_MESSAGE_SET_SENSOR_TYPE, // 12

    BPSPI_MESSAGE_GET_SENSOR_1, // 13
    BPSPI_MESSAGE_GET_SENSOR_2,
    BPSPI_MESSAGE_GET_SENSOR_3,
    BPSPI_MESSAGE_GET_SENSOR_4,

    BPSPI_MESSAGE_I2C_TRANSACT_1, // 17
    BPSPI_MESSAGE_I2C_TRANSACT_2,
    BPSPI_MESSAGE_I2C_TRANSACT_3,
    BPSPI_MESSAGE_I2C_TRANSACT_4,

    BPSPI_MESSAGE_SET_MOTOR_POWER,

    BPSPI_MESSAGE_SET_MOTOR_POSITION,

    BPSPI_MESSAGE_SET_MOTOR_POSITION_KP,

    BPSPI_MESSAGE_SET_MOTOR_POSITION_KD, // 24

    BPSPI_MESSAGE_SET_MOTOR_DPS, // 25

    BPSPI_MESSAGE_SET_MOTOR_DPS_KP,

    BPSPI_MESSAGE_SET_MOTOR_DPS_KD,

    BPSPI_MESSAGE_SET_MOTOR_LIMITS,

    BPSPI_MESSAGE_OFFSET_MOTOR_ENCODER, // 29

    BPSPI_MESSAGE_GET_MOTOR_A_ENCODER, // 30
    BPSPI_MESSAGE_GET_MOTOR_B_ENCODER,
    BPSPI_MESSAGE_GET_MOTOR_C_ENCODER,
    BPSPI_MESSAGE_GET_MOTOR_D_ENCODER,

    BPSPI_MESSAGE_GET_MOTOR_A_STATUS, // 34
    BPSPI_MESSAGE_GET_MOTOR_B_STATUS,
    BPSPI_MESSAGE_GET_MOTOR_C_STATUS,
    BPSPI_MESSAGE_GET_MOTOR_D_STATUS
};

// structure for I2C
struct i2c_struct_t
{
    uint8_t speed;
    uint8_t delay;
    uint8_t address;
    uint8_t length_write;
    uint8_t buffer_write[LONGEST_I2C_TRANSFER];
    uint8_t length_read;
    uint8_t buffer_read[LONGEST_I2C_TRANSFER];
};

int spi_setup();
int spi_transfer_array(uint8_t length, uint8_t *outArray, uint8_t *inArray);
void fatal_error(char *error);
void fatal_error(const char *error);
int BrickPi3_set_address(int addr, const char *id);

#endif // BP_SPI_H