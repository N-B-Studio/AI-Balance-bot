#include "BMI088driver.h"
#include "BMI088reg.h"
#include "BMI088Middleware.h"
/**
************************************************************************
* @brief:       BMI088_read_multi_reg(uint8_t reg, uint8_t *buf, uint8_t len)
* @param:       reg - start register address
* @param:       buf - buffer to store read bytes
* @param:       len - number of bytes to read
* @retval:      void
* @details:     Read multiple consecutive registers from BMI088 over SPI
************************************************************************
**/
* @brief:       BMI088_accel_write_single_reg(reg, data)
* @param:       reg  - register address to write
* @param:       data - byte to write
* @retval:      void
* @details:     Drive SPI to write a single accelerometer register on BMI088
************************************************************************
**/
#define BMI088_accel_write_single_reg(reg, data) \
    {                                            \
        BMI088_ACCEL_NS_L();                     \
        BMI088_write_single_reg((reg), (data));  \
        BMI088_ACCEL_NS_H();                     \
    /**
    ************************************************************************
    * @brief:       BMI088_read_multi_reg(uint8_t reg, uint8_t *buf, uint8_t len)
    * @param:       reg - start register address
    * @param:       buf - buffer to store read bytes
    * @param:       len - number of bytes to read
    * @retval:      void
    * @details:     Read multiple consecutive registers from BMI088 over SPI
    ************************************************************************
    **/
#define BMI088_accel_read_single_reg(reg, data) \
    {                                           \
        BMI088_ACCEL_NS_L();                    \
        BMI088_read_write_byte((reg) | 0x80);   \
        BMI088_read_write_byte(0x55);           \
        (data) = BMI088_read_write_byte(0x55);  \
        BMI088_ACCEL_NS_H();                    \
    }
/**
************************************************************************
* @brief:       BMI088_accel_read_multi_reg(reg, data, len)
* @param:       reg  - start register address
* @param:       data - buffer to store read bytes
* @param:       len  - number of bytes to read
* @retval:      void
* @details:     Drive SPI to read multiple accelerometer registers on BMI088
************************************************************************
**/
#define BMI088_accel_read_muli_reg(reg, data, len) \
    {                                              \
        BMI088_ACCEL_NS_L();                       \
        BMI088_read_write_byte((reg) | 0x80);      \
        BMI088_read_muli_reg(reg, data, len);      \
        BMI088_ACCEL_NS_H();                       \
    }
/**
************************************************************************
* @brief:       BMI088_gyro_write_single_reg(reg, data)
* @param:       reg  - register address to write
* @param:       data - byte to write
* @retval:      void
* @details:     Drive SPI to write a single gyroscope register on BMI088
************************************************************************
**/
#define BMI088_gyro_write_single_reg(reg, data) \
    {                                           \
        BMI088_GYRO_NS_L();                     \
        BMI088_write_single_reg((reg), (data)); \
        BMI088_GYRO_NS_H();                     \
    }
/**
************************************************************************
* @brief:       BMI088_gyro_read_single_reg(reg, data)
* @param:       reg  - register address to read
* @param:       data - output byte read from register
* @retval:      void
* @details:     Drive SPI to read a single gyroscope register on BMI088
************************************************************************
**/
#define BMI088_gyro_read_single_reg(reg, data)  \
    {                                           \
        BMI088_GYRO_NS_L();                     \
        BMI088_read_single_reg((reg), &(data)); \
        BMI088_GYRO_NS_H();                     \
    }
/**
************************************************************************
* @brief:       BMI088_gyro_read_multi_reg(reg, data, len)
* @param:       reg  - start register address
* @param:       data - buffer to store read bytes
* @param:       len  - number of bytes to read
* @retval:      void
* @details:     Drive SPI to read multiple gyroscope registers on BMI088
************************************************************************
**/
#define BMI088_gyro_read_muli_reg(reg, data, len)   \
    {                                               \
        BMI088_GYRO_NS_L();                         \
        BMI088_read_muli_reg((reg), (data), (len)); \
        BMI088_GYRO_NS_H();                         \
    }

static void BMI088_write_single_reg(uint8_t reg, uint8_t data);
static void BMI088_read_single_reg(uint8_t reg, uint8_t *return_data);
//static void BMI088_write_muli_reg(uint8_t reg, uint8_t* buf, uint8_t len );
static void BMI088_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len);

#elif defined(BMI088_USE_IIC)


#endif
/**
************************************************************************
* @brief:       write_BMI088_accel_reg_data_error_init(void)
* @param:       void
* @retval:      void
* @details:     Table of accelerometer register initial values and error codes
************************************************************************
**/
static uint8_t write_BMI088_accel_reg_data_error[BMI088_WRITE_ACCEL_REG_NUM][3] =
    {
        {BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE_ACC_ON, BMI088_ACC_PWR_CTRL_ERROR},
        {BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE_MODE, BMI088_ACC_PWR_CONF_ERROR},
        {BMI088_ACC_CONF,  BMI088_ACC_NORMAL| BMI088_ACC_800_HZ | BMI088_ACC_CONF_MUST_Set, BMI088_ACC_CONF_ERROR},
        {BMI088_ACC_RANGE, BMI088_ACC_RANGE_3G, BMI088_ACC_RANGE_ERROR},
        {BMI088_INT1_IO_CTRL, BMI088_ACC_INT1_IO_ENABLE | BMI088_ACC_INT1_GPIO_PP | BMI088_ACC_INT1_GPIO_LOW, BMI088_INT1_IO_CTRL_ERROR},
        {BMI088_INT_MAP_DATA, BMI088_ACC_INT1_DRDY_INTERRUPT, BMI088_INT_MAP_DATA_ERROR}

};
/**
************************************************************************
* @brief:       write_BMI088_gyro_reg_data_error_init(void)
* @param:       void
* @retval:      void
* @details:     Table of gyroscope register initial values and error codes
************************************************************************
**/
static uint8_t write_BMI088_gyro_reg_data_error[BMI088_WRITE_GYRO_REG_NUM][3] =
    {
        {BMI088_GYRO_RANGE, BMI088_GYRO_2000, BMI088_GYRO_RANGE_ERROR},
        {BMI088_GYRO_BANDWIDTH, BMI088_GYRO_1000_116_HZ | BMI088_GYRO_BANDWIDTH_MUST_Set, BMI088_GYRO_BANDWIDTH_ERROR},
        {BMI088_GYRO_LPM1, BMI088_GYRO_NORMAL_MODE, BMI088_GYRO_LPM1_ERROR},
        {BMI088_GYRO_CTRL, BMI088_DRDY_ON, BMI088_GYRO_CTRL_ERROR},
        {BMI088_GYRO_INT3_INT4_IO_CONF, BMI088_GYRO_INT3_GPIO_PP | BMI088_GYRO_INT3_GPIO_LOW, BMI088_GYRO_INT3_INT4_IO_CONF_ERROR},
        {BMI088_GYRO_INT3_INT4_IO_MAP, BMI088_GYRO_DRDY_IO_INT3, BMI088_GYRO_INT3_INT4_IO_MAP_ERROR}

};
/**
************************************************************************
* @brief:       BMI088_init(void)
* @param:       void
* @retval:      uint8_t - error status (BMI088_NO_ERROR on success)
* @details:     Initialize GPIO/SPI and initialize both accelerometer and gyro
************************************************************************
**/
uint8_t BMI088_init(void)
{
    uint8_t error = BMI088_NO_ERROR;
    // GPIO and SPI  Init .
    BMI088_GPIO_init();
    BMI088_com_init();

    error |= bmi088_accel_init();
    error |= bmi088_gyro_init();

    return error;
}
/**
************************************************************************
* @brief:       bmi088_accel_init(void)
* @param:       void
* @retval:      uint8_t - error status
* @details:     Initialize BMI088 accelerometer: configure registers and verify
************************************************************************
**/
uint8_t bmi088_accel_init(void)
{
    uint8_t res = 0;
    uint8_t write_reg_num = 0;

    //check commiunication
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    //accel software reset
    BMI088_accel_write_single_reg(BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE);
    BMI088_delay_ms(BMI088_LONG_DELAY_TIME);

    //check commiunication is normal after reset
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    // check the "who am I"
    if (res != BMI088_ACC_CHIP_ID_VALUE)
    {
        return BMI088_NO_SENSOR;
    }

    //set accel sonsor config and check
    for (write_reg_num = 0; write_reg_num < BMI088_WRITE_ACCEL_REG_NUM; write_reg_num++)
    {

        BMI088_accel_write_single_reg(write_BMI088_accel_reg_data_error[write_reg_num][0], write_BMI088_accel_reg_data_error[write_reg_num][1]);
        BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        BMI088_accel_read_single_reg(write_BMI088_accel_reg_data_error[write_reg_num][0], res);
        BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        if (res != write_BMI088_accel_reg_data_error[write_reg_num][1])
        {
            return write_BMI088_accel_reg_data_error[write_reg_num][2];
        }
    }
    return BMI088_NO_ERROR;
}
/**
************************************************************************
* @brief:       bmi088_gyro_init(void)
* @param:       void
* @retval:      uint8_t - error status
* @details:     Initialize BMI088 gyroscope: configure registers and verify
************************************************************************
**/
uint8_t bmi088_gyro_init(void)
{
    uint8_t write_reg_num = 0;
    uint8_t res = 0;

    //check commiunication
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    //reset the gyro sensor
    BMI088_gyro_write_single_reg(BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE);
    BMI088_delay_ms(BMI088_LONG_DELAY_TIME);
    //check commiunication is normal after reset
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);
    BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, res);
    BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

    // check the "who am I"
    if (res != BMI088_GYRO_CHIP_ID_VALUE)
    {
        return BMI088_NO_SENSOR;
    }

    //set gyro sonsor config and check
    for (write_reg_num = 0; write_reg_num < BMI088_WRITE_GYRO_REG_NUM; write_reg_num++)
    {

        BMI088_gyro_write_single_reg(write_BMI088_gyro_reg_data_error[write_reg_num][0], write_BMI088_gyro_reg_data_error[write_reg_num][1]);
        BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        BMI088_gyro_read_single_reg(write_BMI088_gyro_reg_data_error[write_reg_num][0], res);
        BMI088_delay_us(BMI088_COM_WAIT_SENSOR_TIME);

        if (res != write_BMI088_gyro_reg_data_error[write_reg_num][1])
        {
            return write_BMI088_gyro_reg_data_error[write_reg_num][2];
        }
    }

    return BMI088_NO_ERROR;
}
/**
************************************************************************
* @brief:      	BMI088_read(float gyro[3], float accel[3], float *temperate)
* @param:       gyro - �������������� (x, y, z)
* @param:       accel - ���ٶȼ��������� (x, y, z)
* @param:       temperate - �¶�����ָ��
* @retval:     	void
* @details:    	��ȡBMI088���������ݣ��������ٶȡ������Ǻ��¶�
************************************************************************
**/
void BMI088_read(float gyro[3], float accel[3], float *temperate)
{
    uint8_t buf[8] = {0, 0, 0, 0, 0, 0};
    int16_t bmi088_raw_temp;

    BMI088_accel_read_muli_reg(BMI088_ACCEL_XOUT_L, buf, 6);

    bmi088_raw_temp = (int16_t)((buf[1]) << 8) | buf[0];
    accel[0] = bmi088_raw_temp * BMI088_ACCEL_SEN;
    bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
    accel[1] = bmi088_raw_temp * BMI088_ACCEL_SEN;
    bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
    accel[2] = bmi088_raw_temp * BMI088_ACCEL_SEN;

    BMI088_gyro_read_muli_reg(BMI088_GYRO_CHIP_ID, buf, 8);
    if(buf[0] == BMI088_GYRO_CHIP_ID_VALUE)
    {
        bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
        gyro[0] = bmi088_raw_temp * BMI088_GYRO_SEN;
        bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
        gyro[1] = bmi088_raw_temp * BMI088_GYRO_SEN;
        bmi088_raw_temp = (int16_t)((buf[7]) << 8) | buf[6];
        gyro[2] = bmi088_raw_temp * BMI088_GYRO_SEN;
    }
    BMI088_accel_read_muli_reg(BMI088_TEMP_M, buf, 2);

    bmi088_raw_temp = (int16_t)((buf[0] << 3) | (buf[1] >> 5));

    if (bmi088_raw_temp > 1023)
    {
        bmi088_raw_temp -= 2048;
    }

    *temperate = bmi088_raw_temp * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;
}

#if defined(BMI088_USE_SPI)
/**
************************************************************************
* @brief:      	BMI088_write_single_reg(uint8_t reg, uint8_t data)
* @param:       reg - �Ĵ�����ַ
* @param:       data - д�������
* @retval:     	void
* @details:    	��BMI088������д�뵥���Ĵ���������
************************************************************************
**/
static void BMI088_write_single_reg(uint8_t reg, uint8_t data)
{
    BMI088_read_write_byte(reg);
    BMI088_read_write_byte(data);
}
/**
************************************************************************
* @brief:       BMI088_read_single_reg(uint8_t reg, uint8_t *return_data)
* @param:       reg         - register address to read
* @param:       return_data - pointer to store the returned byte
* @retval:      void
* @details:     Read a single register from BMI088 over SPI
************************************************************************
**/
static void BMI088_read_single_reg(uint8_t reg, uint8_t *return_data)
/**
************************************************************************
* @brief:       BMI088_read_multi_reg(uint8_t reg, uint8_t *buf, uint8_t len)
* @param:       reg - start register address
* @param:       buf - buffer to store read bytes
* @param:       len - number of bytes to read
* @retval:      void
* @details:     Read multiple consecutive registers from BMI088 over SPI
************************************************************************
**/

//        BMI088_read_write_byte( *buf );
//        buf ++;
//        len --;
//    }

//}
/**
************************************************************************
* @brief:      	BMI088_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len)
* @param:       reg - ��ʼ�Ĵ�����ַ
*               buf - �洢��ȡ���ݵĻ�����
*               len - Ҫ��ȡ���ֽ���
* @retval:     	void
* @details:    	��BMI088������������ȡ����Ĵ���������
************************************************************************
**/
static void BMI088_read_muli_reg(uint8_t reg, uint8_t *buf, uint8_t len)
{
    BMI088_read_write_byte(reg | 0x80);

    while (len != 0)
    {

        *buf = BMI088_read_write_byte(0x55);
        buf++;
        len--;
    }
}
#elif defined(BMI088_USE_IIC)

#endif
