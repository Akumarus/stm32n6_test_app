#include "i2c.hpp"

void I2c::Init()
{
    MX_I2C1_Init();
} 
bool I2c::Write(uint16_t addr, uint8_t* data, uint16_t size)
{
    auto res = HAL_I2C_Master_Transmit(&i2c, addr, data, size, timeout);
    return res == HAL_OK ? true : false;
}

bool I2c::Read(uint16_t addr, uint8_t* data, uint16_t size)
{
    auto res = HAL_I2C_Master_Receive(&i2c, addr, data, size, timeout);
    return res == HAL_OK ? true : false;
}

bool I2c::WriteMem(uint16_t addr, uint16_t reg, uint8_t* data, uint16_t size)
{
    auto res = HAL_I2C_Mem_Write(&i2c, addr, reg, I2C_MEMADD_SIZE_16BIT, data, size, timeout);
    return res == HAL_OK ? true : false;
}
bool I2c::ReadMem(uint16_t addr, uint16_t reg, uint8_t* data, uint16_t size)
{
    auto res = HAL_I2C_Mem_Read(&i2c, addr, reg, I2C_MEMADD_SIZE_16BIT, data, size, timeout);
    return res == HAL_OK ? true : false;
}

void I2c::SetTimeout(uint32_t time)
{
    timeout = time;
}