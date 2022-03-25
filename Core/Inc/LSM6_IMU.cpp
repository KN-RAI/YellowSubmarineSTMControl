#include <LSM6_IMU.h>
#include "main.h"
#include <math.h>

// Defines ////////////////////////////////////////////////////////////////

// The Arduino two-wire interface uses a 7-bit number for the address,
// and sets the last bit correctly based on reads and writes
#define DS33_SA0_HIGH_ADDRESS 0b1101011
#define DS33_SA0_LOW_ADDRESS  0b1101010

#define DS33_WHO_ID    0x69

// Constructors ////////////////////////////////////////////////////////////////

LSM6::LSM6(void)
{
  _device = device_auto;

  io_timeout = 0;  // 0 = no timeout
  did_timeout = false;
}

// Public Methods //////////////////////////////////////////////////////////////

// Did a timeout occur in readAcc(), readGyro(), or read() since the last call to timeoutOccurred()?
bool LSM6::timeoutOccurred()
{
  bool tmp = did_timeout;
  did_timeout = false;
  return tmp;
}

void LSM6::setTimeout(uint16_t timeout)
{
  io_timeout = timeout;
}

uint16_t LSM6::getTimeout()
{
  return io_timeout;
}

bool LSM6::init(I2C_HandleTypeDef _hi2c,deviceType device, sa0State sa0)
{
	this->hi2c=_hi2c;
	device = device_DS33;

    // make sure device and SA0 were successfully detected; otherwise, indicate failure
    if (device == device_auto || sa0 == sa0_auto)
    {
      return false;
    }

  _device = device;

  if(device==device_DS33)
  {
      address = (sa0 == sa0_high) ? DS33_SA0_HIGH_ADDRESS : DS33_SA0_LOW_ADDRESS;
      address_shl=address<<1;
  }

  enableDefault();

  return true;
}

void LSM6::enableDefault(void)
{
  if (_device == device_DS33)
  {
    // Accelerometer

    // 0x80 = 0b10000000
    // ODR = 1000 (1.66 kHz (high performance)); FS_XL = 00 (+/-2 g full scale)
    writeReg(CTRL1_XL, 0x80);
    // Gyro

    // 0x80 = 0b010000000
    // ODR = 1000 (1.66 kHz (high performance)); FS_XL = 00 (245 dps)
    writeReg(CTRL2_G, 0x80);
    // Common

    // 0x04 = 0b00000100
    // IF_INC = 1 (automatically increment register address)
    writeReg(CTRL3_C, 0x04);
  }
}

void LSM6::writeReg(uint8_t reg, uint8_t value)
{
	uint8_t command=value;
	HAL_I2C_Mem_Write(&this->hi2c,address_shl, reg, 1, &command, sizeof(command), 100);

}

uint8_t LSM6::readReg(uint8_t reg)
{
	uint8_t value = 0;
	HAL_StatusTypeDef status=HAL_I2C_Mem_Read(&this->hi2c, address_shl, reg, 1, &value, sizeof(value), 200);
	return (status==HAL_OK)?value:(-1);
}


// Reads the 3 accelerometer channels and stores them in vector a
void LSM6::readAcc(void)
{
	uint8_t data[6];

	data[0]=readReg(OUTX_H_XL) ;
	data[1]=readReg(OUTX_L_XL);

	data[2]=readReg(OUTY_H_XL);
	data[3]=readReg(OUTY_L_XL);

	data[4]=readReg(OUTZ_H_XL);
	data[5]=readReg(OUTZ_L_XL);

	a.x  = (int16_t)((data[0] << 8) | data[1]);
	a.y = (int16_t)((data[2] << 8) | data[3]);
	a.z = (int16_t)((data[4] << 8) | data[5]);
}

// Reads the 3 gyro channels and stores them in vector g
void LSM6::readGyro(void)
{
	uint8_t data[6];

	data[0]=readReg(OUTX_H_G);
	data[1]=readReg(OUTX_L_G);

	data[2]=readReg(OUTY_H_G);
	data[3]=readReg(OUTY_L_G);

	data[4]=readReg(OUTZ_H_G);
	data[5]=readReg(OUTZ_L_G);

	g.x  = (int16_t)((data[0] << 8) | data[1]);
	g.y = (int16_t)((data[2] << 8) | data[3]);
	g.z = (int16_t)((data[4] << 8) | data[5]);

}

// Reads all 6 channels of the LSM6 and stores them in the object variables
void LSM6::read(void)
{
  readAcc();
  readGyro();
  scaleVectors();
}

void LSM6::vector_normalize(vector<float> *a)
{
  float mag = sqrt(vector_dot(a, a));
  a->x /= mag;
  a->y /= mag;
  a->z /= mag;
}

void LSM6::scaleVectors()
{
	   float a_x = this->a.x*2.0f/32678.0f;
	   float a_y = this->a.y*2.0f/32678.0f;
	   float a_z = this->a.z*2.0f/32678.0f;
	   float alpha=atan2f(a_z,a_x)* 180.0f / 3.14 + 90.0f;
	   float gamma=atan2f(a_z,a_y)* 180.0f / 3.14 + 90.0f;
	   float delta=atan2f(a_y,a_x)* 180.0f / 3.14 + 90.0f;

	   this->a_scaled.x=alpha;
	   this->a_scaled.y=gamma;
	   this->a_scaled.z=delta;

	   this->g_scaled.x = this->g.x*125.0f/32678.0f;
	   this->g_scaled.y = this->g.y*125.0f/32678.0f;
	   this->g_scaled.z = this->g.z*125.0f/32678.0f;
}
