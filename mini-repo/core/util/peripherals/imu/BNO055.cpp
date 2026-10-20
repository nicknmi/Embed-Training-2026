#include "BNO055.h"
#include "thisThread.cpp"


/*
We added these two functions since they're definitely beyond what we expect from you as recruits
I'd be impressed if you understood the math behind them.
Basically just know that quats are 4 axis (1 real, 3 imaginary) numbers that allow us
to nicely talk about rotation, and the functions spit out pitch roll and yaw.
*/
void BNO055::get_quaternion(BNO055_QUATERNION_TypeDef *result)
{
    if (cantReadDataCount > 0 && cantReadDataCount < 50) {
        cantReadDataCount++;
        return;
    } else if (cantReadDataCount >= 50) {
        cantReadDataCount = 1;
    }
    int16_t w,x,y,z;

    dt[0] = BNO055_QUATERNION_W_LSB;
    int writeResult = _i2c.write(chip_addr, dt, 1, true);
    if (!writeResult)  {
        if (cantReadDataCount > 0) {
            printf("RESET IMU\n");
            reset();
            cantReadDataCount = 0;
        }
        _i2c.read(chip_addr, dt, 8, false);
        w = dt[1] << 8 | dt[0];
        x = dt[3] << 8 | dt[2];
        y = dt[5] << 8 | dt[4];
        z = dt[7] << 8 | dt[6];

        result->w = (double)w / 16384.0f;
        result->x = (double)x / 16384.0f;
        result->y = (double)y / 16384.0f;
        result->z = (double)z / 16384.0f;
    } else {
        cantReadDataCount++;
    }
}

void BNO055::get_angular_position_quat(IMU::EulerAngles *result){

    BNO055_QUATERNION_TypeDef q;
    get_quaternion(&q);

    float roll  = atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y)) * 180 / PI;
    float pitch = asin(2 * q.w * q.y - q.x * q.z) * 180 / PI;
    float yaw   = atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)) * 180 / PI;

    memcpy(&result->roll, &roll, sizeof(float));
    memcpy(&result->pitch, &pitch, sizeof(float));
    memcpy(&result->yaw, &yaw, sizeof(float));
}


BNO055::BNO055(I2C &i2c, uint8_t addr, PinName p_reset=NC) noexcept
    : _i2c_p(&i2c), _i2c(i2c), chip_addr(addr), _res(p_reset), cantReadDataCount()
{}

void BNO055::init() noexcept {
    _i2c.frequency(100);

    dt[0] = PWR_MODE;
    int writeResult = _i2c.write(chip_addr, dt, 1);

    // TODO: handle writing fail
    if (!writeResult) return;

    ThisThread::sleep_for(400); // 400ms from start-up to config mode

    dt[0] = OPR_MODE;
    dt[1] = 0b0111;
    writeResult = _i2c.write(chip_addr, dt, dt[1], 1); // set AMG mode (enables accel, mag, gyro (non fusion))
    if (!writeResult) return;
}


void BNO055::get_accel(BNO055_VECTOR_TypeDef *la) {
    if (cantReadDataCount > 0 && cantReadDataCount < 50) {
        cantReadDataCount++;
        return;
    } else if (cantReadDataCount >= 50) {
        cantReadDataCount = 1;
    }

    int16_t x, y, z;

    dt[0] = BNO055_LINACCEL_X_LSB;
    int writeResult = _i2c.write(chip_addr, dt, 1, true);

    if (!writeResult) {
        if (cantReadDataCount > 0) {
            printf("RESET IMU\n");
            reset();
            cantReadDataCount = 0;
        }
        // todo: figure out accelerometer offset(?)
        _i2c.read(chip_addr, dt, 6);

        x = dt[1] << 8 | dt[0]; // dt[0] is lin accel lsb, dt[1] is lin accel msb, etc
        y = dt[3] << 8 | dt[2];
        z = dt[5] << 8 | dt[4];

        la->x = (double) x;
        la->y = (double) y;
        la->z = (double) z;
    } else {
        cantReadDataCount++;
    }
}

void BNO055::get_gyro(BNO055_VECTOR_TypeDef *gr) {
    if (cantReadDataCount > 0 && cantReadDataCount < 50) {
        cantReadDataCount++;
        return;
    } else if (cantReadDataCount >= 50) {
        cantReadDataCount = 1;
    }

    int16_t x, y, z;

    dt[0] = BNO055_GYRO_X_LSB;
    int writeResult = _i2c.write(chip_addr, dt, 1, true);

    if (!writeResult) {
        if (cantReadDataCount > 0) {
            printf("RESET IMU\n");
            reset();
            cantReadDataCount = 0;
        }
        // todo: figure out gyro offset(?)
        _i2c.read(chip_addr, dt, 6);

        x = dt[1] << 8 | dt[0];
        y = dt[3] << 8 | dt[2];
        z = dt[5] << 8 | dt[4];

        gr->x = (double) x;
        gr->y = (double) y;
        gr->z = (double) z;
    } else {
        cantReadDataCount++;
    }
}
