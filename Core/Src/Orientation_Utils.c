#include "Orientation_Utils.h"

#define ACCEL_SCALE 16384.0
#define GYRO_SCALE  131.0

float gyro_roll = 0.0;
float gyro_pitch = 0.0;

void get_accel_angles(ring_buffer_s* pRingBuffer,float* accel_pitch, float* accel_roll)
{
    float accel_x = ((float) pRingBuffer->buffer[0][pRingBuffer->write_index-1] ) / ACCEL_SCALE;
    float accel_y = ((float) pRingBuffer->buffer[1][pRingBuffer->write_index-1] ) / ACCEL_SCALE;
    float accel_z = ((float) pRingBuffer->buffer[2][pRingBuffer->write_index-1] ) / ACCEL_SCALE;

    //*accel_roll  = RAD_TO_DEG(atan2(accel_x,accel_z));
    //*accel_pitch = RAD_TO_DEG(atan2(accel_y,accel_z));

    float sign = accel_z > 0 ? 1.0 : -1.0;

    *accel_roll = RAD_TO_DEG(atan2f((accel_x) , sign*(sqrt(accel_y*accel_y + accel_z*accel_z))));
    *accel_pitch = RAD_TO_DEG(atan2f((accel_y) , (sqrt(accel_x*accel_x + accel_z*accel_z))));
}

float compfilter_tick(ring_buffer_s* pRingBuffer,float* comp_pitch, float* comp_roll)
{
    float accel_x = ((float) pRingBuffer->buffer[0][pRingBuffer->write_index-1] ) / ACCEL_SCALE;
    float accel_y = ((float) pRingBuffer->buffer[1][pRingBuffer->write_index-1] ) / ACCEL_SCALE;
    float accel_z = ((float) pRingBuffer->buffer[2][pRingBuffer->write_index-1] ) / ACCEL_SCALE;

    float gyro_x = ((float) pRingBuffer->buffer[3][pRingBuffer->write_index-1] ) / GYRO_SCALE;
    float gyro_y = ((float) pRingBuffer->buffer[4][pRingBuffer->write_index-1] ) / GYRO_SCALE;
    float gyro_z = ((float) pRingBuffer->buffer[5][pRingBuffer->write_index-1] ) / GYRO_SCALE;

    float accel_roll  = RAD_TO_DEG(atan2(accel_x,accel_z));
    float accel_pitch = RAD_TO_DEG(atan2(accel_y,accel_z));
    get_accel_angles(pRingBuffer,&accel_pitch,&accel_roll);

    *comp_roll = ALPHA * (gyro_roll - gyro_y * PERIOD) + (1-ALPHA)* accel_roll;
    *comp_pitch = ALPHA * (gyro_pitch - gyro_x * PERIOD) + (1-ALPHA)* accel_pitch;

}