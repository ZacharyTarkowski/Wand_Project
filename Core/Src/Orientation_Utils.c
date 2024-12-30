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

    float accel_roll  ;
    float accel_pitch ;
    get_accel_angles(pRingBuffer,&accel_pitch,&accel_roll);

    *comp_roll = ALPHA * RAD_TO_DEG(gyro_roll - gyro_y * PERIOD) + (1-ALPHA)* accel_roll;
    *comp_pitch = ALPHA * RAD_TO_DEG(gyro_pitch - gyro_x * PERIOD) + (1-ALPHA)* accel_pitch;

}

//theta is pitch angle
//phi is roll angle
void ring_buffer_pitch_roll_rotation(ring_buffer_s* pRingBuffer, float theta, float phi)
{
/*
    Rotation Matrix
    cos(theta)  0  sin(theta)
        0       1       0
    -sin(theta) 0  cos(theta)
*/

    //need to be negative to reverse the rotation
    float theta_rad = -1*DEG_TO_RAD(theta);
    float phi_rad = -1*DEG_TO_RAD(phi);

    float coeff_00 = cos(phi_rad);               float coeff_01 = 0;          float coeff_02= sin(phi_rad);
    float coeff_10 = sin(theta_rad)*sin(phi_rad);    float coeff_11 = cos(theta_rad); float coeff_12= -1*sin(theta_rad)*cos(phi_rad);
    float coeff_20 = -1*cos(theta_rad)*sin(phi_rad); float coeff_21 = sin(theta_rad); float coeff_22= cos(theta_rad)*cos(phi_rad);

    

    for(u32 i = 0; i<pRingBuffer->write_index; i++)
    {
        float result_i0 = coeff_00 * pRingBuffer->buffer[0][i] + coeff_01 * pRingBuffer->buffer[1][i] + coeff_02 * pRingBuffer->buffer[2][i];
        float result_i1 = coeff_10 * pRingBuffer->buffer[0][i] + coeff_11 * pRingBuffer->buffer[1][i] + coeff_12 * pRingBuffer->buffer[2][i];
        float result_i2 = coeff_20 * pRingBuffer->buffer[0][i] + coeff_21 * pRingBuffer->buffer[1][i] + coeff_22 * pRingBuffer->buffer[2][i];

        pRingBuffer->buffer[0][i] = (buffer_element)result_i0;
        pRingBuffer->buffer[1][i] = (buffer_element)result_i1;
        pRingBuffer->buffer[2][i] = (buffer_element)result_i2;
    }


}