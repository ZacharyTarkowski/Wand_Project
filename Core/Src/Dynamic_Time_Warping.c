#include "Dynamic_Time_Warping.h"

u32 dtw_buf[RING_BUFFER_SIZE][RING_BUFFER_SIZE];

u32 ring_buffer_dimension_sel(buffer_element *data, MPU6050_dimension sel)
{
    switch(sel)
    {
        case X_ACCEL_IDX:
            return data->x_accel_data;
        break;

        case Y_ACCEL_IDX:
            return data->y_accel_data;
        break;

        case Z_ACCEL_IDX:
            return data->z_accel_data;
        break;

        case X_GYRO_IDX:
            return data->x_gyro_data;
        break;

        case Y_GYRO_IDX:
            return data->y_gyro_data;
        break;

        case Z_GYRO_IDX:
            return data->z_gyro_data;
        break;

        default:
            uart_println("Bad Ring Buffer Dimension Select");
        break;
    }

    return -1;
}

// void clear_dtw_buffer()
// {
//     memset(dtw_buf, 0x0, RING_BUFFER_SIZE * RING_BUFFER_SIZE * sizeof(u32));
// }

u32 dtw_single(buffer_element *s, buffer_element *t, u32 n, u32 m, MPU6050_dimension sel) {
    
    u32 i, j;

    for(i = 0; i <= n; i++) {
        for(j = 0; j <= m; j++) {
            if (i == 0 && j == 0) {
                dtw_buf[i][j] = 0;
            } else if (i == 0) {
                dtw_buf[i][j] = 0xFFFFFFFF;
            } else if (j == 0) {
                dtw_buf[i][j] = 0xFFFFFFFF;
            } else {
                u32 cost = abs(ring_buffer_dimension_sel(&s[i - 1], sel) - ring_buffer_dimension_sel(&t[j - 1],sel));
                dtw_buf[i][j] = cost + MIN(MIN(dtw_buf[i - 1][j], dtw_buf[i][j - 1]), dtw_buf[i - 1][j - 1]);
            }
        }
    }

    u32 distance = dtw_buf[n][m];

    return distance;
}

DTW_Result DTW_Distance( buffer_element* s, buffer_element* t, u32 s_length, u32 t_length)
{
    DTW_Result result;

    result.x_accel_result = dtw_single(s,t,s_length,t_length, X_ACCEL_IDX) ;
    result.y_accel_result = dtw_single(s,t,s_length,t_length, Y_ACCEL_IDX) ;
    result.z_accel_result = dtw_single(s,t,s_length,t_length, Z_ACCEL_IDX) ;
    result.x_gyro_result  = dtw_single(s,t,s_length,t_length, X_GYRO_IDX ) ;
    result.y_gyro_result  = dtw_single(s,t,s_length,t_length, Y_GYRO_IDX ) ;
    result.z_gyro_result  = dtw_single(s,t,s_length,t_length, Z_GYRO_IDX ) ;

    return result;
}

void print_dtw_result(DTW_Result* data)
{
    uart_println("X ACCEL DTW %d", data->x_accel_result);
    uart_println("Y ACCEL DTW %d", data->y_accel_result);
    uart_println("Z ACCEL DTW %d", data->z_accel_result);
    uart_println("X GYRO DTW %d", data->x_gyro_result);
    uart_println("Y GYRO DTW %d", data->y_gyro_result);
    uart_println("Z GYRO DTW %d", data->z_gyro_result);
}

