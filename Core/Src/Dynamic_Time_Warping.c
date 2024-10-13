#include "Dynamic_Time_Warping.h"

u32** single_dtw_buf;
u32*** multi_dtw_buf;

HAL_StatusTypeDef dtw_init(DTW_MODE mode)
{
    HAL_StatusTypeDef status = HAL_OK;

    if(mode == SINGLE_MODE)
    {
        single_dtw_buf = malloc(RING_BUFFER_MAX_SIZE * sizeof(u32*));

        if(single_dtw_buf == 0)
        {
            status = HAL_ERROR;
        }

        for(u32 i = 0; i< RING_BUFFER_MAX_SIZE; i++)
        {
            if(status == HAL_OK)
            {
                single_dtw_buf[i] = malloc(RING_BUFFER_MAX_SIZE * sizeof(u32));

                if(single_dtw_buf[i] == 0)
                {
                    status = HAL_ERROR;
                }
            }
        }

    }
    else if(mode == MULTI_MODE)
    {
        multi_dtw_buf = malloc(MPU_6050_NUM_DIMS * sizeof(u32**));

        if(multi_dtw_buf == 0)
        {
            status = HAL_ERROR;
        }

        for(u32 i = 0; i < MPU_6050_NUM_DIMS; i++)
        {
            if(status == HAL_OK)
            {
                multi_dtw_buf[i] = malloc(RING_BUFFER_MAX_SIZE * sizeof(u32*));

                if(multi_dtw_buf[i] == 0)
                {
                    status = HAL_ERROR;
                }

                for(u32 j = 0; j< RING_BUFFER_MAX_SIZE; j++)
                {
                    if(status == HAL_OK)
                    {
                        multi_dtw_buf[i][j] = malloc(RING_BUFFER_MAX_SIZE * sizeof(u32));

                        if(multi_dtw_buf[i][j] == 0)
                        {
                            status = HAL_ERROR;
                        }
                    }
                }
            }
        }
    }
    else
    {
        status = HAL_ERROR;
    }
    
    if(status != HAL_OK)
    {
        u32 requested_bytes = (mode == MULTI_MODE ? MPU_6050_NUM_DIMS : 1 ) * RING_BUFFER_MAX_SIZE * RING_BUFFER_MAX_SIZE; 
        uart_println("Problem with DTW Init, attempting to allocate %d bytes for mode %s", requested_bytes, mode == MULTI_MODE ? "MULTI" : "SINGLE");
    }

    return status;
}

void free_dtw_buf(DTW_MODE mode)
{
    if(mode == SINGLE_MODE)
    {
        for(u32 i = 0; i< RING_BUFFER_MAX_SIZE; i++)
        {
            free(single_dtw_buf[i]);
        }

        free(single_dtw_buf);
    }
    else if(mode == MULTI_MODE)
    {
        for(u32 i = 0; i< RING_BUFFER_MAX_SIZE; i++)
        {
            for(u32 j = 0; j< RING_BUFFER_MAX_SIZE; j++)
            {
                free(multi_dtw_buf[i][j]);
            }

            free(multi_dtw_buf[i]);
        }

        free(multi_dtw_buf);
    }

}

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
//     memset(dtw_buf, 0x0, RING_BUFFER_MAX_SIZE * RING_BUFFER_MAX_SIZE * sizeof(u32));
// }

u32 dtw_single(buffer_element *s, buffer_element *t, u32 n, u32 m, MPU6050_dimension sel) {
    
    u32 i, j;

    for(i = 0; i <= n; i++) {
        for(j = 0; j <= m; j++) {
            if (i == 0 && j == 0) {
                single_dtw_buf[i][j] = 0;
            } else if (i == 0) {
                single_dtw_buf[i][j] = 0xFFFFFFFF;
            } else if (j == 0) {
                single_dtw_buf[i][j] = 0xFFFFFFFF;
            } else {
                u32 cost = abs(ring_buffer_dimension_sel(&s[i - 1], sel) - ring_buffer_dimension_sel(&t[j - 1],sel));
                single_dtw_buf[i][j] = cost + MIN(MIN(single_dtw_buf[i - 1][j], single_dtw_buf[i][j - 1]), single_dtw_buf[i - 1][j - 1]);
            }
        }
    }

    u32 distance = single_dtw_buf[n][m];

    return distance;
}

//stub
u32 dtw_multi()
{
    return 0;
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
    uart_println("X GYRO DTW %d",  data->x_gyro_result );
    uart_println("Y GYRO DTW %d",  data->y_gyro_result );
    uart_println("Z GYRO DTW %d",  data->z_gyro_result );
}

