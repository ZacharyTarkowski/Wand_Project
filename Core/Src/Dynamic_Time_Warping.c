#include "Dynamic_Time_Warping.h"

u32 DTW_Min(u32 dtw_buf[MPU_6050_NUM_DIMS][0][0], int k, int i, int j)
{
    u32 test1 = dtw_buf[k][i-1][j];
    u32 test2 = dtw_buf[k][i][j-1];
    u32 test3 = dtw_buf[k][i-1][j-1];

    //return test1 + test2 + test3;

    return MIN( MIN(test1,test2),test3 );
}

void populate_dtw_result(DTW_Result* pStruct, u32 buf[MPU_6050_NUM_DIMS][0][0], u32 idx1, u32 idx2)
{
    pStruct->x_accel_result = buf[X_ACCEL_IDX][idx1][idx2] ;
    pStruct->y_accel_result = buf[Y_ACCEL_IDX][idx1][idx2] ;
    pStruct->z_accel_result = buf[Z_ACCEL_IDX][idx1][idx2] ;
    pStruct->x_gyro_result = buf[X_GYRO_IDX ][idx1][idx2] ;
    pStruct->y_gyro_result = buf[Y_GYRO_IDX ][idx1][idx2] ;
    pStruct->z_gyro_result = buf[Z_GYRO_IDX ][idx1][idx2] ;
}

DTW_Result DTW_Distance( buffer_element* s, buffer_element* t)
{
    u32 s_length = write_index[0];
    u32 t_length = write_index[1];

    u32 dtw_buf[MPU_6050_NUM_DIMS][s_length][t_length];
    memset(dtw_buf, MAX_INT, MPU_6050_NUM_DIMS * s_length * t_length * sizeof(u32));

    
    
    for(int k = 0; k< MPU_6050_NUM_DIMS; k++)
    {
        

        for(int i = 0; i< s_length; i++)
        {
            for(int j = 0; j< t_length; j++)
            {
                dtw_buf[k][i][j] = 0xFFFFFFFF;
            }

        }
        dtw_buf[k][0][0] = 0;
    }

    

    //FIXME hardcoded to be ring buf 0 to ring buf 1
    for(int i = 1; i< s_length; i++)
    {
        for(int j = 1; j< t_length; j++)
        {
            //fixme find a clever way to do this
            //need to advance linked list if I end up doing linked list ring buffer
            //should probably use ring buffer read element here
            dtw_buf[X_ACCEL_IDX][i][j] = abs(s[i].x_accel_data - t[j].x_accel_data ) + DTW_Min(dtw_buf,X_ACCEL_IDX,i,j);
            dtw_buf[Y_ACCEL_IDX][i][j] = abs(s[i].y_accel_data - t[j].y_accel_data ) + DTW_Min(dtw_buf,Y_ACCEL_IDX,i,j);
            dtw_buf[Z_ACCEL_IDX][i][j] = abs(s[i].z_accel_data - t[j].z_accel_data ) + DTW_Min(dtw_buf,Z_ACCEL_IDX,i,j);
            dtw_buf[X_GYRO_IDX ][i][j] = abs(s[i].x_gyro_data - t[j].x_gyro_data  )  + DTW_Min(dtw_buf,X_GYRO_IDX ,i,j);
            dtw_buf[Y_GYRO_IDX ][i][j] = abs(s[i].y_gyro_data - t[j].y_gyro_data  )  + DTW_Min(dtw_buf,Y_GYRO_IDX ,i,j);
            dtw_buf[Z_GYRO_IDX ][i][j] = abs(s[i].z_gyro_data - t[j].z_gyro_data  )  + DTW_Min(dtw_buf,Z_GYRO_IDX ,i,j);
        }
    }

    DTW_Result result;
    //populate_dtw_result(&result, dtw_buf, s_length-1, t_length-1);

    result.x_accel_result = dtw_buf[X_ACCEL_IDX][s_length-1][t_length-1] ;
    result.y_accel_result = dtw_buf[Y_ACCEL_IDX][s_length-1][t_length-1] ;
    result.z_accel_result = dtw_buf[Z_ACCEL_IDX][s_length-1][t_length-1] ;
    result.x_gyro_result  = dtw_buf[X_GYRO_IDX ][s_length-1][t_length-1] ;
    result.y_gyro_result  = dtw_buf[Y_GYRO_IDX ][s_length-1][t_length-1] ;
    result.z_gyro_result  = dtw_buf[Z_GYRO_IDX ][s_length-1][t_length-1] ;

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