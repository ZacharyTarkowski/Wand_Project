#include "Dynamic_Time_Warping.h"

u32 DTW_Min(u32 dtw_buf[MPU_6050_NUM_DIMS][0][0], int k, int i, int j)
{
    u32 test1 = dtw_buf[k][i-1][j];
    u32 test2 = dtw_buf[k][i][j-1];
    u32 test3 = dtw_buf[k][i-1][j-1];

    //return test1 + test2 + test3;

    return MIN( MIN(test1,test2),test3 );
}

u32 DTW_Distance( buffer_element* s, buffer_element* t)
{
    u32 dtw_buf[MPU_6050_NUM_DIMS][write_index[0]][write_index[1]];
    memset(dtw_buf, MAX_INT, MPU_6050_NUM_DIMS * write_index[0] * write_index[1] * sizeof(u32));
    
    for(int i = 0; i< MPU_6050_NUM_DIMS; i++)
    {
        dtw_buf[i][0][0] = 0;
    }

    //FIXME hardcoded to be ring buf 0 to ring buf 1
    for(int i = 1; i< write_index[0]; i++)
    {
        for(int j = 1; j< write_index[1]; j++)
        {
            //fixme find a clever way to do this
            //need to advance linked list if I end up doing linked list ring buffer
            //should probably use ring buffer read element here
            dtw_buf[0][i][j] = abs(s[i].x_accel_data - t[j].x_accel_data );
            dtw_buf[0][i][j] =  dtw_buf[0][i][j] + DTW_Min(dtw_buf,0,i,j);
            // dtw_buf[1][i][j] = abs(s[i].y_accel_data - t[j].y_accel_data ) + DTW_Min(dtw_buf,1,i,j);
            // dtw_buf[2][i][j] = abs(s[i].z_accel_data - t[j].z_accel_data ) + DTW_Min(dtw_buf,2,i,j);
            // dtw_buf[3][i][j] = abs(s[i].x_gyro_data - t[j].x_gyro_data  ) + DTW_Min(dtw_buf,3,i,j);
            // dtw_buf[4][i][j] = abs(s[i].y_gyro_data - t[j].y_gyro_data  ) + DTW_Min(dtw_buf,4,i,j);
            // dtw_buf[5][i][j] = abs(s[i].z_gyro_data - t[j].z_gyro_data  ) + DTW_Min(dtw_buf,5,i,j);
        }
    }

    //FIXME
    return dtw_buf[0][write_index[0]][write_index[1]];
}