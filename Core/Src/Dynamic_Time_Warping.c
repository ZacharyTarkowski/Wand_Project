#include "Dynamic_Time_Warping.h"

#ifdef ENABLE_NAIVE_SINGLE
//u32 single_dtw_buf[135][135]; // Assuming you have space for n+1 and m+1 elements
#else
u32 single_dtw_buf[1][1];
#endif
u32*** multi_dtw_buf;

DTWSettings g_dtw_settings;
seq_t * dtw;

buffer_element dtw_memo[2][RING_BUFFER_MAX_SIZE];



HAL_StatusTypeDef dtw_init()
{
    g_dtw_settings = dtw_settings_default();
    dtw = (seq_t *)malloc(sizeof(seq_t) * RING_BUFFER_MAX_SIZE * 2);
    
    
    // single_dtw_buf = malloc(RING_BUFFER_MAX_SIZE * sizeof(u32*));
    // for(u32 i = 0; i< RING_BUFFER_MAX_SIZE; i++)
    // {
    //     single_dtw_buf[i] = malloc(RING_BUFFER_MAX_SIZE * sizeof(u32));
    // }

    return HAL_OK;
}

void free_dtw_buf()
{
    //free dtw buffer after preallocated
}

u32 dtw_single(buffer_element *s, u32 n, buffer_element *t, u32 m) {
    
    #ifdef ENABLE_NAIVE_SINGLE
        return 0;
    #endif
  
    u32 i, j;
    

    // Initialize the buffer
    for(i = 0; i <= n; i++) {
        for(j = 0; j <= m; j++) {
            if (i == 0 && j == 0) {
                single_dtw_buf[i][j] = 0;
            } else if (i == 0 || j == 0) {
                single_dtw_buf[i][j] = 0xFFFFFFFF; // Infinity for boundary conditions
            }
        }
    }

    // Compute the DTW matrix
    for(i = 1; i <= n; i++) {
        for(j = 1; j <= m; j++) {
            u32 cost = abs(s[i - 1] - t[j - 1]);
            single_dtw_buf[i][j] = cost + MIN(MIN(single_dtw_buf[i - 1][j], single_dtw_buf[i][j - 1]), single_dtw_buf[i - 1][j - 1]);
        }
    }

    // Return the final DTW distance
    return single_dtw_buf[n][m];
    
}

u32 dtw_single_memo(buffer_element *s, u32 n, buffer_element *t, u32 m)
{
    u32 i, j, p, c, temp;

    p = 0; c = 1;

    for(j = 0; j <= m; j++) {
        dtw_memo[p][j] = INFINITY;
        dtw_memo[c][j] = INFINITY;
    }

    dtw_memo[c][0] = 0;

    for(i = 1; i <= n; i++) 
    {
        for(j = 1; j <= m; j++) 
        {
            u32 cost = pow(s[i - 1] - t[j - 1], 2);
            dtw_memo[c][j] = cost + MIN(MIN(dtw_memo[p][j], dtw_memo[c][j - 1]), dtw_memo[p][j - 1]);
        }

        // Only 0,0 starts at 0. 
        if(i>1) {dtw_memo[p][0] = INFINITY;}

        temp = c;
        c = p;
        p = temp;
    }

    u32 distance = dtw_memo[p][m];
    distance = sqrt(distance);

    return distance;
}

DTW_Result DTW_Distance(ring_buffer_s* s, ring_buffer_s* t)
{
    DTW_Result result;
    u32 startTime = HAL_GetTick();

    //result.x_accel_result = dtw_single(      s->buffer[0],     s->write_index,        t->buffer[0],    t->write_index) ;
    //result.y_accel_result = dtw_distance(      s->buffer[1],     s->write_index,        t->buffer[1],    t->write_index, &g_dtw_settings) ;
    //result.z_accel_result = dtw_single(      s->buffer[2],     s->write_index,        t->buffer[2],    t->write_index) ;
    //uart_println("Single");
    //print_dtw_result(&result);  


    //remember that if these return 0 likely a s32 somehow went negative and got square rooted
    result.x_accel_result = dtw_single_memo(      s->buffer[0],     s->write_index,        t->buffer[0],    t->write_index) ;
    //result.y_accel_result = dtw_distance(      s->buffer[1],     s->write_index,        t->buffer[1],    t->write_index, &g_dtw_settings) ;
    result.z_accel_result = dtw_single_memo(      s->buffer[2],     s->write_index,        t->buffer[2],    t->write_index) ;
    uart_println("Memo");
    print_dtw_result(&result);

    

    g_dtw_settings = dtw_settings_default();
    //result.x_accel_result  = dtw_distance(      s->buffer[0],     s->write_index,        t->buffer[0],    t->write_index, &g_dtw_settings) ;
    //result.z_accel_result  = dtw_distance(      s->buffer[2],     s->write_index,        t->buffer[2],    t->write_index, &g_dtw_settings) ;
    //uart_println("DTAI");
    //print_dtw_result(&result);
    //result.x_gyro_result  = dtw_distance(      s->buffer[3],     s->write_index,        t->buffer[3],    t->write_index, &g_dtw_settings) ;
    //result.y_gyro_result  = dtw_distance(      s->buffer[4],     s->write_index,        t->buffer[4],    t->write_index, &g_dtw_settings) ;
    //result.z_gyro_result  = dtw_distance(      s->buffer[5],     s->write_index,        t->buffer[5],    t->write_index, &g_dtw_settings) ;
    u32 endTime = HAL_GetTick();
    uart_println("DTW time : %d %d %d", endTime - startTime, s->write_index, t->write_index);
    return result;
}

void print_dtw_result(DTW_Result* data)
{
    uart_println("X ACCEL DTW %d", data->x_accel_result);
    //uart_println("Y ACCEL DTW %d", data->y_accel_result);
    uart_println("Z ACCEL DTW %d", data->z_accel_result);
    //uart_println("X GYRO DTW %d",  data->x_gyro_result );
    //uart_println("Y GYRO DTW %d",  data->y_gyro_result );
    //uart_println("Z GYRO DTW %d",  data->z_gyro_result );
}

// void print_dtw_result(DTW_Result* data)
// {
//     uart_println("X ACCEL DTW %f", data->x_accel_result);
//     uart_println("Y ACCEL DTW %f", data->y_accel_result);
//     uart_println("Z ACCEL DTW %f", data->z_accel_result);
//     uart_println("X GYRO DTW %f",  data->x_gyro_result );
//     uart_println("Y GYRO DTW %f",  data->y_gyro_result );
//     uart_println("Z GYRO DTW %f",  data->z_gyro_result );
// }

