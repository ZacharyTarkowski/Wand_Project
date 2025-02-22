#include "Dynamic_Time_Warping.h"

#define DTW_TIMING_ANALYSIS

#ifdef ENABLE_NAIVE_SINGLE
//u32 single_dtw_buf[135][135]; // Assuming you have space for n+1 and m+1 elements
#else
u32 single_dtw_buf[1][1];
#endif
u32*** multi_dtw_buf;

DTWSettings g_dtw_settings;
seq_t * dtw;


buffer_element dtw_memo[2][RING_BUFFER_MAX_SIZE];

/**   
*   Name  : dtw_init
*   Brief : Allocates the memory used for dynamic time warping (in the dtaid library, not the homebrew)
*   Param : 
*   Return: 
*/
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

/**   
*   Name  : free_dtw_buf
*   Brief : Frees the allocated DTW memory
*   Param : 
*   Return: 
*/
void free_dtw_buf()
{
    free(dtw);
}

/**   
*   Name  : dtw_single
*   Brief : Depricated, naive implementation of dynamic time warping [ O(n^2) space and time efficient ]
*   Param : 
*   Return: 
*/
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

/**   
*   Name  : dtw_single_memo
*   Brief : Depricated, memoized dynamic time warping. [ O(n^2) time efficient, O(2*n) space efficient) ]
*   Param : 
*   Return: 
*/
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


/**   
*   Name  : dtw_single_memo_with_window
*   Brief : Handmade Dynamic Time Warping with window and memoization [ O(n*w) time complexity, O(2*n) space complexity) ]
*   Param : Takes two buffers and their lengths.
*   Return: Returns the DTW distance between the two buffers. Also may return 46340, which is sqrt(0x7FFFFFFF), meaning a minimum cost path was not found. 
*/
u32 dtw_single_memo_with_window(buffer_element *s, u32 n, buffer_element *t, u32 m)
{
    s32 i, j, p, c, temp, w;

    w = DTW_WINDOW;

    p = 0; c = 1;

    //todo replace this with a memset no reason to loop
    for(j = 0; j <= m; j++) {
        dtw_memo[p][j] = INFINITY;
        dtw_memo[c][j] = INFINITY;
    }

    dtw_memo[c][0] = 0;

    for(i = 1; i <= n; i++) 
    {

        // Only 0,0 starts at 0. Backfill infinities outside of the working band.
        // Mueen and Keogh omit this step when applying memoization...
        if(i>1) 
        {
            //todo replace this with a memset no reason to loop
            for(j = 0; j<MAX(1,i-w); j++)
            {
                dtw_memo[c][j] = INFINITY;
            }
        }

        for(j = MAX(1,i-w); j <= MIN(m,i+w); j++) 
        {
            

            //original dtw cost function. 
            //decided against it based on Parameterizing the cost function of Dynamic Time Warping with application to time series classification Matthieu Herrmann · Chang Wei Tan · Geoffrey I. Webb
            //https://arxiv.org/pdf/2301.10350

            //u32 cost = abs(s[i - 1] - t[j - 1]);

            //don't use POW for ints! slows it way down, probably has to use the FPU
            //u32 cost = pow(s[i - 1] - t[j - 1], 2);
            s32 cost = s[i - 1] - t[j - 1]; 
            s32 prevcost = cost;
            cost = cost * cost;

            if(cost < prevcost)
            {
                uart_println("overflow!");
            }
            
            
            dtw_memo[c][j] = cost + MIN(MIN(dtw_memo[p][j], dtw_memo[c][j - 1]), dtw_memo[p][j - 1]);
        }

        temp = c;
        c = p;
        p = temp;
    }

    u32 distance = dtw_memo[p][m];

    distance = sqrt(distance);

    return distance;
}

/**   
*   Name  : dtw_single_memo_with_window_ring_buffer
*   Brief : Bastardized dtw_single_memo_with_window function to run starting from any element of a ring buffer
*   Param : Takes two ring buffers buffers and their lengths.
*   Return: Returns the DTW distance between the two buffers. Also may return 46340, which is sqrt(0x7FFFFFFF), meaning a minimum cost path was not found. 
*/
u32 dtw_single_memo_with_window_ring_buffer(ring_buffer_s *s, u32 n, ring_buffer_s *t, u32 m, u32 dim)
{
    s32 i, j, p, c, temp, w;

    w = DTW_WINDOW;

    p = 0; c = 1;

    //todo replace this with a memset no reason to loop
    for(j = 0; j <= m; j++) {
        dtw_memo[p][j] = INFINITY;
        dtw_memo[c][j] = INFINITY;
    }

    dtw_memo[c][0] = 0;

    for(i = 1; i <= n; i++) 
    {

        // Only 0,0 starts at 0. Backfill infinities outside of the working band.
        // Mueen and Keogh omit this step when applying memoization...
        if(i>1) 
        {
            //todo replace this with a memset no reason to loop
            for(j = 0; j<MAX(1,i-w); j++)
            {
                dtw_memo[c][j] = INFINITY;
            }
        }

        for(j = MAX(1,i-w); j <= MIN(m,i+w); j++) 
        {
            

            //original dtw cost function. 
            //decided against it based on Parameterizing the cost function of Dynamic Time Warping with application to time series classification Matthieu Herrmann · Chang Wei Tan · Geoffrey I. Webb
            //https://arxiv.org/pdf/2301.10350

            //u32 cost = abs(s[i - 1] - t[j - 1]);

            //don't use POW for ints! slows it way down, probably has to use the FPU
            //u32 cost = pow(s[i - 1] - t[j - 1], 2);
            s32 cost = ring_buffer_read(s, dim, i-1) - ring_buffer_read(t, dim, j-1); 
            cost = cost * cost;
            
            dtw_memo[c][j] = cost + MIN(MIN(dtw_memo[p][j], dtw_memo[c][j - 1]), dtw_memo[p][j - 1]);
        }

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

    #ifdef DTW_TIMING_ANALYSIS
    u32 startTime = HAL_GetTick();
    u32 endTime;
    #endif

    result.x_accel_result = dtw_single_memo_with_window(      s->buffer[0],     s->write_index,        t->buffer[0],    t->write_index) ;
    result.z_accel_result = dtw_single_memo_with_window(      s->buffer[2],     s->write_index,        t->buffer[2],    t->write_index) ;

    //uart_println("Memo");
    //print_dtw_result(&result);

    #ifdef DTW_TIMING_ANALYSIS
    endTime = HAL_GetTick();
    //uart_println("DTW time : %d %d %d", endTime - startTime, s->write_index, t->write_index);
    startTime = HAL_GetTick();
    #endif

    //sqrt of 7FFFFFFF, didn't find a min cost path
    if(result.x_accel_result == 46340 || result.z_accel_result == 46340 || result.z_accel_result == INFINITY || result.z_accel_result == INFINITY)
    {
        uart_println("No path found, window likely must be larger");
    }

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

