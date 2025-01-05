Wand Project

Cast "spells" with accelerometer motion recognition using Dynamic Time Warping (DTW).

Done on STM32F401RE with the MPU6050 accelerometer, planning to do an actual board spin eventually.

Current features:

  Accelerometer based rotation cancellation : 
  
    Accelerometer is periodically sampled to get the current orientation, which can then cancelled out (rotated by the negative angles of pitch and roll. Don't care about yaw). This effectively aligns the gravity vectors so that two accelerometer captures can be compared using DTW.

  Dynamic Time Warping :
  
    Work in progress, but compares two buffers to generate a "measure" of how similar they are. Originally ripped the C backend out from the DTAIDTW python library for this, but am attempting to make my own using
  
      Abdullah Mueen, Eamonn J. Keogh: Extracting Optimal Performance from Dynamic Time Warping. KDD 2016: 2129-2130 (https://www.cs.unm.edu/~mueen/DTW.pdf)
    
