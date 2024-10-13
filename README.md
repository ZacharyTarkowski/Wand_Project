Wand Project

Cast "spells" with accelerometer motion recognition using Dynamic Time Warping.

Done on STM32F401RE, planning to do an actual board spin eventually.

Very much a work in progress so far. 

  Hilights so far:
  
    General Utils, mostly uart printing nice to haves.
    
    MPU6050 Utils, driver for MPU6050 accelerometer/gyroscope
    
    Ring Buffer, rudimenary ring buffer implementation. Spells so far are stored in these ring buffers.
    
    Dynamic Time Warping, compares ring buffers to see how similar they are. 
    
