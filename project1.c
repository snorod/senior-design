#define THRESHOLD 70 // light sensor threshold to find the black border line
#define TOOFAR 25 // cm, range to look for objects within
#define FORWARD_TIME 3000 // ms, how long the robot should move after a full scan
#define TIME_AFTER_EDGE 1000 // ms, how long the robot should move after detecting the border line and turning around
#define ERROR_TIME 4000 // ms, how much time to move forward before determining that checkEdge failed

mutex theTalkingStick;

// completes a 180 degree turn
inline void turn_around()
{
 OnRev(OUT_A, 25);
 OnFwd(OUT_C, 25); Wait(4000);
}

// this task removes an object from the arena if it is detected by the ultrasonic sensor
task scan()
{
 while (true)
 {
  if (SensorUS(IN_4) < TOOFAR) // object within range of sensor
  {
   Acquire(theTalkingStick); // acquire mutex
   OnFwdSync(OUT_AC, 50, 0); // move toward object, slow enough to register black line

   // error block, should not be reached, because checkEdge should register black line
   // this is in place here in case the checkEdge task fails
   Release(theTalkingStick);
   Wait(ERROR_TIME);
   turn_around();
   OnFwdSync(OUT_AC, 50, 0);
   Wait(ERROR_TIME);
  }
 }
}

// this task monitors whether or not the robot is in danger of leaving the arena
// it uses the light sensor to check for the black border line
task checkEdge()
{
 while (true)
 {
  if (Sensor(IN_3) > THRESHOLD)
  {
   Acquire(theTalkingStick);
   turn_around();
   
   // move forward so that the robot will not detect the line again while rotating in the scan task
   OnFwdSync(OUT_AC, 50, 0);
   Wait(TIME_AFTER_EDGE);
   Release(theTalkingStick);
  }
 }
}

task move()
{
 while (true)
 {
  // rotate 360 degrees
  OnRev(OUT_A, 25);
  OnFwd(OUT_C, 25); Wait(8000);
  
  OnFwdSync(OUT_AC, 50, 0);
  Wait(FORWARD_TIME);
 }
}

task main()
{
 SetSensorLight(IN_3); // input port 3 controls the light sensor

 // desperately trying to get light sensor to do what i want
 SetSensorMode(IN_3, IN_MODE_PCTFULLSCALE);
 ResetSensor(IN_3);

 SetSensorLowspeed(IN_4); // input port 4 controls the ultrasonic sensor
 
 Precedes(move, scan, checkEdge);
}
