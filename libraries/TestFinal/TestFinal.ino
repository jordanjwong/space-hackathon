/* Jordan Wong, UC Berkeley c/o 2017 
*  Implementation for the ARLISS Challenge.
*  Our code is based off of documentation files
*  and examples found on the web.
*
*  ***EXAMPLE SOURCES***
*  Magnetometer:
*  http://sfecdn.s3.amazonaws.com/datasheets/Sensors/Magneto/HMC5883.pde
*  Barometer:
*  https://github.com/sparkfun/BMP180_Breakout/blob/master/software/Arduino/libraries/sfecdn_BMP180/examples/SFE_BMP180_example/SFE_BMP180_example.ino
*/

// Sketch must #include this library, and the Wire library.
// (Wire is a standard library included with Arduino.):

#include <Wire.h>
#include <Servo.h>
#include <SFE_BMP180.h>

Servo servo1; //create servo object to control servo1
Servo servo2; //creat servo object to control servo2
int pos = 0; //variable to store the servo position
int x1 = 0; //initial value for the change in x-position (horizontal axis)

// Need to create an SFE_BMP180 object, here called "pressure":

SFE_BMP180 pressure;

#define ALTITUDE 1305.0 //Altitude at Black Rock Desert
#define address 0x1E //0011110b, I2C 7bit address of HMC5883

void setup()
{
  Serial.begin(9600);
  Serial.println("REBOOT");

  // Initialize the sensor (it is important to get calibration values stored on the device).

  if (pressure.begin())
    Serial.println("BMP180 init success");
  else
  {
    // Oops, something went wrong, this is usually a connection problem,
    // see the comments at the top of this sketch for the proper connections.

    Serial.println("BMP180 init fail\n\n");
    while(1); // Pause forever.
  }

  //Initialize Serial and I2C communications
  Serial.begin(9600);
  Wire.begin();
  
  //Put the HMC5883 IC into the correct operating mode
  Wire.beginTransmission(address); //open communication with HMC5883
  Wire.write(0x02); //select mode register
  Wire.write(0x00); //continuous measurement mode
  Wire.endTransmission();
  
  servo1.attach(3); // attaches servo1 on pin 3 to the servo object
  servo2.attach(9); //attaches servo2 on pin 9 to the servo object

}

void loop()
{

  // DETERMINING ALTITUDE

  char status;
  double T,P,p0,a;

  // Loop here getting pressure readings every 1 seconds.

  // If you want sea-level-compensated pressure, as used in weather reports,
  // you will need to know the altitude at which your measurements are taken.
  // We're using a constant called ALTITUDE in this sketch:
  
  Serial.println();
  Serial.print("provided altitude: ");
  Serial.print(ALTITUDE,0);
  Serial.print(" meters, ");
  Serial.print(ALTITUDE*3.28084,0);
  Serial.println(" feet");
  
  // If you want to measure altitude, and not pressure, you will instead need
  // to provide a known baseline pressure. This is shown at the end of the sketch.

  // You must first get a temperature measurement to perform a pressure reading.
  
  // Start a temperature measurement:
  // If request is successful, the number of ms to wait is returned.
  // If request is unsuccessful, 0 is returned.

  status = pressure.startTemperature();
  if (status != 0)
  {
    // Wait for the measurement to complete:
    delay(status);

    // Retrieve the completed temperature measurement:
    // Note that the measurement is stored in the variable T.
    // Function returns 1 if successful, 0 if failure.

    status = pressure.getTemperature(T);
    if (status != 0)
    {
      // Print out the measurement:
      Serial.print("temperature: ");
      Serial.print(T,2);
      Serial.print(" deg C, ");
      Serial.print((9.0/5.0)*T+32.0,2);
      Serial.println(" deg F");
      
      // Start a pressure measurement:
      // The parameter is the oversampling setting, from 0 to 3 (highest res, longest wait).
      // If request is successful, the number of ms to wait is returned.
      // If request is unsuccessful, 0 is returned.

      status = pressure.startPressure(3);
      if (status != 0)
      {
        // Wait for the measurement to complete:
        delay(status);

        // Retrieve the completed pressure measurement:
        // Note that the measurement is stored in the variable P.
        // Note also that the function requires the previous temperature measurement (T).
        // (If temperature is stable, you can do one temperature measurement for a number of pressure measurements.)
        // Function returns 1 if successful, 0 if failure.

        status = pressure.getPressure(P,T);
        if (status != 0)
        {
          // Print out the measurement:
          Serial.print("absolute pressure: ");
          Serial.print(P,2);
          Serial.print(" mb, ");
          Serial.print(P*0.0295333727,2);
          Serial.println(" inHg");

          // The pressure sensor returns abolute pressure, which varies with altitude.
          // To remove the effects of altitude, use the sealevel function and your current altitude.
          // This number is commonly used in weather reports.
          // Parameters: P = absolute pressure in mb, ALTITUDE = current altitude in m.
          // Result: p0 = sea-level compensated pressure in mb

          p0 = pressure.sealevel(P,ALTITUDE); // we're at 1305 meters (Black Rock)
          Serial.print("relative (sea-level) pressure: ");
          Serial.print(p0,2);
          Serial.print(" mb, ");
          Serial.print(p0*0.0295333727,2);
          Serial.println(" inHg");

          // On the other hand, if you want to determine your altitude from the pressure reading,
          // use the altitude function along with a baseline pressure (sea-level or other).
          // Parameters: P = absolute pressure in mb, p0 = baseline pressure in mb.
          // Result: a = altitude in m.

          a = pressure.altitude(P,p0);
          Serial.print("computed altitude: ");
          Serial.print(a,0);
          Serial.print(" meters, ");
          Serial.print(a*3.28084,0);
          Serial.println(" feet");
          
          int x,y,z; //triple axis data

          //Tell the HMC5883 where to begin reading data
          Wire.beginTransmission(address);
          Wire.write(0x03); //select register 3, X MSB register
          Wire.endTransmission();
  
 
          //Read data from each axis, 2 registers per axis
          Wire.requestFrom(address, 6);
          if(6<=Wire.available()){
            x = Wire.read()<<8; //X msb
            x |= Wire.read(); //X lsb
            z = Wire.read()<<8; //Z msb
            z |= Wire.read(); //Z lsb
            y = Wire.read()<<8; //Y msb
            y |= Wire.read(); //Y lsb
          }
  
          //Print out values of each axis
          Serial.print("x: ");
          Serial.print(x);
          Serial.print("  y: ");
          Serial.print(y);
          Serial.print("  z: ");
          Serial.println(z);


          // Point up!
          if (ALTITUDE >= a)
          {
            pos = 0;
            servo1.write(pos);
          }
          // Point down!
          else if (ALTITUDE < a)
          {
            pos = 30;
            servo1.write(pos);
          }
          // Point left!
          if (x1 != 0)
          {
            if ((x - x1) > 0)
            {
              pos = 30;
              servo2.write(pos);
            }
            // Point right!
            else if ((x - x1) < 0)
            {
              pos = 0;
              servo2.write(pos);
            }
          }
          x1 = x;
        }
        else Serial.println("error retrieving pressure measurement\n");
      }
      else Serial.println("error starting pressure measurement\n");
    }
    else Serial.println("error retrieving temperature measurement\n");
  }
  else Serial.println("error starting temperature measurement\n");
  delay(1000);  // Pause for 1 second.
}
