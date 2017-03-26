#include "LineDetector.h"
#include <Arduino.h>
#include <Wire.h>

LineDetector::LineDetector(unsigned char lm)  
{
    lineMode = lm;
    Wire.begin();
}


void LineDetector::readRawSensors(unsigned int sensorData[])
{
    int t = 0;
    Wire.requestFrom(9, 16);    // request 16 bytes from slave device #9
    while (Wire.available())   // slave may send less than requested
    {
        sensorData[t]  = Wire.read(); // receive a byte as character  
        unsigned int secondByte = Wire.read();
        if (t < 7)
        t++;
    }
}


void LineDetector::printRawSensors() 
{
    unsigned int data[8];

    readRawSensors(data);

    for (int i=0; i < 8; i++) 
    {
        Serial.print("SENSORE ");
        Serial.print(i);
        Serial.print(": ");
        Serial.println(data[i]);
    }

}



unsigned int LineDetector::readLinePosition()
{
    unsigned char i;
    unsigned long avg; // this is for the weighted total, which is long
                       // before division
    unsigned int sum; // this is for the denominator which is <= 64000

    unsigned int sensor_values[8];
	
	
	readRawSensors(sensor_values);

    avg = 0;
    sum = 0;

    for(i=0;i<8;i++) {
        /*unsigned int value = sensor_values[i]/18;
		
		if (value<0) 
			value=0;
		if (value>10) 
			value=10;
		
		unsigned int remapped = 10 -value;
		
		if (remapped < 5 && remapped > 2)
			remapped = remapped -2;

		Serial.print("remapped: ");
		Serial.println(remapped);
			avg += (long)(remapped) * (i*10);
			sum += remapped;*/
		
//		unsigned int val = sensor_values[i];
		
	//	if (val < 60)
			  
		unsigned int val = sensor_values[i];
		if (val < 100)
			val = 1;
		else 
			val = 0;

		avg += (long)(val) * (i*1000);
		sum += val;
		 
    }


	if(sum == 0)
		return 0;
	

	return avg/sum;
}
