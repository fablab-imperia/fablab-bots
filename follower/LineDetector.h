#ifndef LINE_DETECTOR_H
#define LINE_DETECTOR_H



#define BLACK_ON_WHITE 0
#define WHITE_ON_BLACK 1

class LineDetector {
    public: 
        
        LineDetector(unsigned char lineMode = BLACK_ON_WHITE);

        void readRawSensors(unsigned int sensorData[]);



        unsigned int readLinePosition();
        
        void printRawSensors();

    private:

        unsigned int calibratedMinimumOn=0;
        unsigned int calibratedMaximumOn=0;
        unsigned char lineMode;
        int _lastValue=0;

        unsigned int calibratedMax[8];
        unsigned int calibratedMin[8];

 

}; 


#endif