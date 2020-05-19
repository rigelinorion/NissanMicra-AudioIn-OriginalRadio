
// program emulating a CDC changer for the original radio (blaupunkt) in nissan micra k12
// unlocks audioin lines and allows you to connect an external audio source to the miniiso connector (blue block)
// written for ATTINY85 or ATTINY45 - minimum clock frequency 8MHz fusebits - Lo: E2 Hi: D7
// rigelinorion 2019
//rigel.inorion@wp.pl


#include <SoftwareSerial9.h> //library that supports 9bit transmission

            // 0   1   2   3   4   5   6   7   8   9   10  11  12  13  14  15  16   17   18   19   20   21   22   23
int _HEX[] = {0x0,0x1,0x2,0x3,0x4,0x5,0x6,0x7,0x8,0x9,0xA,0xB,0xC,0xD,0xE,0xF,0x16,0x17,0x18,0x19,0x1A,0x1B,0x1C,0x1D,//>23
            // 24   25   26   27   28   29   30   31   32   33   34   35   36   37   38   39   40   41   42   43   44  
              0x1E,0x1F,0x26,0x27,0x28,0x29,0x2A,0x2B,0x2C,0x2D,0x2E,0x2F,0x36,0x37,0x38,0x39,0x3A,0x3B,0x3C,0x3D,0x3E,
            // 45   46   47   48   49   50   51   52   53   54   55   56   57   58   59   60   61   62   63   64   65   66
              0x3F,0x46,0x47,0x48,0x49,0x4A,0x4B,0x4C,0x4D,0x4E,0x4F,0x56,0x57,0x58,0x59,0x5A,0x5B,0x5C,0x5D,0x5E,0x5F,
            // 67   68   69   70   71   72   73   74   75   76   77   78   79   80   81   82   83   84   85   86   87   88
              0x66,0x76,0x68,0x69,0x6A,0x6B,0x6C,0x6D,0x6E,0x6F,0x76,0x77,0x78,0x79,0x7A,0x7B,0x7C,0x7D,0x7E,0x7F,0x86,0x87,
            // 90   91   92   93   94   95   96   97   98   99   100
              0x89,0x8A,0x8B,0x8C,0x8d,0x8E,0x8F,0x96,0x97,0x98,0x99
          };
          
int  frame_buf[9]; 
int  frame_buf_Count = -1;
bool captureFrame    = false;
bool cdcin           = false;
int counter          = 0;
bool transmiting     = false;

#define frameEndMarker   0x14F
#define frameDelay       1
#define pin_voltage     A2

SoftwareSerial9 mySerial(0,1); // RX, TX >>> if won't work swap it

void setup() {
  mySerial.begin(4800);
}

void loop() {
  int hold;
  counter++;
  //if counter >= send to display acu voltage
  if(counter>=10000){
    counter=0;
    if(cdcin) displayVoltage();
  }
  
  if ((mySerial.available())) {
    while (mySerial.available() > 0) {
        hold = mySerial.read();
        
        if(hold==0x180){      //start sign of the frame broadcast by radio
          captureFrame=true;  //set the capture flag
          clearFrameBuffer();
        }
      
        if(captureFrame){ //if capture flag is set
          if(hold!=frameEndMarker){
            mySerial.write9(hold); //returns the char to the radio
            delay(frameDelay);
            frame_buf_Count++; //increasing the frame buffer index
            frame_buf[frame_buf_Count]=hold;  //save the character to the frame buffer
          }

          if(hold==frameEndMarker) {  //if the radio broadcast a frame-end marker
              captureFrame=false;     //remove frame capture flag          
          }
        }
    }
    
    switch(frame_buf[1]){ 

      case 0x0A5:               //the radio said it was in CDC mode
        clearFrameBuffer();
        cdcin=true;
      break;

      case 0x48: 
        if(frame_buf[2]==0x002){ // the radio sent a request to change the communication speed
          mySerial.flush();
          mySerial.begin(9600);
          clearFrameBuffer();
        }
        if(frame_buf[2]==0x001){ //first frame broadcast by radio
          mySerial.write9(0x10F);delay(frameDelay);
          mySerial.write9(0x048);delay(frameDelay);
          mySerial.write9(0x001);delay(frameDelay);
          mySerial.write9(frameEndMarker);
          clearFrameBuffer();
        }

      break;

      case 0x021:// after exiting the radio from CDC mode (switching to radio or before turning off the radio)
        mySerial.write9(0x103);delay(frameDelay);
        mySerial.write9(0x020);delay(frameDelay);
        mySerial.write9(0x00A);delay(frameDelay);
        mySerial.write9(0x020);delay(frameDelay);
        mySerial.write9(0x000);delay(frameDelay);
        mySerial.write9(frameEndMarker);
        clearFrameBuffer();
        cdcin=false;
      break;

      case 0x0B0: clearFrameBuffer();
      break;
    }
  }
}

void clearFrameBuffer(){
  frame_buf[0]=0;
  frame_buf[1]=0;
  frame_buf[2]=0;
  frame_buf[3]=0;
  frame_buf[4]=0;
  frame_buf[5]=0;
  frame_buf[6]=0;
  frame_buf[7]=0;
  frame_buf[8]=0;
  frame_buf_Count=-1;
}
void transmitFrame(int args[], int flength){
  
  while(captureFrame) delayMicroseconds(10);
 
  delay(30);;
  transmiting = true;
    for (int i=0; i<flength; i++){ 
       mySerial.write9(args[i]); 
       delay(frameDelay); 
    }
  delay(30);;
  transmiting = false;
}

void setTimeSection(int m, int s){
  int args[] = {0x109, _HEX[m], _HEX[s], frameEndMarker};
  transmitFrame(args, 4);
}
void setDIGITSection(int disc, int track){
   int args[] = {0x101, _HEX[disc], _HEX[track], frameEndMarker};
   transmitFrame(args, 4);
}
void displayVoltage(){
  int raw = analogRead(A2);
  float voltage = ((21.0 / 1023.0)*raw)+1;
  int fv = int(voltage);
  int sv = (voltage - fv)*10; //it should by *100 but time section in sec accept only 0..59
  setTimeSection(fv, sv);
}
