import controlP5.*;
import processing.serial.*;

import processing.serial.Serial; // serial library
import controlP5.*; // controlP5 library

ControlP5 controlP5;

ListBox commListbox;
int commListMax;
Textlabel txtlblWhichcom;

Textfield txtfldRCThrottle;
Textfield txtfldRCRoll;
Textfield txtfldRCPitch;
Textfield txtfldRCYaw;

Textfield txtfldRawAccX;
Textfield txtfldRawAccY;
Textfield txtfldRawAccZ;
Textfield txtfldRawGyroX;
Textfield txtfldRawGyroY;
Textfield txtfldRawGyroZ;

Textfield txtfldIMURoll;
Textfield txtfldIMUPitch;
Textfield txtfldIMUYaw;

Textfield txtfldMotor0;
Textfield txtfldMotor1;
Textfield txtfldMotor2;
Textfield txtfldMotor3;

Textfield txtfldDebug0;
Textfield txtfldDebug1;
Textfield txtfldDebug2;
Textfield txtfldDebug3;

Textarea txtareaSerialOut;

Serial g_serial;
int init_com=0;

final int MSP_RC = 105;
final int MSP_RAW_IMU     = 102;
final int MSP_ATTITUDE    = 108;
final int MSP_MOTOR       = 104;
final int MSP_DEBUG       = 199;

final int RC_CHANS = 16;
short rcValue[] = new short[RC_CHANS];

short AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
short IMURoll,IMUPitch,IMUYaw;
final int NUM_MOTORS = 16;
short Motors[] = new short[NUM_MOTORS];
short Debug[] = new short[4];

short serialRead16()
{
  int x = g_serial.read() + (g_serial.read() << 8);
  if (x > 32767)
    x -= 65536;
   return (short)x;
}

int fastFrameRate = 100;
void setup() {
  size(640,480);
  frameRate(fastFrameRate);
  
  controlP5 = new ControlP5(this); // initialize the GUI controls

  // make a listbox and populate it with the available comm ports
  commListbox = controlP5.addListBox("portComList",5,5,180,120); 
  commListbox.captionLabel().set("PORT COM");
  //commListbox.setColorBackground(red_);
  commListbox.setBarHeight(17);  
  for(int i=0;i<Serial.list().length;i++) {
    String pn = shortifyPortName(Serial.list()[i], 13);
    //if( pn.startsWith("/dev/ttyUSB") ) {
      if (pn.length() >0 ) commListbox.addItem(pn,i); // addItem(name,value)
      commListMax = i;
    //}
  }
  commListbox.addItem("Close Comm",++commListMax); // addItem(name,value)
  txtlblWhichcom = controlP5.addTextlabel("txtlblWhichcom","No Port Selected",5,95); // textlabel(name,text,x,y)
  
  // RC boxes
  txtfldRCThrottle = controlP5.addTextfield("Throttle",  5, 130, 40, 20);
  txtfldRCYaw      = controlP5.addTextfield("Yaw",       5, 180, 40, 20);
  txtfldRCRoll     = controlP5.addTextfield("Roll",     50, 130, 40, 20);
  txtfldRCPitch    = controlP5.addTextfield("Pitch",    50, 180, 40, 20);
  
  // Raw IMU boxes
  txtfldRawAccX    = controlP5.addTextfield("AccX",     5, 280, 40, 20);
  txtfldRawAccY    = controlP5.addTextfield("AccY",    50, 230, 40, 20);
  txtfldRawAccZ    = controlP5.addTextfield("AccZ",    50, 280, 40, 20);
  txtfldRawGyroX    = controlP5.addTextfield("GyroX",     5, 380, 40, 20);
  txtfldRawGyroY    = controlP5.addTextfield("GyroY",    50, 330, 40, 20);
  txtfldRawGyroZ    = controlP5.addTextfield("GyroZ",    50, 380, 40, 20);
  
  // Cooked IMU boxes
  txtfldIMURoll    = controlP5.addTextfield("IMURoll",   105, 280, 40, 20);
  txtfldIMUPitch   = controlP5.addTextfield("IMUPitch",  150, 230, 40, 20);
  txtfldIMUYaw     = controlP5.addTextfield("IMUYaw",    150, 280, 40, 20);
  
  // motor boxes
  txtfldMotor0     = controlP5.addTextfield("Motor0",    150, 380, 40, 20);
  txtfldMotor1     = controlP5.addTextfield("Motor1",    150, 330, 40, 20);
  txtfldMotor2     = controlP5.addTextfield("Motor2",    105, 380, 40, 20);
  txtfldMotor3     = controlP5.addTextfield("Motor3",    105, 330, 40, 20);
  // debug boxes
  txtfldDebug0     = controlP5.addTextfield("Debug0",      5, 420, 40, 20);
  txtfldDebug1     = controlP5.addTextfield("Debug1",     50, 420, 40, 20);
  txtfldDebug2     = controlP5.addTextfield("Debug2",    100, 420, 40, 20);
  txtfldDebug3     = controlP5.addTextfield("Debug3",    150, 420, 40, 20);
  
  // big debig box
  txtareaSerialOut = controlP5.addTextarea("txtareaSerialOut", "", 300,5,300,400);
  txtareaSerialOut.enableColorBackground();
}

void mspRequest(int msg) {
    // send a request for a debug message
    g_serial.write("$M<");
    g_serial.write(0);         // data length
    g_serial.write(msg);
    g_serial.write(msg); // checksum
  
}

int size = 0;
int msg = 0;
int frameCnt = 0;
int frameMax = fastFrameRate / 10;
void draw() {
  if (init_com == 1)
  {
    char c;
    String str="";
    
    // check for time to request new data
    frameCnt = (frameCnt + 1) % frameMax;
    if (frameCnt == 0)
    {
      mspRequest(MSP_DEBUG);
      mspRequest(MSP_MOTOR);
      mspRequest(MSP_RC);
      mspRequest(MSP_RAW_IMU);
      mspRequest(MSP_ATTITUDE);
    }
    
    // thow away junk until we have a new header
    while ((size == 0) && (g_serial.available()>=5) && (g_serial.read() != '$'))
        txtareaSerialOut.append("flushing one\n");
    if ((size == 0) && (g_serial.available()>=5))
    {
        int p = g_serial.read();g_serial.read(); // "M>"
        size = g_serial.read();
        msg = g_serial.read();
        //txtareaSerialOut.append("p   " + (char)p + "\n");
        //txtareaSerialOut.append("siz " + size + "\n");
        //txtareaSerialOut.append("msg " + msg + "\n");
    }
    // read a command, once we have the whole thing
    if ((size > 0) && (g_serial.available() > (size+1)))
    {
        switch (msg) {
          // RC msg
          case (MSP_RC):
            //txtareaSerialOut.append("RC msg\n");
            for (int i=0; i<(RC_CHANS); i++)
            {
              rcValue[i] = serialRead16();
            }
            g_serial.read(); // checksum
            txtfldRCThrottle.setText(Integer.toString(rcValue[0]));
            txtfldRCRoll.setText(Integer.toString(rcValue[1]));
            txtfldRCPitch.setText(Integer.toString(rcValue[2]));
            txtfldRCYaw.setText(Integer.toString(rcValue[3]));
            break;
          // RAW IMU msg
          case (MSP_RAW_IMU):
            //txtareaSerialOut.append("RAW IMU msg\n");
            AcX = serialRead16();
            AcY = serialRead16();
            AcZ = serialRead16();
            Tmp = serialRead16();
            GyX = serialRead16();
            GyY = serialRead16();
            GyZ = serialRead16();
            g_serial.read(); // checksum
//            if (GyX > 32767) GyX -= 65536;
//            if (GyY > 32767) GyY -= 65536;
//            if (GyZ > 32767) GyZ -= 65536;
            txtfldRawAccX.setText(Integer.toString(AcX));
            txtfldRawAccY.setText(Integer.toString(AcY));
            txtfldRawAccZ.setText(Integer.toString(AcZ));
            txtfldRawGyroX.setText(Integer.toString(GyX));
            txtfldRawGyroY.setText(Integer.toString(GyY));
            txtfldRawGyroZ.setText(Integer.toString(GyZ));
            break;
          case (MSP_ATTITUDE):
            //txtareaSerialOut.append("Attitude msg\n");
            IMURoll = serialRead16();
            IMUPitch = serialRead16();
            IMUYaw = serialRead16();
            g_serial.read(); // checksum
            txtfldIMURoll.setText(Integer.toString(IMURoll));
            txtfldIMUPitch.setText(Integer.toString(IMUPitch));
            txtfldIMUYaw.setText(Integer.toString(IMUYaw));
            break;
          case (MSP_MOTOR):
            //txtareaSerialOut.append("Motor msg\n");
            for (int i=0; i<(NUM_MOTORS); i++)
            {
              Motors[i] = serialRead16();
            }
            g_serial.read(); // checksum
            txtfldMotor0.setText(Integer.toString(Motors[0]));
            txtfldMotor1.setText(Integer.toString(Motors[1]));
            txtfldMotor2.setText(Integer.toString(Motors[2]));
            txtfldMotor3.setText(Integer.toString(Motors[3]));
            break;
          case (MSP_DEBUG):
            //txtareaSerialOut.append("Debug msg\n");
            for (int i=0; i<4; i++)
            {
              Debug[i] = serialRead16();
            }
            g_serial.read(); // checksum
            txtfldDebug0.setText(Integer.toString(Debug[0]));
            txtfldDebug1.setText(Integer.toString(Debug[1]));
            txtfldDebug2.setText(Integer.toString(Debug[2]));
            txtfldDebug3.setText(Integer.toString(Debug[3]));
            break;
          default:
            txtareaSerialOut.append("default msg\n");
            for (int i=0; i<size+1; i++) // +1 for checksum
              g_serial.read();
            break;
        }
        size = 0;

//      c = (char)(g_serial.read());
//      if (c == '\n')
//      {
//        //txtareaSerialOut.clear();
//        txtareaSerialOut.append(str);
//      }
//      else
//        str += c;
    }
  }
}

// coded by Eberhard Rensch
// Truncates a long port name for better (readable) display in the GUI
String shortifyPortName(String portName, int maxlen)  {
  String shortName = portName;
  if(shortName.startsWith("/dev/cu.")) shortName = "";// only collect the corresponding tty. devices
  return shortName;
}

public void controlEvent(ControlEvent theEvent) {
  if (theEvent.isGroup()) if (theEvent.name()=="portComList") InitSerial(theEvent.group().value()); // initialize the serial port selected
}

// initialize the serial port selected in the listBox
void InitSerial(float portValue) {
  if ((portValue < commListMax) && (init_com==0)) {
    String portPos = Serial.list()[int(portValue)];
    txtlblWhichcom.setValue("COM = " + shortifyPortName(portPos, 8));
    g_serial = new Serial(this, portPos, 115200);
    //SaveSerialPort(portPos);
    init_com=1;
    g_serial.buffer(256);
  } else {
    txtlblWhichcom.setValue("Comm Closed");
    init_com=0;
    init_com=0;
    g_serial.stop();
  }
}


