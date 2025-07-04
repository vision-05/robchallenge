#include <Motoron.h>
#include <motoron_protocol.h>

#include <WiFi.h>
#include <WiFiUdp.h>

#include <qtr.h>
#include <string>

WiFiUDP Udp; //UDP to receive WiFi signal

//WiFi variables
char ssid[] = "Timtimtimtimtim"; // your network SSID (name)
char pass[] = "ajs35ddagtxq7m2"; // your network password (use for WPA, or use as key for WEP)
int status = WL_IDLE_STATUS; // the WiFi radio's status
unsigned int localPort = 2390; // local port to listen for UDP packets


byte packetBuffer[48]; // buffer to hold incoming and outgoing packets

const int count = 9;
const int halfwidth = 6;
constexpr float invRadius = 1/6.5;
const float invArrayCount = 0.333;
QTR qtrs[3];
uint8_t sensors[3][count] = {{22,24,26,28,30,32,34,36,38},
                             {23,25,27,29,31,33,35,37,39},
                             {42,43,44,45,46,47,48,49,50}};

//allow for changing over serial
float kp = 0.04f; //p225
float kd = 0.0f; //d5
float ki = 0.0f;//i0.1

int killStatus = 0;

int blackthresholds[3] = {5,5,5};
const float samplet = 0.3;
constexpr float invSamplet = 1/0.3;
constexpr float invCount = 1/9;

MotoronI2C mdL(15);
MotoronI2C mdR(17);

float sum(float* arr) {
  int s = 0;
  for(int i = 0; i < count; ++i) {
    s += arr[i];
  }
  return s;
}

int32_t prevErrors[3][count] = {{0,0,0,0,0,0,0,0,0},
                                {0,0,0,0,0,0,0,0,0},
                                {0,0,0,0,0,0,0,0,0}};

void PID(float velocity) {
    //sensors to errors
    float errors[3][count] = {{-12,-9,-6,-3,0,3,6,9,12},
                              {-8,-6,-4,-2,0,2,4,6,8},
                              {-4,-3,-2,-1,0,1,2,3,4}};

    for(int i = 0; i < 3; ++i) {
      for(int j = 0; j < count; ++j) {
        errors[i][j] *= qtrs[i][j];
      }
    }

    //derivative errors
    float derivErrors[3][count];
    for(int i = 0; i < 3; ++i) {
      for(int j = 0; j < count; ++j) {
        derivErrors[i][j] = (errors[i][j] - prevErrors[i][j])*10; //divide optimised at compile time
      }
    } 

    //"integral" errors
    float integralErrors[3][count];
    for(int i = 0; i < 3; ++i) {
      for(int j = 0; j < count; ++j) {
        integralErrors[i][j] = (errors[i][j] + prevErrors[i][j])*samplet; //divide optimised at compile time
      }
    } 

    //total steering signal
    float pe[3] = {sum(errors[0]),sum(errors[1]),sum(errors[2])};
    float de[3] = {sum(derivErrors[0]),sum(derivErrors[1]),sum(derivErrors[2])};
    float ie[3] = {sum(integralErrors[0]),sum(integralErrors[1]),sum(integralErrors[2])};

    float mpe = (pe[0] + pe[1] + pe[2])*invArrayCount;
    float mde = (de[0] + de[1] + de[2])*invArrayCount;
    float mie = (ie[0] + ie[1] + ie[2])*invArrayCount;

    float dtheta = -kp*mpe - kd*mde + ki*mie;

    // if (dtheta < 0.05f){
    //   dtheta = 0;
    // }

    Serial.print("DTheta ");
    Serial.print(dtheta);
    Serial.println();
    float omega = dtheta*invSamplet;
    Serial.print("Omega ");
    Serial.print(omega);
    Serial.println();
    
    //inverse kinematics

    float phiL = (velocity*invRadius) - (omega*invRadius*0.3);
    float phiR = (velocity*invRadius) + (omega*invRadius*0.3);


    if (phiL < 0.7 && phiR < 0.7){
      phiL = phiL * 1.8;
      phiR = phiR * 1.8;
    }
    else {
      phiL = phiL * 1.25;
      phiR = phiR * 1.25;
    }

    Serial.print("PhiL: ");
    Serial.print(phiL);
    Serial.print("PhiR: ");
    Serial.print(phiR);
    Serial.println();

    //run motors

    mdL.setSpeed(1,250 + 800*phiL);
    mdL.setSpeed(3,250 + 800*phiL);

    mdR.setSpeed(1,-250 - 800*phiR);
    mdR.setSpeed(3,-250 - 800*phiR);

    //set prev errors for next loop
    for(int i = 0; i < 3; ++i) {
      for(int j = 0; j < count; ++j) {
        prevErrors[i][j] = errors[i][j];
      }
    }
    delay(100);
}

void setupUDP() {
  while (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    delay(500);
  }

  // attempt to connect to WiFi network:
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to WPA SSID: ");
    Serial.println(ssid);

    status = WiFi.begin(ssid, pass);

    // 5 second delay waiting for connection
    delay(5000);
  }

  Serial.println("You're connected to the network");
  IPAddress ip = WiFi.localIP();
  Serial.println(ip);

  Udp.begin(localPort);
}

void setupQTR() {
  for(int i = 0; i < 3; ++i) {
    qtrs[i].setTimeout(600);
    qtrs[i].setSensorPins(sensors[i],count);
    qtrs[i].setThreshold(&blackthresholds[i]);
  }

  for(uint16_t i = 0; i < 20; i++) {
    for(int i = 0; i < 3; ++i) {
      qtrs[i].calibrate(10, Emitter::Off, Parity::EvenAndOdd);
      delay(100);
    }
  }


  for(int j = 0; j < 3; ++j) {
    for(uint8_t i = 0; i < count; i++) {
      Serial.print(qtrs[j].getCalMin(1));
      Serial.print(' ');
    }
    Serial.println();

    for(uint8_t i = 0; i < count; i++) {
      Serial.print(qtrs[j].getCalMax(1));
      Serial.print(' ');
    }
    Serial.println();
  }

  //qtr.setEvenEmitter(3);
  //qtr.setOddEmitter(2);

  //qtr.emitterTest();
}

void setupMotors() {
  Serial.println("Initialising Left Motoron Motor Driver...");
  Wire.begin();
  mdL.reinitialize();
  Serial.println("Initd");
  mdL.disableCrc();
  mdL.clearResetFlag();
  Serial.println("Left Motoron Motor Driver Connected!");

  Serial.println("Initialising Right Motoron Motor Driver...");
  mdR.reinitialize();
  mdR.disableCrc();
  mdR.clearResetFlag();
  Serial.println("Right Motoron Motor Driver Connected!");

  mdR.setMaxAcceleration(1,80);
  mdL.setMaxAcceleration(1,80);
  mdR.setMaxDeceleration(1,300);
  mdL.setMaxDeceleration(1,300);
  mdR.setMaxAcceleration(3,80);
  mdL.setMaxAcceleration(3,80);
  mdR.setMaxDeceleration(3,300);
  mdL.setMaxDeceleration(3,300);
}

String readSerialCommand(char* com) {
  int incomingByte;
  int c = -1;
  int idx = 0;
  int bten[4] = {1000,100,10,1};
  int res = 0;
  while (Serial.available() > 0) {
    incomingByte = Serial.read();
    if(c == -1) {
      if(incomingByte > 58) {
        char command = incomingByte;
        String f = Serial.readString();
        float k = f.toFloat()/100;
        Serial.print("K: ");
        Serial.println(k);
        if(command == 'k') {
          mdL.setSpeed(1,0);
          mdL.setSpeed(3,0);

          mdR.setSpeed(1,0);
          mdR.setSpeed(3,0);
          while(!Serial.available()) {
            Serial.println("killed");
          }
          break;
        }
        if(command == 'p') {
          kp = k;
        }
        if(command == 'i') {
          ki = k;
        }
        if(command == 'd') {
          kd = k;
        }
        break;
      }
      idx = incomingByte-48;
      ++c;
      continue;
    }
    if(c == 4) {
      Serial.println(res);
      blackthresholds[idx] = res;
      Serial.println(qtrs[idx].getThreshold());
      delay(3000);
      break;
    }
    res += bten[c]*(incomingByte-48);
    Serial.println(incomingByte);
    ++c;
  }
}

String readUDPCommand(char* com) {
  int packSize = Udp.parsePacket();
  Serial.println(packSize);
  if(packSize > 1) {
    Udp.read(packetBuffer, 48);
    *com = packetBuffer[0];
    String f = (char*)(packetBuffer + sizeof(char));
    Serial.println(f);
    return f;
    }
  if(packSize == 1) {
    Udp.read(packetBuffer, 48);
    *com = packetBuffer[0];
    return "nil";
  }
  return "none";
}

int handleCommand(char command, String payload, int kill) {
  float k;
  Serial.print("Command: ");
  Serial.println(command);
  if(command == 'p') {
    k = payload.toFloat()/100;
    kp = k;
    return kill;
  }
  if(command == 'i') {
    k = payload.toFloat()/100;
    ki = k;
    return kill;
  }
  if(command == 'd') {
    k = payload.toFloat()/100;
    kd = k;
    return kill;
  }
  if(command == 'f') {
    return 0;
  }
  if(command == 'k') {
    return 1;
  }
  return kill;
}

/*void readSendQTR(QTR* sensors) {

}*/

void setup() {
  Serial.begin(9600);

  setupUDP();
  setupQTR();
  setupMotors();
  killStatus = 0;
}

void loop() {
  char command;
  String payload;
  //payload = readSerialCommand();
  payload = readUDPCommand(&command);
  killStatus = handleCommand(command, payload, killStatus);

  qtrs[0].readCalibrated();
  qtrs[1].readCalibrated();
  qtrs[2].readCalibrated();

  for(int i = 0; i < 3; ++i) {
    std::string nums = std::to_string(i+1) + "c"; //key is calibrated
    for(int j = 0; j < count; ++j) {
      nums += std::to_string(qtrs[i][j]);
      nums += " ";
    }
    Serial.println(nums.c_str());
    Udp.beginPacket("10.17.186.85",2300);
    Udp.write(nums.c_str(),nums.length());
    Udp.endPacket();
  }
  
  
  qtrs[0].readBlackLine();
  qtrs[1].readBlackLine();
  qtrs[2].readBlackLine();

  for(int i = 0; i < 3; ++i) {
    std::string nums = std::to_string(i+1) + "b"; //key is blacklined
    for(int j = 0; j < count; ++j) {
      nums += std::to_string(qtrs[i][j]);
      nums += " ";
    }
    Serial.println(nums.c_str());
    Udp.beginPacket("10.17.186.85",2300);
    Udp.write(nums.c_str(),nums.length());
    Udp.endPacket();
  }
  
  if(!killStatus) {
    Serial.println("PID");
    PID(2);
  }
}
