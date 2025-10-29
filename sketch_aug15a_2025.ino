////SPI//////
#define MOSI PB4
#define MISO PB3
#define SCK PB5
void SPIbegin() {
  DDRB |= (1 << MOSI) | (1 << SCK);  //one
  DDRB &= ~(1 << MISO);              //zero;
  PORTB &= ~((1 << MOSI) | (1 << SCK));
}
void SPIwrite(byte value) {
  for (int8_t i = 7; i >= 0; i--) {
    bitWrite(PORTB, MOSI, bitRead(value, i));
    bitWrite(PORTB, SCK, 1);
    bitWrite(PORTB, SCK, 0);
  }
  bitWrite(PORTB, MOSI, 0);
}

byte SPIread() {
  byte value = 0;
  for (int8_t i = 7; i >= 0; i--) {
    value |= bitRead(PINB, MISO) << i;
    bitWrite(PORTB, SCK, 1);
    bitWrite(PORTB, SCK, 0);
  }
  return value;
}
///SPI//////


//#define TESTING_CONDITION
//#define MOTORS_DIRECTION_REVERSED
#define DataDir 10
#define OK_LED 9
#define ERR_LED 8

#if defined (TESTING_CONDITION)
  #define PCB_ADDR 0
#else
  #define PCB_ADDR PINC
#endif

#define PCB_ADDR_DIR DDRC

//expander
#define IODIRA 0x00  //0 is output
#define IODIRB 0x01
#define GPIOA 0x12
#define GPIOB 0x13

#define MOTOR_CENTER 40
#define LIMIT MOTOR_CENTER - 2

#define BAUDRATE 115200

const byte motorStart = 0b11001100;
int currentPos[16];
int setPos[16];
unsigned int T[16]; //time beetwen steps
bool motorSleepStatus[16];
byte motorState[8];
byte motorStateBeforeSleep[8];
unsigned long stepTimer;
const byte MOTOR_CONNECTION_TAB[] = { 0,1,8,9,3,2,10,11,5,12,13,14,7,6,15,4};


void writeReg(byte pin, byte addr, byte value);
byte readReg(byte pin, byte addr);

enum directions{
  UP = 0,
  DOWN = 1
};
struct Motor{
  byte expander;
  byte tabAddr;
  byte reg;
  byte shift;
};

void axisReset();
void setMotor(byte addr, unsigned int T_tmp,int Pos_tmp); //zero based

bool getMotor(byte num,Motor &motor);
void stepMotor(Motor motor, directions dir); //zero based
void sleepMotor(Motor motor);
void awakeMotor(Motor motor);

void setup() {
  Serial.begin(BAUDRATE);

  #if defined (TESTING_CONDITION)
  Serial.println("Welcome in test mode :)");
  #endif

  Serial.setTimeout(5);
  SPIbegin();

  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(DataDir, OUTPUT);
  pinMode(OK_LED, OUTPUT);
  pinMode(ERR_LED, OUTPUT);

  digitalWrite(2, HIGH);
  digitalWrite(3, HIGH);
  digitalWrite(4, HIGH);
  digitalWrite(5, HIGH);

  digitalWrite(OK_LED, LOW);
  digitalWrite(ERR_LED, LOW);
  digitalWrite(DataDir, LOW);

  //Testing connection
  byte test = 0xAA;
  for (byte i = 2; i < 6; i++) {
    writeReg(i, IODIRA, test);
    if (readReg(i, IODIRA) != test) {
      #ifndef TESTING_CONDITION
      digitalWrite(ERR_LED, HIGH);
      #endif
    }
  }



  for (byte i = 2; i < 6; i++) {
    writeReg(i, IODIRA, 0);
    writeReg(i, IODIRB, 0);

    writeReg(i, GPIOA, motorStart);
    writeReg(i, GPIOB, motorStart);
  }

  PCB_ADDR_DIR = 0; //device address selector
  //Serial.print("My addres is: ");
  //Serial.println(PCB_ADDR,HEX); //address
  digitalWrite(OK_LED, HIGH);
  axisReset();
  delay(1000);
  digitalWrite(OK_LED, LOW);
  //Serial.println(PINC, HEX);
  stepTimer = micros();
}


unsigned int timer = 1;

void loop() {
  unsigned long currentMicros = micros();
  if(abs(currentMicros-stepTimer)>=6536){ //takes max 2396 us but further testing needed
    stepTimer = micros();
    //unsigned long time = micros();
    for(byte i = 0; i<16; i++){
      if(timer % T[i] == 0){
        Motor motor;
        getMotor(i, motor);
        if(currentPos[i] != setPos[i]){
            if(motorSleepStatus[i]){
              awakeMotor(motor);
              motorSleepStatus[i] = false;
            }
            if(setPos[i] > currentPos[i]){
              stepMotor(motor,UP);
              currentPos[i]++;
            }else{
              stepMotor(motor,DOWN);
              currentPos[i]--;
            }
        }else{
          if(!motorSleepStatus[i]){
            sleepMotor(motor);
            motorSleepStatus[i] = true;
          }
        }
      }
    }
    timer++;
    if (timer > 255) timer = 1; 
    //time = micros() - time;
    //Serial.println(time);
    //Serial.flush();
  }
  if(Serial.read() == 0x55){ //takes 640 - 660 us
      //unsigned long stopwatch = micros();
      byte buf[6];
      Serial.readBytes(buf,6);
      if(buf[5] == 0xAA){
        byte CRC = 0;
        for(byte i = 0; i<4; i++){
          CRC ^= buf[i];
        }

        #if defined (TESTING_CONDITION)
        CRC = buf[4];
        #endif

        if(buf[0]==PCB_ADDR){
          digitalWrite(OK_LED, HIGH);
          int8_t angle = buf[2];
          if(CRC == buf[4] && (abs(angle) <= LIMIT) && buf[1] <= 28 && buf[3] != 0){
            // 25,5 deg/s = 153 steps/s
            // 1 step per 0,0065359477124183s ~ 6536us
            // T * 6536us
            double speed = (double) buf[3]/10;
            unsigned int T_tmp = round((1/(speed*6))/0.006536);
            int setPos_tmp = angle * 6; // 2048steps / 360degrees =  5,688888888888889 ~ 6 steps/degree

            switch(buf[1]){ //multi-motor frames support
              case 16: //first row
                setMotor(0, T_tmp,setPos_tmp);
                setMotor(1, T_tmp,setPos_tmp);
                setMotor(2, T_tmp,setPos_tmp);
                setMotor(3, T_tmp,setPos_tmp);
              break;

              case 17: //second row
                setMotor(4, T_tmp,setPos_tmp);
                setMotor(5, T_tmp,setPos_tmp);
                setMotor(6, T_tmp,setPos_tmp);
                setMotor(7, T_tmp,setPos_tmp);
              break;

              case 18: //third row
                setMotor(8, T_tmp,setPos_tmp);
                setMotor(9, T_tmp,setPos_tmp);
                setMotor(10, T_tmp,setPos_tmp);
                setMotor(11, T_tmp,setPos_tmp);
              break;

              case 19: //fourth row
                setMotor(12, T_tmp,setPos_tmp);
                setMotor(13, T_tmp,setPos_tmp);
                setMotor(14, T_tmp,setPos_tmp);
                setMotor(15, T_tmp,setPos_tmp);
              break;

              case 20: // first col
                setMotor(0, T_tmp,setPos_tmp);
                setMotor(4, T_tmp,setPos_tmp);
                setMotor(8, T_tmp,setPos_tmp);
                setMotor(12, T_tmp,setPos_tmp);
              break;

              case 21: // second col
                setMotor(1, T_tmp,setPos_tmp);
                setMotor(5, T_tmp,setPos_tmp);
                setMotor(9, T_tmp,setPos_tmp);
                setMotor(13, T_tmp,setPos_tmp);
              break;

              case 22: // third col
                setMotor(2, T_tmp,setPos_tmp);
                setMotor(6, T_tmp,setPos_tmp);
                setMotor(10, T_tmp,setPos_tmp);
                setMotor(14, T_tmp,setPos_tmp);
              break;

              case 23: // fourth col
                setMotor(3, T_tmp,setPos_tmp);
                setMotor(7, T_tmp,setPos_tmp);
                setMotor(11, T_tmp,setPos_tmp);
                setMotor(15, T_tmp,setPos_tmp);
              break;

              case 24: // first and second row 
                setMotor(0, T_tmp,setPos_tmp);
                setMotor(1, T_tmp,setPos_tmp);
                setMotor(2, T_tmp,setPos_tmp);
                setMotor(3, T_tmp,setPos_tmp);
                setMotor(4, T_tmp,setPos_tmp);
                setMotor(5, T_tmp,setPos_tmp);
                setMotor(6, T_tmp,setPos_tmp);
                setMotor(7, T_tmp,setPos_tmp);
              break; 

              case 25: // third and fourth row
                setMotor(8, T_tmp,setPos_tmp);
                setMotor(9, T_tmp,setPos_tmp);
                setMotor(10, T_tmp,setPos_tmp);
                setMotor(11, T_tmp,setPos_tmp);
                setMotor(12, T_tmp,setPos_tmp);
                setMotor(13, T_tmp,setPos_tmp);
                setMotor(14, T_tmp,setPos_tmp);
                setMotor(15, T_tmp,setPos_tmp);
              break;

              case 26: // first and second col
                setMotor(0, T_tmp,setPos_tmp);
                setMotor(4, T_tmp,setPos_tmp);
                setMotor(8, T_tmp,setPos_tmp);
                setMotor(12, T_tmp,setPos_tmp);
                setMotor(1, T_tmp,setPos_tmp);
                setMotor(5, T_tmp,setPos_tmp);
                setMotor(9, T_tmp,setPos_tmp);
                setMotor(13, T_tmp,setPos_tmp);
              break;

              case 27: // third and fourth col
                setMotor(2, T_tmp,setPos_tmp);
                setMotor(6, T_tmp,setPos_tmp);
                setMotor(10, T_tmp,setPos_tmp);
                setMotor(14, T_tmp,setPos_tmp);
                setMotor(3, T_tmp,setPos_tmp);
                setMotor(7, T_tmp,setPos_tmp);
                setMotor(11, T_tmp,setPos_tmp);
                setMotor(15, T_tmp,setPos_tmp);
              break;

              case 28: //all
                for(byte i = 0; i<16;i++){
                  T[i] = T_tmp;
                  setPos[i] = setPos_tmp;
                }
              break;

              default: //normal address
                setMotor(buf[1], T_tmp,setPos_tmp);
              break;
            }

          }else{
            digitalWrite(ERR_LED, HIGH);
          }
          digitalWrite(OK_LED, LOW);
        }else if(buf[0]==0xFF && buf[1]==0xFF && buf[2]==0x00 && buf[3]==0x00 && CRC == buf[4]){ //Homing frame
          digitalWrite(ERR_LED, LOW);
          digitalWrite(OK_LED, HIGH);
          axisReset();
          timer = 1;
          digitalWrite(OK_LED, LOW);
        }else if(buf[0]==0xF0 && buf[1]==0xFF && buf[2]==0x00 && buf[3]==0x00 && CRC == buf[4]){ //Freeze frame
          for(byte i = 0; i<16; i++){
            setPos[i] = currentPos[i];
          }
        }
      }else{
          digitalWrite(ERR_LED, HIGH);
      }
      //Serial.println(micros()-stopwatch);
    }

}

void setMotor(byte addr, unsigned int T_tmp,int Pos_tmp){
  byte motor = MOTOR_CONNECTION_TAB[addr];
  T[motor] = T_tmp;
  setPos[motor] = Pos_tmp;

  #if defined (TESTING_CONDITION)
  Serial.print(addr);
  Serial.print("->");
  Serial.print(motor);
  Serial.print(":");
  Serial.print(T[motor]);
  Serial.print(" ");
  Serial.println(setPos[motor]);
  #endif
}

void writeReg(byte pin, byte addr, byte value) {
  digitalWrite(pin, LOW);
  SPIwrite(0b01000000);  //opcode
  SPIwrite(addr);
  SPIwrite(value);
  digitalWrite(pin, HIGH);
}

byte readReg(byte pin, byte addr) {
  digitalWrite(pin, LOW);
  SPIwrite(0b01000001);  //opcode
  SPIwrite(addr);
  byte x = SPIread();
  digitalWrite(pin, HIGH);
  return x;
}

void axisReset(){
  memset(currentPos, 0, sizeof(currentPos));
  memset(setPos, 0, sizeof(setPos));

  memset(motorState, motorStart, sizeof(motorState)); 
  memset(motorStateBeforeSleep, motorStart, sizeof(motorStateBeforeSleep)); //just copy for later use

  for (int i = 0; i<sizeof(motorSleepStatus); i++){
    motorSleepStatus[i] = false;
  }
  for (int i = 0; i<sizeof(T)/sizeof(int); i++){
    T[i] = 1;
  }
  for(int i = 0; i < 90 * 6; i++){ //all the way up
    for(byte m = 0; m < 16;m++){
      Motor motor;
      getMotor(m,motor);
      stepMotor(motor, UP);
    }
    delayMicroseconds(6536);
  }

  for(int i = 0; i < MOTOR_CENTER * 6; i++){ //go to center
    for(byte m = 0; m < 16;m++){
      Motor motor;
      getMotor(m,motor);
      stepMotor(motor, DOWN);
    }
    delayMicroseconds(6536);
  }
}

bool getMotor(byte num,Motor &motor){ // num is zero based !!!!
  //unsigned long time = micros();
  if (num > 15) {
    digitalWrite(ERR_LED, HIGH);
    return false;
  }
  motor.expander = num / 4 + 2;
  motor.tabAddr = num / 2;
  byte motorNum = num % 4;
  switch (motorNum) {
    case 0:
      motor.reg = GPIOB;
      motor.shift = 0;
      break;

    case 1:
      motor.reg = GPIOB;
      motor.shift = 4;
      break;

    case 2:
      motor.reg = GPIOA;
      motor.shift = 0;
      break;

    case 3:
      motor.reg = GPIOA;
      motor.shift = 4;
  }
  return true;
}

void stepMotor(Motor motor, directions dir) { 
  //byte tmp = readReg (expander, reg);
  //tmp &= (0b11110000 >> shift);
  byte value = (motorState[motor.tabAddr] >> motor.shift) & 0b00001111;

  #if defined (MOTORS_DIRECTION_REVERSED) 
    const directions binding = UP;
  #else
    const directions binding = DOWN;
  #endif

  if(dir == binding){
    value = (value >> 1) & 0b00001111;
    if (value == 0b00000001) value = 0b00001001;
    if (value == 0b00000100) value = 0b00001100;
  }else{
    value = (value << 1) & 0b00001111;
    if (value == 0b00001000) value = 0b00001001;
    if (value == 0b00000010) value = 0b00000011;
  }

  motorState[motor.tabAddr] &= (0b11110000 >> motor.shift);
  //tmp |= (value << shift);
  motorState[motor.tabAddr] |= (value << motor.shift);
  //writeReg (expander, reg, tmp);
  writeReg(motor.expander, motor.reg, motorState[motor.tabAddr]);
  //time = micros() - time;
  //Serial.println(time);
  //Serial.flush();
}

void sleepMotor(Motor motor) {
  byte value = (motorState[motor.tabAddr] >> motor.shift) & 0b00001111;

  //saving motor value to make sure it wont lose any step
  motorStateBeforeSleep[motor.tabAddr] &= (0b11110000 >> motor.shift);
  motorStateBeforeSleep[motor.tabAddr] |= (value << motor.shift);

  //putting motor to sleep by turning off power on all 4 pins
  motorState[motor.tabAddr] &= (0b11110000 >> motor.shift);
  
  //applying changes
  writeReg(motor.expander, motor.reg, motorState[motor.tabAddr]);
}

void awakeMotor(Motor motor) {
  //get previous motor value
  byte value = (motorStateBeforeSleep[motor.tabAddr] >> motor.shift) & 0b00001111;

  //turning motor back on
  motorState[motor.tabAddr] &= (0b11110000 >> motor.shift);
  motorState[motor.tabAddr] |= (value << motor.shift);

  //applying changes
  writeReg(motor.expander, motor.reg, motorState[motor.tabAddr]);
}
