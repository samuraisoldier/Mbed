#include "mbed.h"
#include "SHA256.h"
#include "rtos.h"

//Photointerrupter input pins
#define I1pin D2
#define I2pin D11
#define I3pin D12

//Incremental encoder input pins
#define CHA   D7
#define CHB   D8  

//Motor Drive output pins   //Mask in output byte
#define L1Lpin D4           //0x01
#define L1Hpin D5           //0x02
#define L2Lpin D3           //0x04
#define L2Hpin D6           //0x08
#define L3Lpin D9           //0x10
#define L3Hpin D10          //0x20

//Mapping from sequential drive states to motor phase outputs
/*
State   L1  L2  L3
0       H   -   L
1       -   H   L
2       L   H   -
3       L   -   H
4       -   L   H
5       H   L   -
6       -   -   -
7       -   -   -
*/

//Drive state to output table
const int8_t driveTable[] = {0x12,0x18,0x09,0x21,0x24,0x06,0x00,0x00};

//Mapping from interrupter inputs to sequential rotor states. 0x00 and 0x07 are not valid
const int8_t stateMap[] = {0x07,0x05,0x03,0x04,0x01,0x00,0x02,0x07};  
//const int8_t stateMap[] = {0x07,0x01,0x03,0x02,0x05,0x00,0x04,0x07}; //Alternative if phase order of input or drive is reversed

//Phase lead to make motor spin
volatile int8_t lead = 2;  //2 for forwards, -2 for backwards


//motor stuff
volatile int8_t orState = 0;
const int16_t us = 2000;
volatile int8_t oldRotorState;
volatile int8_t motorPosition;
volatile uint8_t basepw;

//Decode Variables
volatile uint64_t newKey;
Mutex newKey_mutex;
volatile int16_t newVel;
Mutex newVel_mutex;
volatile uint8_t newRot = 0;
Mutex newRot_mutex;
volatile uint8_t newTor = 0;

//Father Time, his son and daughter
Timer t;
Timer c;
Timer m;

//Status LED
DigitalOut led1(LED1);

//Photointerrupter inputs
InterruptIn I1(I1pin);
InterruptIn I2(I2pin);
InterruptIn I3(I3pin);

//Motor Drive outputs
PwmOut L1L(L1Lpin);
DigitalOut L1H(L1Hpin);
PwmOut L2L(L2Lpin);
DigitalOut L2H(L2Hpin);
PwmOut L3L(L3Lpin);
DigitalOut L3H(L3Hpin);

//Initialise the serial port
RawSerial pc(SERIAL_TX, SERIAL_RX);

//Comm
Thread commOutT(osPriorityNormal,512);
Thread motorCtrlT (osPriorityAboveNormal,2048);
Thread commInT (osPriorityNormal,1024);

//queue for received characters
Queue<void, 8> inCharQ;

//struct for message sending
typedef struct{
    uint8_t code;
    uint32_t data;
    } message_t ;

//instance of mail function to send out messages
Mail<message_t,2> outMessages;

//function to print out message
void putMessage(uint8_t code, uint32_t data){
    message_t *pMessage = outMessages.alloc();
    pMessage->code = code;
    pMessage->data = data;
    outMessages.put(pMessage);
 }


void serialISR(){
    //led1=!led1;
    uint8_t newChar = pc.getc();
    inCharQ.put((void*)newChar);
    //pc.printf("serialISR\n\r");
}


//Set a given drive state
void motorOut(int8_t driveState, uint32_t pw, uint32_t torque){
    //Lookup the output byte from the drive state.
    int8_t driveOut = driveTable[driveState & 0x07];
      
    //Turn off first
    if (~driveOut & 0x01) L1L.pulsewidth_us(0);
    if (~driveOut & 0x02) L1H = 1;
    if (~driveOut & 0x04) L2L.pulsewidth_us(0);
    if (~driveOut & 0x08) L2H = 1;
    if (~driveOut & 0x10) L3L.pulsewidth_us(0);
    if (~driveOut & 0x20) L3H = 1;
    
    //Then turn on
    if (driveOut & 0x01) L1L.pulsewidth_us((us*pw)/100);
    if (driveOut & 0x02) L1H = 0;
    if (driveOut & 0x04) L2L.pulsewidth_us((us*pw)/100);
    if (driveOut & 0x08) L2H = 0;
    if (driveOut & 0x10) L3L.pulsewidth_us((us*pw)/100);
    if (driveOut & 0x20) L3H = 0;
    }
 
 void rotorchange() {
    
    int8_t rotorState = stateMap[I1 + 2*I2 + 4*I3];
    motorOut((rotorState-orState+lead+6)%6,basepw,newTor);
    if (rotorState - oldRotorState == 5) motorPosition--;
    else if (rotorState - oldRotorState == -5) motorPosition++;
    else motorPosition += (rotorState - oldRotorState);
    oldRotorState = rotorState;
    }
 
void motorCtrlTick(){
    motorCtrlT.signal_set(0x1);
    }

void motorCtrlFn(){
    Ticker motorCtrlTicker;
    int16_t vel;
    int8_t oldmotorposition = 0;
    motorCtrlTicker.attach_us(&motorCtrlTick,100000);
    uint32_t counterrr = 0;
    int y = 0;
    int k = 300;
    while(1){
       motorCtrlT.signal_wait(0x1);
       vel = (int16_t)((motorPosition - oldmotorposition)*10)/6;
       oldmotorposition = motorPosition;
       counterrr++;
       if (!(counterrr%10)) putMessage(7 ,(int16_t)(vel));
       newKey_mutex.lock();
       if(newVel==0){
           y = 100;
        }
        /*else if(vel==0){
            motorOut((motorPosition-2), 50, newTor);
        }*/
        else{
            y = k*(newVel-vel);
            if (y<0){
                y = -y;
                lead = -2;
            }
            else{
                lead = 2;
            }
        }
        newKey_mutex.unlock();
        basepw=y; 
        rotorchange();
    }
}
    //Convert photointerrupter inputs to a rotor state
inline int8_t readRotorState(){
    return stateMap[I1 + 2*I2 + 4*I3];
    }

int8_t motorhome(){
    //Run the motor synchronisation
    motorOut(0, 50, newTor);
    wait(1.0);
    return readRotorState();
    //orState is subtracted from future rotor state inputs to align rotor and motor states
}    
 
// Comm function
void commOutFn(){
    while(1) {
    osEvent newEvent = outMessages.get();
    message_t *pMessage = (message_t*)newEvent.value.p;
    if(pMessage->code == 8){
        pc.printf("%x %c \n\r",pMessage->code,pMessage->data);
        outMessages.free(pMessage);
    }
    else if(pMessage->code == 0){
        pc.printf("%x %s \n\r",pMessage->code,pMessage->data);
        outMessages.free(pMessage);
    }
    else if(pMessage->code == 7){
        pc.printf("%x %d \n\r",pMessage->code,pMessage->data);
        outMessages.free(pMessage);
    }          
    else{
        pc.printf("%x %x \n\r",pMessage->code,pMessage->data);
        outMessages.free(pMessage);
    }
    }
}

void threadf(){
    //pc.printf("threadf\n\r");
    uint8_t index = 0;
    char newCmd[32];
    pc.attach(&serialISR);
    while(1) {
    
    osEvent newEvent = inCharQ.get();
    //led1 = !led1;
    uint8_t newChar = (uint8_t)newEvent.value.p;
    
    //led1 = !led1;
    putMessage(8,newChar);
    
    if (newChar == '\r'){
        newCmd[index] = '\0';
        index = 0;
            if (newCmd[0] == 'K'){
                newKey_mutex.lock();
                sscanf(newCmd, "K%x", &newKey); //Decode the command
                newKey_mutex.unlock();
                putMessage(0 ,((uint32_t)("Key Decoded")));
            }
            else if (newCmd[0]=='V'){
                newVel_mutex.lock();
                sscanf(newCmd, "V%x", &newVel);
                newVel_mutex.unlock();
                putMessage(0 ,((uint32_t)("Velocity Decoded")));
            }
            else if (newCmd[0]=='R'){
                newRot_mutex.lock();
                sscanf(newCmd, "R%x", &newRot); //Decode the command
                newRot_mutex.unlock();
                putMessage(0 ,((uint32_t)("Rotations Decoded")));
            }
            else if (newCmd[0] == 'T'){
                sscanf(newCmd, "T%x", &newTor); //Decode the command
                putMessage(0 ,((uint32_t)("Torque Decoded")));
            }
        } 
        else {
            led1 = !led1;
            newCmd[index] = newChar;
            index++;
        }
    }
}

//Main
int main(){
    //All comm stuff gon go here
    commOutT.start(commOutFn);
    motorCtrlT.start(motorCtrlFn);
    commInT.start(threadf);
    L1L.period_us(us);
    L2L.period_us(us);
    L3L.period_us(us);
    
    m.start();
    
    putMessage(0 ,((uint32_t)("Hello\n\r")));
    
    orState = motorhome();
//    putMessage(1 ,("Rotor origin: %x\n\r",orState));
    
    //autostart
    motorOut(1,50,newTor);    
    
    I1.rise(rotorchange);
    I2.rise(rotorchange);
    I3.rise(rotorchange);
    I1.fall(rotorchange);
    I2.fall(rotorchange);
    I3.fall(rotorchange);
        
    //bitcoin stuff starts here
    
    uint8_t sequence[] = {0x45,0x6D,0x62,0x65,0x64,0x64,0x65,0x64,
    0x20,0x53,0x79,0x73,0x74,0x65,0x6D,0x73,
    0x20,0x61,0x72,0x65,0x20,0x66,0x75,0x6E,
    0x20,0x61,0x6E,0x64,0x20,0x64,0x6F,0x20,
    0x61,0x77,0x65,0x73,0x6F,0x6D,0x65,0x20,
    0x74,0x68,0x69,0x6E,0x67,0x73,0x21,0x20,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
    volatile uint64_t* key = (uint64_t*)((int)sequence + 48);
    uint64_t* nonce = (uint64_t*)((int)sequence + 56);
    uint8_t hash[32];
    uint8_t count =0;
    uint8_t hashrate =0;
    t.start();
    c.start();
    
    newKey_mutex.lock();
    if (!(&newKey==key)){
        key = &newKey;
    }
    newKey_mutex.unlock();

        
    for (uint64_t ij=0; ij<=0xFFFFFFFFFFFFFFFF; ij++){
        *nonce = ij;
        SHA256::computeHash(hash, sequence, 64);
        if (!(hash[0] or hash[1])){
            count++;
        //  putMessage(hashf ,((uint32_t)("Successful Hash found: ", hash)));
        //    putMessage(0 ,((uint32_t)("Successful Hash found, nonce: ")));
            putMessage(2 ,((uint32_t)(*nonce)));
        }
        if ((t.read()>=1)){
        //    putMessage(hashr8 ,((uint32_t)("Hash rate is ", hashrate)));
            t.reset();
        }
        if (count == 3){
            hashrate = 1000*(count/c.read());
            count = 0;
            c.reset();
        }
    }               
    //pc.printf("Loop over yo idk what to do now so I'm terminating gbye");
}
