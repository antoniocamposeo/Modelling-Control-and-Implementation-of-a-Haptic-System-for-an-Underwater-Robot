/*----------------------------
 * Final Version of OpenCM904*
-----------------------------*/
/*Import Libraries*/
#include "ros.h"
#include "./std_msgs/Int32MultiArray.h"
#include "./std_msgs/String.h"
#include <Dynamixel2Arduino.h>


#define timerOnePeriod 13000 // Set Timer Period in microsec
//#define timerOnePeriod 500 

/* Motor Variables and Parameters */
#define DXL_SERIAL   Serial3 //OpenCM9.04 EXP Board's DXL port Serial. (Serial1 for the DXL port on the OpenCM 9.04 board)
#define DEBUG_SERIAL Serial
const int DXL_DIR_PIN = 22; //OpenCM9.04 EXP Board's DIR PIN. (28 for the DXL port on the OpenCM 9.04 board)
const int OFFSET_STR = 1000;
const uint8_t BROADCAST_ID = 254;
const float DYNAMIXEL_PROTOCOL_VERSION = 2.0;
const uint8_t DXL_ID_CNT = 2;
const uint8_t DXL_ID_LIST[DXL_ID_CNT] = {1, 2};
const uint16_t user_pkt_buf_cap = 128;
uint8_t user_pkt_buf[user_pkt_buf_cap];
// Starting address of the Data to read; Present Position = 132
const uint16_t SR_START_ADDR = 132;
// Length of the Data to read; Length of Position data of X series is 4 byte
const uint16_t SR_ADDR_LEN = 4;
// Starting address of the Data to write; Goal Position = 116
const uint16_t SW_START_ADDR = 116;
// Length of the Data to write; Length of Position data of X series is 4 byte
const uint16_t SW_ADDR_LEN = 4;
// Variable to count how many motor are connected
uint8_t recv_cnt;
/* Struct to store motors data*/
typedef struct sr_data{
  int32_t present_position;
} __attribute__((packed)) sr_data_t;

typedef struct sw_data{
  int32_t goal_position;
} __attribute__((packed)) sw_data_t;

sr_data_t sr_data[DXL_ID_CNT];
DYNAMIXEL::InfoSyncReadInst_t sr_infos;
DYNAMIXEL::XELInfoSyncRead_t info_xels_sr[DXL_ID_CNT];

sw_data_t sw_data[DXL_ID_CNT];
DYNAMIXEL::InfoSyncWriteInst_t sw_infos;
DYNAMIXEL::XELInfoSyncWrite_t info_xels_sw[DXL_ID_CNT];


HardwareTimer Timer(1); // Defintion of Hardware Timer 
Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN); // Definition of Motor Connection 

//This namespace is required to use DYNAMIXEL Control table item name definitions
using namespace ControlTableItem;

//Pid Parameter Setting
uint16_t position_p_gain = 900;
uint16_t position_i_gain = 10;
uint16_t position_d_gain = 0;


unsigned long startMillis;  //some global variables available anywhere in the program
unsigned long currentMillis;
const unsigned long period = 50000;  //the value is a number of milliseconds


int ResetPin = 10; // Set DigitalPin to reset the board. 


/* ROSSerial Variables and Parameters */
// Define Ros Object 
ros::NodeHandle  nh;
// Boolean variable to check if the ros callback is active or not
bool callback_active = false;  
// Define variable in for-loop
uint8_t i = 0; 
// Define an array of chars, it is used to store data of sensor FT from arduino via UART
char test[37]; 
// Define an array of chars, it is used to store data of sensors, motor and status. And publish it with ros 
String value_ft; // ArrayString = "CHOCH1CH2CH3CH4CH5M1M2S" each CHi has 6 char, M1|M2 have 4 char and S has 1 char
// Define an array of chars, it is used to store data of sensors, motor and status. And publish it with ros 
int32_t value_position[2] = {0, 0}; // Array = [actualMotor1,actualMotor2] 
// Define a boolean variable to check the message in input frrom ros and reset the board
uint8_t reset_input = 0; 

// Set a Single Message to be publish , it is a String Message  
std_msgs::String data_msg;
// Define a publisher 
ros::Publisher data_opencm("data_opencm", &data_msg);

// Callback Function to Subscribe to a topic and get data
void messageCb( const std_msgs::Int32MultiArray& msg) {
  
    // Set callback active  true 
    callback_active = true;
    // Store motor reference position
    for( i = 0; i < DXL_ID_CNT; i++){
        // Insert a new Goal Position to the SyncWrite Packet
        sw_data[i].goal_position = msg.data[i];
     } 
    // Check is the board will be reset or not 
    if (msg.data[2] == 0){reset_input = 0;}
    else if(msg.data[2] == 1){reset_input = 1;}
    
    // Change the info of motors struct   
    sw_infos.is_info_changed = true;
    // Send data to the motors
    dxl.syncWrite(&sw_infos); 
    // Set Boolean to false 
    callback_active = false;
}

// Define a Subriber to get motor position reference from topic /motor_position
ros::Subscriber <std_msgs::Int32MultiArray> sub("motor_position", &messageCb );

// Setup Function
void setup(){
    // Init Timer 
    Timer.pause();
    Timer.setPeriod(timerOnePeriod);
    // Attach interrupt function to timer 
    Timer.attachInterrupt(handler_data_sender);
    // Set baudrate serial connection
    nh.getHardware()->setBaud(115200);
    //Init connection uart arduino 
    Serial2.begin(115200); 
    // Init ros node
    nh.initNode(); 
    // Init motor connection
    init_motors();  
    /* Set publishers */
    nh.advertise(data_opencm);
    /* Set subscribers*/    
    nh.subscribe(sub); 
    
    // Restarting Timer 
    Timer.refresh();
    Timer.resume();
    
    //startMillis = millis();
    
    // Set digital pin to 5V
    digitalWrite(ResetPin, HIGH); 
    // Set the digital pin to an OUTPUT pin
    pinMode(ResetPin, OUTPUT); 
}


void loop(){
} 

void init_motors(){
    
    dxl.begin(115200);
    dxl.setPortProtocolVersion(DYNAMIXEL_PROTOCOL_VERSION);
    // Prepare the SyncRead structure
    for(i = 0; i < DXL_ID_CNT; i++){
        dxl.torqueOff(DXL_ID_LIST[i]);
        dxl.setOperatingMode(DXL_ID_LIST[i], OP_POSITION);
        dxl.writeControlTableItem(POSITION_P_GAIN, DXL_ID_LIST[i], position_p_gain);
        dxl.writeControlTableItem(POSITION_I_GAIN, DXL_ID_LIST[i], position_i_gain);
        dxl.writeControlTableItem(POSITION_D_GAIN, DXL_ID_LIST[i], position_d_gain);
    }
    dxl.torqueOn(BROADCAST_ID);


    // Fill the members of structure to syncRead using external user packet buffer
    sr_infos.packet.p_buf = user_pkt_buf;
    sr_infos.packet.buf_capacity = user_pkt_buf_cap;
    sr_infos.packet.is_completed = false;
    sr_infos.addr = SR_START_ADDR;
    sr_infos.addr_length = SR_ADDR_LEN;
    sr_infos.p_xels = info_xels_sr;
    sr_infos.xel_count = 0;  

    for(i = 0; i < DXL_ID_CNT; i++){
        info_xels_sr[i].id = DXL_ID_LIST[i];
        info_xels_sr[i].p_recv_buf = (uint8_t*)&sr_data[i];
        sr_infos.xel_count++;
    }
    sr_infos.is_info_changed = true;

    // Fill the members of structure to syncWrite using internal packet buffer
    sw_infos.packet.p_buf = nullptr;
    sw_infos.packet.is_completed = false;
    sw_infos.addr = SW_START_ADDR;
    sw_infos.addr_length = SW_ADDR_LEN;
    sw_infos.p_xels = info_xels_sw;
    sw_infos.xel_count = 0;

    for(i = 0; i < DXL_ID_CNT; i++){
        info_xels_sw[i].id = DXL_ID_LIST[i];
        info_xels_sw[i].p_data = (uint8_t*)&sw_data[i].goal_position;
        sw_infos.xel_count++;
    }
    sw_infos.is_info_changed = true;
}


   
void handler_data_sender(void) {    
  // Check reset input and reset the board 
  if (reset_input == 1) 
  {
   digitalWrite(ResetPin, LOW);
  }else if (reset_input == 0)
  {
    digitalWrite(ResetPin, HIGH);
    }
  // Check ros connection if available go in the condition
  if (nh.connected())
    {
  // Check if the ros callback is active
      if(callback_active == false)
      {
        // Send a msg to arduino to get the data;
        Serial2.write('0');
        // Read the array of char from arduino 
        Serial2.readBytesUntil('\n', test, sizeof(test)); 
        Serial2.flush();  
        // Get the number of motor available and connected
        recv_cnt = dxl.syncRead(&sr_infos);   
        if(recv_cnt > 0) {
           for( i = 0; i < DXL_ID_CNT; i++){
            // Get position of motors
            value_position[i] = sr_data[i].present_position;
            } 
            }
            // Create the string to send to the topic               
            data_msg.data = (test + String(value_position[0]+OFFSET_STR)+String(value_position[1]+OFFSET_STR) + String(reset_input)).c_str();
            data_opencm.publish( &data_msg);
        }
       }
  nh.spinOnce(); // Ros function to update the callback function
}
