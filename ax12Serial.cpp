#include "ax12Serial.hh"
#include <iostream>

#include "dynamixel_sdk.h"                                  // Uses Dynamixel SDK library
#include "unistd.h"


using namespace std;

string getAx12RegName(int reg)
{
static vector<string> ax12RegIdToName = {
"AX_MODEL_NUMBER_L", //0
"AX_MODEL_NUMBER_H", //1
"AX_VERSION", //2
"AX_ID", //3
"AX_BAUD_RATE", //4
"AX_RETURN_DELAY_TIME", //5
"AX_CW_ANGLE_LIMIT_L", //6
"AX_CW_ANGLE_LIMIT_H", //7
"AX_CCW_ANGLE_LIMIT_L", //8
"AX_CCW_ANGLE_LIMIT_H", //9
"AX_SYSTEM_DATA2", //10
"AX_LIMIT_TEMPERATURE", //11
"AX_DOWN_LIMIT_VOLTAGE", //12
"AX_UP_LIMIT_VOLTAGE", //13
"AX_MAX_TORQUE_L", //14
"AX_MAX_TORQUE_H", //15
"AX_RETURN_LEVEL", //16
"AX_ALARM_LED", //17
"AX_ALARM_SHUTDOWN", //18
"AX_OPERATING_MODE", //19
"AX_DOWN_CALIBRATION_L", //20
"AX_DOWN_CALIBRATION_H", //21
"AX_UP_CALIBRATION_L", //22
"AX_UP_CALIBRATION_H", //23
/**", //RAM AREA **/
"AX_TORQUE_ENABLE", //24
"AX_LED", //25
"AX_CW_COMPLIANCE_MARGIN", //26
"AX_CCW_COMPLIANCE_MARGIN", //27
"AX_CW_COMPLIANCE_SLOPE", //28
"AX_CCW_COMPLIANCE_SLOPE", //29
"AX_GOAL_POSITION_L", //30
"AX_GOAL_POSITION_H", //31
"AX_GOAL_SPEED_L", //32
"AX_GOAL_SPEED_H", //33
"AX_TORQUE_LIMIT_L", //34
"AX_TORQUE_LIMIT_H", //35
"AX_PRESENT_POSITION_L", //36
"AX_PRESENT_POSITION_H", //37
"AX_PRESENT_SPEED_L", //38
"AX_PRESENT_SPEED_H", //39
"AX_PRESENT_LOAD_L", //40
"AX_PRESENT_LOAD_H", //41
"AX_PRESENT_VOLTAGE", //42
"AX_PRESENT_TEMPERATURE", //43
"AX_REGISTERED_INSTRUCTION", //44
"AX_PAUSE_TIME", //45
"AX_MOVING", //46
"AX_LOCK", //47
"AX_PUNCH_L", //48
"AX_PUNCH_H" //49
};
  if (reg < ax12RegIdToName.size())
    return ax12RegIdToName[reg];
  return "unknown";
}
string getAx12RegWithName(int reg)
{
   return std::to_string(reg) + "(" + getAx12RegName(reg) + ")";
}



dynamixel::PortHandler *portHandler = 0;
dynamixel::PacketHandler *packetHandler = 0;

void ax12Init(long baud)
{
  if (portHandler && packetHandler) return; // don't initialise twice

#define BAUDRATE                        1000000
#define DEVICENAME                      "/dev/ttyUSB0"      // Check which port is being used on your controller
                                                            // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"
// Protocol version
#define PROTOCOL_VERSION                1.0                 // See which protocol version is used in the Dynamixel

  // Initialize PortHandler Structs
  // Set the port path
  // Get methods and members of PortHandlerLinux or PortHandlerWindows
  portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

  // Initialize PacketHandler instance
  // Set the protocol version
  // Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
  packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
  usleep(500000);

  int index = 0;
  int dxl_comm_result = COMM_TX_FAIL;             // Communication result
 // int dxl_goal_position[2] = {DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE};         // Goal position

  uint8_t dxl_error = 0;                          // Dynamixel error
  uint16_t dxl_present_position = 0;              // Present position

  // Open port
  if (portHandler->openPort())
  {
    printf("Succeeded to open the port!\n");
  }
  else
  {
    printf("Failed to open the port!\n");
    printf("Terminating...\n");
    exit(1);
  }

  // Set port baudrate
  if (portHandler->setBaudRate(BAUDRATE))
  {
    printf("Succeeded to change the baudrate!\n");
  }
  else
  {
    printf("Failed to change the baudrate!\n");
    printf("Terminating...\n");
    exit(1);
  }
  usleep(500000);


    // Read all servo positions
    for (int servoId=1; servoId<=18; servoId++)
    {
      dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, servoId, AX_PRESENT_POSITION_L, &dxl_present_position, &dxl_error);
      if (dxl_comm_result != COMM_SUCCESS)
      {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
      }
      else if (dxl_error != 0)
      {
        printf("%s\n", packetHandler->getRxPacketError(dxl_error));
      }

      printf("[ID:%03d] Pos:%03d\n", servoId, dxl_present_position);
    }


  // Debugging mode: make servos slow and weak
  for (int servoId=1; servoId<=18; servoId++)
  {
    #define ADDR_SET_MOVING_SPEED 32
    #define ADDR_LED 25
    #define ADDR_TORQUE_LIMIT 34
    ax12SetRegister( servoId, ADDR_LED, 1 );
    ax12SetRegister( servoId, ADDR_SET_MOVING_SPEED, 200, 2 );
    ax12SetRegister( servoId, ADDR_TORQUE_LIMIT, 900, 2 );
  }
  void setAllPunch(int val);
  setAllPunch(4);
}

void setAllPunch(int val)
{
   for (int servoId=1; servoId<=18; servoId++)
  { 
    #define ADDR_PUNCH 48
    ax12SetRegister( servoId, ADDR_PUNCH, val, 2 );
  }
}


void ax12Finish()
{
  std::cout << "ax12Finish - closing ax12 port" << std::endl;

  // Close port
  portHandler->closePort();

  portHandler = 0;
  packetHandler = 0;
}


void setTXall() {
  cout << "setTXall" << endl;
}     // for sync write
void setTX(int id) {
  cout << "setTX " << id << endl;
}
void setRX(int id) {
  cout << "setRX " << id << endl;
}

void ax12write(unsigned char data) {
  cout << "ax12write " << getAx12RegWithName((int)data) << endl;
}
void ax12write(unsigned char *pdata, int length) {
  cout << "ax12write.2 " << endl;
}
void ax12writeB(unsigned char data) {
  cout << "ax12writeB " << endl;
}

int ax12ReadPacket(int length) { 
  cout << "ax12ReadPacket " << length << endl;
  return 0;
}
int ax12GetRegister(int servoId, int regstart, int length) { 
 cout << "ax12GetRegister servoId=" << servoId << " regstart=" << getAx12RegWithName(regstart) << " length=" << length << endl;

 int retries = 0;
 int val;

 for (;;) {
  if (retries == 5) { cout << "Aborting" << endl; exit(1); }
  if (retries++ > 0) { cout << "Retrying" << endl; }

  int dxl_comm_result;
  uint8_t dxl_error;

  switch (length) {
   case 1:
    {
      uint8_t val1;
      dxl_comm_result = packetHandler->read1ByteTxRx(portHandler, servoId, regstart, &val1, &dxl_error);
      val = val1;
    }
    break;
   case 2:
    {
      uint16_t val2;
      dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, servoId, regstart, &val2, &dxl_error);
      val = val2;
    }
    break;
   default:
    cout << "TODO: length>2" << endl;
    exit(1);
  }
  if (dxl_comm_result != COMM_SUCCESS)
  {
    printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    //val = (1<<(length*8))-1; // 255 or 65535
    continue; // retry
  }
  else if (dxl_error != 0)
  {
    printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    //val = (1<<(length*8))-1; // 255 or 65535
    continue; // retry
  }
  else
    break;
 }
 printf(" => [ID:%03d] %03d:%03d\n", servoId, regstart, val);
 return val;
}


void ax12GroupSyncWriteDetailed(uint8_t startAddr, uint8_t length, uint8_t bVals[], const uint8_t servoIds[], unsigned int NUM_SERVOS)
{
 // Initialize GroupSyncWrite instance
  dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, startAddr, length);
  
  // Add each servo id and value
  for (unsigned int i=0; i<NUM_SERVOS; ++i) {
    uint8_t servoId = servoIds[i];
    bool dxl_addparam_result = groupSyncWrite.addParam(servoId, &bVals[i*length]);
    if (dxl_addparam_result != true)
    { 
      fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", servoId);
      throw 1;
    }
  }
  
  // Syncwrite goal position
  int dxl_comm_result = groupSyncWrite.txPacket();
  if (dxl_comm_result != COMM_SUCCESS) printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
  
  // Clear syncwrite parameter storage
  groupSyncWrite.clearParam();
}

void ax12GroupSyncWrite(uint8_t bReg, uint8_t bVal, const uint8_t cPinTable[], unsigned int NUM_SERVOS)
{
  // Initialize GroupSyncWrite instance
  int len = 1;
  dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, bReg, len);

  // Add each servo id and value
  for (unsigned int i=0; i<NUM_SERVOS; ++i) {
    uint8_t servoId = cPinTable[i];
    bool dxl_addparam_result = groupSyncWrite.addParam(servoId, &bVal);
    if (dxl_addparam_result != true)
    {
      fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", servoId);
      throw 1;
    }
  }

  // Syncwrite goal position
  int dxl_comm_result = groupSyncWrite.txPacket();
  if (dxl_comm_result != COMM_SUCCESS) printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));

  // Clear syncwrite parameter storage
  groupSyncWrite.clearParam();
}


void ax12SetRegister(int servoId, int regstart, int data, int length) {
  cout << "ax12SetRegister servoId=" << servoId << " regstart=" << getAx12RegWithName(regstart) << " data=" << data << " length=" << length << endl;

 int retries = 0;
 int val;

 for (;;) {
  if (retries == 5) { cout << "Aborting" << endl; exit(1); }
  if (retries++ > 0) { cout << "Retrying" << endl; }

  int dxl_comm_result;
  uint8_t dxl_error;

  switch (length) {
   case 1:
    {
      dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, servoId, regstart, data, &dxl_error);
    }
    break;
   case 2:
    {
      dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, servoId, regstart, data, &dxl_error);
    }
    break;
   default:
    cout << "TODO: length>2" << endl;
    exit(1);
  }
  if (dxl_comm_result != COMM_SUCCESS)
  {
    printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    continue; // retry
  }
  else if (dxl_error != 0)
  {
    printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    continue; // retry
  }
  else
    break;
 }
}
int ax12GetLastError() {
  cout << "ax12GetLastError" << endl;
  return 0;
}

unsigned char ax_rx_buffer[AX12_BUFFER_SIZE];
unsigned char ax_tx_buffer[AX12_BUFFER_SIZE];
unsigned char ax_rx_int_buffer[AX12_BUFFER_SIZE];
#if defined(AX_RX_SWITCHED)
// Need to stow type of servo (which bus it's on)
unsigned char dynamixel_bus_config[AX12_MAX_SERVOS];
#endif

