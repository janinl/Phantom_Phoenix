#include "ax12Serial.hh"
#include <iostream>

using namespace std;

vector<string> ax12RegIdToName = {
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

string getAx12RegName(int reg)
{
  if (reg < ax12RegIdToName.size())
    return ax12RegIdToName[reg];
  return "unknown";
}
string getAx12RegWithName(int reg)
{
   return std::to_string(reg) + "(" + getAx12RegName(reg) + ")";
}

void ax12Init(long baud) {}

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
int ax12GetRegister(int id, int regstart, int length) { 
  cout << "ax12GetRegister id=" << id << " regstart=" << getAx12RegWithName(regstart) << " length=" << length << endl;
  return 0;
}
void ax12SetRegister(int id, int regstart, int data) {
  cout << "ax12SetRegister id=" << id << " regstart=" << getAx12RegWithName(regstart) << " data=" << data << endl;
}
void ax12SetRegister2(int id, int regstart, int data) {
  cout << "ax12SetRegister2 id=" << id << " regstart=" << getAx12RegWithName(regstart) << " data=" << data << endl;
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

