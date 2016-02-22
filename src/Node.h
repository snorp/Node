#ifndef Node_h__
#define Node_h__

#include <RFM69_WL.h>
#include <SPIFlash.h>
#include <LowPower.h>
#include "Message.h"

#define GATEWAY_ADDRESS 1
#define DEFAULT_ADDRESS 10
#define DEFAULT_NETWORK_ID 1
#define DEFAULT_NODE_SETTINGS_ADDRESS 256
#define DEFAULT_GROUP 1
#define MAX_NODES 254
#define MAX_MESSAGE_LISTENERS 8

enum DeviceClass {
  kDeviceUnknown,
  kTiltBlinds // Battery powered blinds control
};

class Node {
public:
  typedef void (* MessageListener)(Message& message);

  Node() : _flash(8, 0xEF30) {}

  void setup(int defaultAddress = DEFAULT_ADDRESS, int networkId = DEFAULT_NETWORK_ID,
             uint32_t settingsAddress = DEFAULT_NODE_SETTINGS_ADDRESS, const char* encryptKey = NULL,
             bool hasFlash = true,
             DeviceClass deviceClass = kDeviceUnknown);

  uint8_t address(void) {
    return _settings.address;
  }

  void setAddress(uint8_t address);

  uint8_t group(void) {
    return _settings.group;
  }
  void setGroup(uint8_t group);

  DeviceClass deviceClass(void) {
    return _deviceClass;
  }

  void setDeviceClass(DeviceClass dc) {
    _deviceClass = dc;
  }

  void setBatteryParams(uint8_t pin, int maxVoltage, int minVoltage, float ratio);
  uint8_t readBatteryLevel(void);

  RFM69_WL& radio(void) {
    return _radio;
  }

  void clearSettings(void);
  void reboot(void);

  bool receiveMessage(Message& msg);
  bool sendMessage(Message& msg);
  bool sendMessage(uint8_t to, CommandOp op, uint16_t arg1 = 0, uint16_t arg2 = 0, uint8_t flags = 0);
  void sendBroadcast(CommandOp op, uint16_t arg1 = 0, uint16_t arg2 = 0, uint8_t group = 0, uint8_t flags = 0);

  void bump(void);
  bool needSleep(void);

  // Do nothing for 15ms, but don't waste power
  void pause(void);

  // Returns true if we received a wakeup msg, false otherwise
  void sleep(period_t period = SLEEP_FOREVER);

  void onMessage(MessageListener listener);
  void removeMessageListener(MessageListener listener);

  void run(void);

  void sendBatteryLevel(uint8_t to);
  void sendGroupNumber(uint8_t to, bool reply = false);
  void sendDeviceClass(uint8_t to, bool reply = false);
  void sendInfo(uint8_t to);

  struct NodeSettings {
    uint32_t magic;
    uint8_t address;
    uint8_t group;
  };
protected:
  bool readSettings(void);
  void writeSettings(void);

  bool isMessageForMe(Message& msg);
  void handleMessage(Message& msg);
  void dispatchMessage(Message& msg);

  bool receiveFlashImage(uint16_t length, uint16_t crc);

  void unbump(void);

  DeviceClass _deviceClass;
  uint32_t _settingsAddress;

  int _batteryPin;
  int _maxBatteryVoltage;
  int _minBatteryVoltage;
  float _batterySenseRatio;

  // Only used by nodes that wake other nodes (usually the gateway)
  bool _wakeStates[MAX_NODES];
  MessageListener _listeners[MAX_MESSAGE_LISTENERS];
  uint8_t _sleepNotifyAddress;
  RFM69_WL _radio;
  SPIFlash _flash;
  uint8_t _bumps;
  long _wokeMillis;
  bool _addressChanged;
  NodeSettings _settings;
};

#endif
