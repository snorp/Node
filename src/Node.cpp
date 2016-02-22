#include <avr/wdt.h>
#include <util/crc16.h>
#include <Arduino.h>
#include <EEPROMex.h>
#include "Node.h"

// Uncomment the next line to enable debug logging
// If enabled on the gateway, however, it will interfere
// with the communication with the host
// #define NODE_DEBUG
#include "Debug.h"

#define LED_PIN 9

#define WAKE_DURATION 5000 // 5s

#define REPLY_TIMEOUT 1200
#define NUM_RETRIES 5

#define LISTEN_RX_DURATION 256
#define LISTEN_IDLE_DURATION 1000000

#define BATTERY_MAX_V 68
#define BATTERY_MIN_V 58

#define FLASH_BLOCK_SIZE 32 * 1024

#define SETTINGS_MAGIC 0xaf249e

static Node::NodeSettings defaultSettings = {
  SETTINGS_MAGIC,
  DEFAULT_ADDRESS,
  DEFAULT_GROUP
};

static long readVcc() {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    ADMUX = _BV(MUX5) | _BV(MUX0);
  #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    ADMUX = _BV(MUX3) | _BV(MUX2);
  #else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif

  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring

  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH
  uint8_t high = ADCH; // unlocks both

  long result = (high<<8) | low;

  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result; // Vcc in millivolts
}

static void toggleLED(void)
{
  static byte status = LOW;

  pinMode(LED_PIN, OUTPUT);
  status = status == LOW ? HIGH : LOW;
  digitalWrite(LED_PIN, status);
}

#define IMAGE_HEADER_LENGTH 10 // FLXIMG:NN:

bool Node::receiveFlashImage(uint16_t length, uint16_t expectedCrc)
{
  uint16_t bytesReceived = 0;

  DEBUG("Receiving flash image of %u bytes and CRC %u", length, expectedCrc);

  _flash.wakeup();

  // Erase the first 32k and write the image header
  _flash.blockErase32K(0);
  _flash.writeBytes(0, "FLXIMG:", 7);
  _flash.writeBytes(7, &length, 2);
  _flash.writeByte(9, ':');

  long lastChunkMillis = millis();
  while (bytesReceived < length && (millis() - lastChunkMillis) < 3000) {
    if (!_radio.receiveDone()) {
      continue;
    }

    noInterrupts();
    if (bytesReceived == 32768) {
      _flash.blockErase32K(bytesReceived);
    }

    _flash.writeBytes(IMAGE_HEADER_LENGTH + bytesReceived, (const void*)_radio.DATA, _radio.DATALEN);
    bytesReceived += _radio.DATALEN;
    interrupts();

    _radio.sendACK();
    toggleLED();
    lastChunkMillis = millis();
  }

  digitalWrite(LED_PIN, LOW);

  if (bytesReceived != length) {
    DEBUG("Flashing failed, only received %u of %u bytes", bytesReceived, length);
    return false;
  }

  uint16_t computedCrc = 0xffff;
  for (uint32_t i = 0; i < bytesReceived; i++) {
    computedCrc = _crc16_update(computedCrc, (uint8_t)_flash.readByte(i + IMAGE_HEADER_LENGTH));
  }

  bool success = computedCrc == expectedCrc;
  if (success) {
    // Leave the flash awake
    DEBUG("CRC matched!");
  } else {
    _flash.sleep();
    DEBUG("CRC check failed! Got %u, expected %u", computedCrc, expectedCrc);
  }

  return success;
}

uint8_t Node::readBatteryLevel(void)
{
  if (_batteryPin == 0xff) {
    // No sense pin set
    return 0;
  }

  long vcc = readVcc();

  long voltage = (analogRead(_batteryPin) / 1023.0) * vcc;
  voltage *= _batterySenseRatio;
  return constrain(map(voltage, _minBatteryVoltage, _maxBatteryVoltage, 0, 100), 0, 100);
}

void Node::sendBatteryLevel(uint8_t to)
{
  if (!sendMessage(to, CMD_OP_BATTERY_LEVEL, readBatteryLevel())) {
    DEBUG("Failed to send battery level");
  }
}

void Node::sendGroupNumber(uint8_t to, bool reply)
{
  if (!sendMessage(to, CMD_OP_GROUP, _settings.group, 0, reply ? CMD_FLAG_REPLY : 0)) {
    DEBUG("Failed to send group number");
  }
}

void Node::sendDeviceClass(uint8_t to, bool reply)
{
  if (!sendMessage(to, CMD_OP_CLASS, static_cast<uint8_t>(_deviceClass), 0, reply ? CMD_FLAG_REPLY : 0)) {
    DEBUG("Failed to send device class");
  }
}

void Node::sendInfo(uint8_t to)
{
  sendDeviceClass(to);
  sendGroupNumber(to);
  sendBatteryLevel(to);
}

bool Node::readSettings(void) {
  EEPROM.readBlock<NodeSettings>(_settingsAddress, _settings);
  if (_settings.magic != SETTINGS_MAGIC) {
    _settings = defaultSettings;
    return false;
  }
  return true;
}

void Node::writeSettings(void) {
  EEPROM.updateBlock<NodeSettings>(_settingsAddress, _settings);
}

void Node::setup(int defaultAddress, int networkId, uint32_t settingsAddress,
                 const char* encryptKey, bool hasFlash, DeviceClass deviceClass)
{
  _deviceClass = deviceClass;
  _settingsAddress = settingsAddress;
  _batteryPin = 0xff;

  DEBUG("Build: " __DATE__ ", " __TIME__);
  if (hasFlash) {
    DEBUG("Initializing flash...");
    if (_flash.initialize()) {
      _flash.sleep();
    }
  }

  if (!readSettings()) {
    _settings.address = defaultAddress;
    writeSettings();
  }

  memset(_wakeStates, 0, MAX_NODES * sizeof(bool));
  memset(_listeners, 0, MAX_MESSAGE_LISTENERS * sizeof(MessageListener));

  _sleepNotifyAddress = 0;

  _radio.initialize(RF69_433MHZ, address(), networkId);
  _radio.encrypt(encryptKey);

  uint32_t rxDuration = LISTEN_RX_DURATION;
  uint32_t idleDuration = LISTEN_IDLE_DURATION;
  _radio.setListenDurations(rxDuration, idleDuration);

  DEBUG("RX duration: %luus, idle duration: %luus", rxDuration, idleDuration);

  _wokeMillis = 0;
  _addressChanged = false;
}

void Node::setBatteryParams(uint8_t pin, int maxVoltage, int minVoltage, float ratio)
{
  _batteryPin = pin;
  pinMode(_batteryPin, INPUT);

  _maxBatteryVoltage = maxVoltage;
  _minBatteryVoltage = minVoltage;
  _batterySenseRatio = ratio;
  DEBUG("Battery level is %u", readBatteryLevel());
}

bool Node::isMessageForMe(Message& msg)
{
  return msg.to == address() ||
    (msg.isBroadcast() && (msg.cmd.group == group() || msg.cmd.group == 0));
}

void Node::handleMessage(Message& msg)
{
  if (msg.cmd.flags & CMD_FLAG_WILL_SLEEP) {
    DEBUG("Node %u will sleep", msg.from);

    // The sender is going to sleep immediately, so mark it as such
    _wakeStates[msg.from] = false;
  }

  if (msg.cmd.flags & CMD_FLAG_STAY_AWAKE) {
    DEBUG("Node %u requested we stay awake", msg.from);

    // This node asked us to stay awake, so let them know when we aren't anymore
    _sleepNotifyAddress = msg.from;
    bump();
  }

  // We are receiving a message, so unbump
  unbump();

  switch (msg.cmd.op) {
    case CMD_OP_PING:
      DEBUG("Received ping from %u", msg.from);
      break;
    case CMD_OP_SLEEPING:
      _wakeStates[msg.from] = false;
      break;
    case CMD_OP_WOKE:
      _wakeStates[msg.from] = true;
      break;
    default:
      break;
  }

  if (address() == GATEWAY_ADDRESS) {
    return;
  }

  // These only apply to non-gateway nodes
  switch (msg.cmd.op) {
    case CMD_OP_GROUP:
      if (msg.cmd.arg1 > 0) {
        setGroup(msg.cmd.arg1);
        writeSettings();
        DEBUG("Now member of group %u", group());
      } else {
        sendGroupNumber(msg.from, true);
      }
      break;
    case CMD_OP_CLASS:
      // You cannot change the device class remotely, so this is only used to request the value
      sendDeviceClass(msg.from, true);
      break;
    case CMD_OP_REASSIGN:
      setAddress(msg.cmd.arg1);
      DEBUG("Reassigned to address %u", address());
      break;
    case CMD_OP_RESET:
      DEBUG("Performing factory reset!");
      clearSettings();
      reboot();
      break;
    case CMD_OP_FLASH_IMAGE: {
      DEBUG("Beginning flash session!");
      bool success = receiveFlashImage(msg.cmd.arg1, msg.cmd.arg2);
      sendMessage(msg.from, CMD_OP_FLASH_IMAGE, success, 0, CMD_FLAG_REPLY);
      if (success) {
        DEBUG("Rebooting to flash new image");
        reboot();
      }
      break;
    }
    default:
      break;
  }
}

void Node::dispatchMessage(Message& msg)
{
  for (int i = 0; i < MAX_MESSAGE_LISTENERS; i++) {
    if (_listeners[i]) {
      DEBUG("Dispatching message to listener %d", i);
      _listeners[i](msg);
    }
  }
}

void Node::setAddress(uint8_t address)
{
  // Actual reassignment will occur before going to sleep
  _settings.address = address;
  _addressChanged = true;
  writeSettings();
}

void Node::setGroup(uint8_t group)
{
  _settings.group = group;
  writeSettings();
}

void Node::clearSettings(void)
{
  _settings.magic = 0;
  writeSettings();
  readSettings();
}

void Node::reboot(void)
{
  // Tell the watchdog to reset us in 15ms
  wdt_enable(WDTO_15MS);

  // Loop forever (but hopefully just 15ms)
  while(1)
  {
  }
}

bool Node::receiveMessage(Message& msg)
{
  if (_radio.receiveDone() && _radio.DATALEN == sizeof(Command)) {
    DEBUG("Received a message, op = %u", msg.cmd.op);

    noInterrupts();
    msg.from = _radio.SENDERID;
    msg.to = _radio.TARGETID;

    memcpy(&msg.cmd, (void*)_radio.DATA, sizeof(Command));
    bool needACK = _radio.ACKRequested();
    interrupts();

    if (!isMessageForMe(msg)) {
      return false;
    }

    if (needACK) {
      _radio.sendACK();
    }

    handleMessage(msg);
    return true;
  }

  return false;
}

bool Node::sendMessage(Message& msg)
{
  if (needSleep() && address() != GATEWAY_ADDRESS) {
    // It's very likely we'll go to sleep after sending this, so make sure
    // the flag is set
    msg.cmd.flags |= CMD_FLAG_WILL_SLEEP;
  }

  if (msg.to != GATEWAY_ADDRESS && !_wakeStates[msg.to]) {
    // The remote node is asleep, so we're going to send a burst to wake it up.
    // Once it responds, we'll send the real message. For broadcast messages
    // we do not get a reply, instead we just send it in the blind.
    Command burstCmd(CMD_OP_PING);
    burstCmd.group = msg.cmd.group;

    _radio.sendBurst(msg.to, &burstCmd, sizeof(Command));

    if (msg.isBroadcast()) {
      // We don't wait on a reply with broadcast messages
      pause();

      // Send the real message now
      _radio.send(msg.to, &msg.cmd, sizeof(Command));
      return true;
    }

    DEBUG("Sent burst, waiting for reply...");

    Message reply;
    uint32_t wait = millis() + REPLY_TIMEOUT;
    bool gotWoke = false;
    while (millis() < wait) {
      if (receiveMessage(reply) && reply.from == msg.to && reply.cmd.op == CMD_OP_WOKE) {
        gotWoke = true;
        break;
      }
    }

    DEBUG("Got burst reply? %d", gotWoke);

    // Timeout waiting on burst ack, probably not received
    if (!gotWoke) {
      return false;
    }

    pause();

    // If we get here then we are presuming the remote node to be awake. Send the real message.
    return _radio.sendWithRetry(msg.to, &msg.cmd, sizeof(Command), NUM_RETRIES);
  }

  DEBUG("Sending non-burst message");
  bool sent = _radio.sendWithRetry(msg.to, &msg.cmd, sizeof(Command), NUM_RETRIES);
  if (!sent) {
    DEBUG("Failed to send message!");
    if (msg.to != GATEWAY_ADDRESS) {
      DEBUG("Falling back to burst mode");
      _wakeStates[msg.to] = false;
      return sendMessage(msg);
    }
  }

  return sent;
}

bool Node::sendMessage(uint8_t to, CommandOp op, uint16_t arg1, uint16_t arg2, uint8_t flags)
{
  Message msg;
  msg.to = to;
  msg.cmd.op = op;
  msg.cmd.arg1 = arg1;
  msg.cmd.arg2 = arg2;
  msg.cmd.flags = flags;
  return sendMessage(msg);
}

void Node::sendBroadcast(CommandOp op, uint16_t arg1, uint16_t arg2, uint8_t group, uint8_t flags)
{
  Message msg;
  msg.to = RF69_BROADCAST_ADDR;
  msg.cmd.op = op;
  msg.cmd.arg1 = arg1;
  msg.cmd.arg2 = arg2;
  msg.cmd.group = group;
  msg.cmd.flags = flags;
  sendMessage(msg);
}

void Node::bump(void)
{
  _wokeMillis = millis();
  _bumps++;
}

void Node::unbump(void)
{
  if (_bumps == 0) {
    return;
  }

  _bumps--;
}

bool Node::needSleep(void)
{
  return _bumps == 0 || (_wokeMillis > 0 && (millis() - _wokeMillis) >= WAKE_DURATION);
}

void Node::pause(void)
{
  LowPower.powerDown(SLEEP_15MS, ADC_OFF, BOD_OFF);
}

void Node::sleep(period_t period)
{
  _bumps = 0;

  // Tell the node that woke us that we're going to sleep
  if (_sleepNotifyAddress) {
    sendMessage(_sleepNotifyAddress, CMD_OP_SLEEPING);
    _sleepNotifyAddress = 0;
  }

  // Our address may have changed, set it now
  if (_addressChanged) {
    DEBUG("Changing address from to %u", address());
    _radio.setAddress(address());
    _addressChanged = false;
  }

  // Put the radio into low power listen mode
  _radio.startListening();
  DEBUG("Sleeping...(listen)");

  // DEBUG("Sleeping (not listening)");
  // _radio.sleep();

  // Power off the cpu
  LowPower.powerDown(period, ADC_OFF, BOD_OFF);

  DEBUG("Woke up!");

  _sleepNotifyAddress = 0;

  // We woke up, bring the radio out of listen mode
  if (_radio.endListening() && _radio.DATALEN == sizeof(Command)) {
    // We have a message. We don't care what it is other than whether or not
    // it actually applies to us. The message we receive here is unencrypted
    // so we only use this as an indication that someone wants to talk to us.
    Message burstMsg;

    noInterrupts();
    burstMsg.from = _radio.SENDERID;
    burstMsg.to = _radio.TARGETID;
    memcpy(&burstMsg.cmd, (const void*)_radio.DATA, sizeof(Command));
    interrupts();

    if (!isMessageForMe(burstMsg)) {
      DEBUG("Burst message is not for me");
      return;
    }

    // There is an implicit bump() if we are woken up from sleep
    bump();

    if (burstMsg.isBroadcast()) {
      // We don't send WOKE for broadcast messages, so we don't need
      // to wait for that period to end.
      return;
    }

    // Message is specifically for us. We'll reply with WOKE after the
    // burst timeout.
    DEBUG("Burst remaining %ums", _radio.LISTEN_BURST_REMAINING_MS);

    // We don't need the radio on while we wait on the burst to end
    _radio.sleep();

    // Wait until the remote side is done trying to wake us up, then reply
    // that we are awake
    int32_t sleepRemaining = _radio.LISTEN_BURST_REMAINING_MS;

    DEBUG("Waiting %dms to send WOKE", sleepRemaining);

    while (sleepRemaining > 0) {
      LowPower.powerDown(SLEEP_60MS, ADC_OFF, BOD_OFF);
      sleepRemaining -= 60;
    }

    if (sendMessage(burstMsg.from, CMD_OP_WOKE)) {
      DEBUG("Sent WOKE message");
    } else {
      DEBUG("Failed to send WOKE");
    }
  } else {
    DEBUG("No message");
  }
}

void Node::onMessage(MessageListener listener)
{
  for (int i = 0; i < MAX_MESSAGE_LISTENERS; i++) {
    if (!_listeners[i]) {
      _listeners[i] = listener;
      break;
    }
  }
}

void Node::removeMessageListener(MessageListener listener)
{
  for (int i = 0; i < MAX_MESSAGE_LISTENERS; i++) {
    if (_listeners[i] == listener) {
      _listeners[i] = NULL;
      break;
    }
  }
}

void Node::run(void)
{
  Message msg;
  if (receiveMessage(msg)) {
    dispatchMessage(msg);
  }

  if (needSleep()) {
    sleep();
  }
}