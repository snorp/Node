#ifndef Command_h__
#define Command_h__

#include <stdint.h>

enum CommandOp {
  // Common commands
  CMD_OP_PING = 0,
  CMD_OP_SLEEPING,      // node has gone to sleep
  CMD_OP_WOKE,          // sent in reply to a burst message, indicating we have woken up and received the message
  CMD_OP_REASSIGN,      // change a node's address
  CMD_OP_BATTERY_LEVEL, // notify the battery level
  CMD_OP_GROUP,         // set or notify group membership
  CMD_OP_CLASS,         // sent by device to notify what class it belongs
  CMD_OP_POLL,          // request the node send all status information
  CMD_OP_RESET,         // do a factory reset!
  CMD_OP_FLASH_IMAGE,   // begin flashing a new image!
  CMD_OP_ACK,

  // Commands for blinds control
  CMD_OP_BLINDS_POSITION = 50, // set or notify blinds position, which an index 0-2
  CMD_OP_BLINDS_ANGLE,         // set or notify the blinds angle
  CMD_OP_BLINDS_STEP,          // step the blinds to the next position
  CMD_OP_BLINDS_ANGLES,        // set or notify the blinds angles; 3 bytes between arg1 and arg2
  CMD_OP_BLINDS_SPEED,         // set or notify the blinds speed, 0-100
};

enum CommandFlags {
  CMD_FLAG_WILL_SLEEP = 1,        // Sender is going to sleep after sending this message
  CMD_FLAG_STAY_AWAKE = (1 << 1), // Receiver should stay awake for some period of time
  CMD_FLAG_REPLY = (1 << 2),      // This a reply to a message
};

struct Command {
  uint16_t op;
  uint16_t arg1;
  uint16_t arg2;
  uint8_t group;
  uint8_t flags;

  Command() {
    memset(this, 0, sizeof(Command));
  }

  Command(int16_t op, int16_t arg1 = 0, int16_t arg2 = 0, uint8_t group = 0, uint8_t flags = 0)
    : op(op)
    , arg1(arg2)
    , arg2(arg2)
    , group(group)
    , flags(flags)
  {
  }
};

#endif
