#ifndef Message_h__
#define Message_h__

#include "RFM69_WL.h"
#include "Command.h"

struct Message {
  uint8_t from;
  uint8_t to;
  Command cmd;

  bool isBroadcast(void) {
    return from == RF69_BROADCAST_ADDR;
  }
};

#endif
