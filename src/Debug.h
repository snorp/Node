#ifndef Debug_h__
#define Debug_h__

#ifdef NODE_DEBUG
#define MAX_DEBUG_LINE 128
static void printDebug(const __FlashStringHelper *pgmFmt, ...)
{
  char fmt[MAX_DEBUG_LINE];
  char buf[MAX_DEBUG_LINE];

  PGM_P p = reinterpret_cast<PGM_P>(pgmFmt);
  unsigned char c = 0;
  size_t n = 0;
  while ((c = pgm_read_byte(p++)) != 0) {
    fmt[n++] = c;
  }
  fmt[n] = '\0';

  va_list args;
  va_start(args, pgmFmt);
  vsnprintf(buf, MAX_DEBUG_LINE, fmt, args);
  va_end(args);
  Serial.println(buf);
  Serial.flush();
}

#define DEBUG(fmt, ...) printDebug(F(fmt), ##__VA_ARGS__)

#define DEBUG_INIT(baudRate) Serial.begin(baudRate)

#else

#define DEBUG(fmt, ...)
#define DEBUG_INIT(baudRate)

#endif

#endif // Debug_h__