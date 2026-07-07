#ifndef DEBUGLOG_H
#define DEBUGLOG_H

#include <Arduino.h>

// Debug prints share UART0 with the ROS2 STATE/CMD protocol, so a print
// from another task can interleave mid-STATE-line and corrupt a read on
// the host. Runtime debug prints must go through DBG_PRINTF, which
// compiles to nothing unless the build enables it:
//
//   build_flags = -DDEBUG_LOG=1   (platformio.ini)
//
// Enable only for bench debugging without the ROS2 host attached.
// Boot-time prints in setup()/begin() may use Serial directly.

#ifndef DEBUG_LOG
#define DEBUG_LOG 0
#endif

#if DEBUG_LOG
#define DBG_PRINTF(...)  Serial.printf(__VA_ARGS__)
#define DBG_PRINTLN(x)   Serial.println(x)
#else
#define DBG_PRINTF(...)  do {} while (0)
#define DBG_PRINTLN(x)   do {} while (0)
#endif

#endif // DEBUGLOG_H
