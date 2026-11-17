#pragma once

#include <Arduino.h>
#include <string.h>

namespace observer {

// Admin/USB routing only; never call from the observer command interface.
inline bool handleAdminOtaCommand(const char* command, char* reply) {
  if (strncmp(command, "start ota", 9) != 0) return false;
  if (command[9] != '\0') {
    strcpy(reply, "ERR: use start ota without arguments");
    return true;
  }
  // The observer already owns Bluefruit. Do not enter the legacy path that
  // initializes it again; the core primitive resets directly to bootloader OTA.
  enterOTADfu();
  strcpy(reply, "ERR: OTA reset returned");  // Hardware must not return.
  return true;
}

}  // namespace observer
