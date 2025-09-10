#include "bt_kmn/commands.h"
#include <stdio.h>
#include <string.h>

extern const Command __start_BT_COMMANDS[];
extern const Command __stop_BT_COMMANDS[];

void run_command(const char *command) {
  const Command *cmd = __start_BT_COMMANDS;
  while (cmd < __stop_BT_COMMANDS) {
    if (strcmp(cmd->command, command) == 0) {
      cmd->method(command);
      return;
    }
    cmd++;
  }
  printf("Command not found\n");
}
