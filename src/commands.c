#include "bt_kmn/commands.h"
#include <stdio.h>
#include <string.h>

extern const Command __start_BT_COMMANDS[];
extern const Command __stop_BT_COMMANDS[];

#define COMMAND_DELIMITER "="

void parse_command(char *bt_input, char **bt_command, char **bt_params) {
  *bt_command = strtok(bt_input, COMMAND_DELIMITER);
  *bt_params = strtok(NULL, COMMAND_DELIMITER);
}

CommandMethod get_command(const char *command) {
  const Command *cmd = __start_BT_COMMANDS;
  while (cmd < __stop_BT_COMMANDS) {
    if (strcmp(cmd->command, command) == 0) {
      return cmd->method;
    }
    cmd++;
  }
  return NULL;
}

void run_command(char bt_input[]) {
  char *command, *params;
  parse_command(bt_input, &command, &params);

  CommandMethod method = get_command(command);
  if (method == NULL) {
    printf("Command not found\n");
    return;
  }

  method(params);
}
