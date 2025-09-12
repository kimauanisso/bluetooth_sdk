#pragma once

typedef void (*CommandMethod)(const char *params);

typedef struct Command {
  char *command;
  CommandMethod method;
} Command;

#define CONCAT(a, b) a##b

// clang-format off
#define BT_COMMAND(command_name, command_method)                  \
  __attribute__((used))                                           \
  __attribute__((section("BT_COMMANDS")))                         \
  static const Command CONCAT(__bt_command_, command_name) = {    \
      .command = #command_name,                                   \
      .method = command_method                                    \
  };

#define BT_COMMAND_DEFINE(command_name)                           \
  static void command_name(const char *params);                   \
  BT_COMMAND(command_name, command_name);                         \
  void command_name(const char *params)
// clang-format on

void __run_command(char bt_input[]);