// Copyright (c) 2022 Richard Kelsey. All rights reserved.

#include <cstdint>
#include <cstring>
#include <cstdio>
#include <cassert>

#include "gsl-lite.hpp"

#include "command.hpp"

static void command0(const CommandArguments &arguments) 
{
  assert(arguments.argumentCount() == 0);
  fprintf(stderr, "[command '%s']\n", arguments.commandName().data());
}

static void command1(const CommandArguments &arguments) 
{
  int32_t arg0 = arguments.int32At(0);
  fprintf(stderr, "[command '%s' argument %d]\n", arguments.commandName().data(), arg0);
}

const std::vector<CommandSpec> commandTable = {
  { "x", "",  command0 },
  { "y", "i", command1 },
};

CommandInterpreter commander = {commandTable};

void commandErrorHandler(const char *command,
                         CommandError error,
                         const char *message)
{
  fprintf(stderr, "Command error: '%s' %d '%s'\n", command, (int) error, message);
}

int main(void)
{
    commander.processInput("x\n");
    commander.processInput("y 7\n");
    commander.processInput("y -70\n");
    commander.processInput("y 0xA\n");
    commander.processInput("y -2147483647\n");
    commander.processInput("y -2147483648\n");
    commander.processInput("y 2147483647\n");
    commander.processInput("x 7\n");
    commander.processInput("y\n");
    commander.processInput("y x\n");
    return 0;
}
