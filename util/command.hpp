// Copyright (c) 2022 Richard Kelsey. All rights reserved.

#include <cstdint>
#include "gsl-lite.hpp"

class CommandArgument;
enum class CommandArgumentType;

class CommandArguments {
public:
    CommandArguments(const gsl::cstring_span commandName, std::vector<CommandArgument> arguments);

    size_t argumentCount() const;
    uint32_t uint32At(uint8_t index) const;
    int32_t int32At(uint8_t index) const;
    gsl::cstring_span stringAt(uint8_t index) const;
    gsl::span<uint8_t> bytesAt(uint8_t index) const;
    gsl::cstring_span commandName() const;

    ~CommandArguments() = default;

    // Don't move or copy.
    CommandArguments& operator=(const CommandArguments&) = delete;
    CommandArguments& operator=(CommandArguments&& other) = delete;
    CommandArguments(const CommandArguments&) = delete;
    CommandArguments(CommandArguments&& other) = delete;

private:
    const gsl::cstring_span mCommandName;
    const std::vector<CommandArgument> mArguments;
    void checkType(uint8_t index, CommandArgumentType type) const;
};

using CommandAction = void(*)(const CommandArguments &arguments);

// argumentTypes is a string that gives the types of the arguments to
// the command.
//  'u' unsigned integer
//  'i' unsigned integer
//  's' string
// So "iis" would mean three arguments, the first two are integers and
// the third a string.

struct CommandSpec {
    const std::string name;
    const std::string argumentTypes;
    CommandAction action;
};

using CommandTable = std::vector<CommandSpec>;

enum class CommandError {
    UNKNOWN_COMMAND,
    WRONG_NUMBER_OF_ARGUMENTS,
    ARGUMENT_OUT_OF_RANGE,
    ARGUMENT_SYNTAX_ERROR
};

void commandErrorHandler(const char *command, 
                         CommandError error, 
                         const char *message);

class CommandInterpreter
{
public:
    CommandInterpreter(const CommandTable& commandTable) :
        mCommandTable(std::move(commandTable)),
        mDiscard(false)
    {
    }
    bool processInput(gsl::cstring_span input);
    void readCommands(int fd);

private:
    const CommandTable mCommandTable;
    std::string mBuffer;
    bool mDiscard;

    void processCommand();
    size_t nextSpace(size_t i);
    size_t nextNonspace(size_t i);
    bool readInt(gsl::cstring_span input, int64_t *valueLoc);
    void callErrorHandler(CommandError error);
};

void sendCommand(int fd,
                 const std::string& name,
                 const std::initializer_list<int32_t> args);
