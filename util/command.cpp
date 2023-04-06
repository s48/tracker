// Copyright (c) 2022 Richard Kelsey. All rights reserved.

#include <cstdint>
#include <cstring>

#include "gsl-lite.hpp"

#include "socket.hpp"
#include "command.hpp"

static const char * const errorMessages[] = {
    "unknown command",
    "wrong number of arguments",
    "argument out of range",
    "argument syntax error",
    "command too long"
};

// Argument types.
enum class CommandArgumentType {
    int32,
    uint32,
    bytes
};

struct CommandArgument {
    CommandArgument(CommandArgumentType type, int32_t int32) :
        mType(type),
        mInt32(int32)
    {}
    CommandArgument(CommandArgumentType type, gsl::span<uint8_t> bytes) :
        mType(type),
        mBytes(bytes)
    {}

    CommandArgumentType mType;
    int32_t mInt32;
    gsl::span<uint8_t> mBytes;
};

//----------------------------------------------------------------
// CommandArguments methods

gsl::cstring_span CommandArguments::commandName() const
{
    return mCommandName;
}

size_t CommandArguments::argumentCount() const
{
    return mArguments.size();
};

void CommandArguments::checkType(uint8_t index, CommandArgumentType type) const
{
    if (mArguments.size() <= index
        || mArguments[index].mType != type) {
        exit(-1);
    }
}

uint32_t CommandArguments::uint32At(uint8_t index) const
{
    checkType(index, CommandArgumentType::uint32);
    return (uint32_t) mArguments[index].mInt32;
}

int32_t CommandArguments::int32At(uint8_t index) const
{
    checkType(index, CommandArgumentType::int32);
    return mArguments[index].mInt32;
}

gsl::cstring_span CommandArguments::stringAt(uint8_t index) const
{
    checkType(index, CommandArgumentType::bytes);
    return gsl::cstring_span((char *)mArguments[index].mBytes.data(),
                             mArguments[index].mBytes.length());
}

gsl::span<uint8_t> CommandArguments::bytesAt(uint8_t index) const
{
    checkType(index, CommandArgumentType::bytes);
    return mArguments[index].mBytes;
}

CommandArguments::CommandArguments(const gsl::cstring_span commandName,
                                   std::vector<CommandArgument> arguments) :
    mCommandName(commandName),
    mArguments(std::move(arguments))
{
}

//----------------------------------------------------------------

// forward declarations

void CommandInterpreter::callErrorHandler(CommandError error)
{
    commandErrorHandler((char *) mBuffer.data(),
                        error,
                        errorMessages[(int)error]);
}

bool CommandInterpreter::processInput(gsl::cstring_span input)
{
    bool newlineSeen = false;
    for (auto& next : input) {
        bool isNewline = next == '\r' || next == '\n';
        if (isNewline) {
            newlineSeen = true;
        }
        if (mDiscard) {
            if (isNewline) {
                mDiscard = false;
                mBuffer.clear();
            }
        } else if (mBuffer.size() == 0
                   && isspace(next)) {
            // do nothing - we drop initial whitespace
        } else if (isNewline) {
            // have every argument end with whitespace
            mBuffer.push_back(' ');
            processCommand();
            mBuffer.clear();
        } else {
            mBuffer.push_back(next);
        }
    }
    // Print a prompt if there was a newline and there are no leftover chars.
    return newlineSeen && mBuffer.size() == 0;
}

size_t CommandInterpreter::nextSpace(size_t i)
{
    for (; i < mBuffer.size() && !isspace(mBuffer[i]); i++);
    return i;
}

size_t CommandInterpreter::nextNonspace(size_t i)
{
    for (; i < mBuffer.size() && isspace(mBuffer[i]); i++);
    return i;
}

void CommandInterpreter::processCommand()
{
    std::vector<CommandArgument> arguments;
    const CommandSpec *command = nullptr;

    size_t readIndex = nextSpace(0);
    mBuffer[readIndex] = 0;   // terminate command name
    readIndex += 1;

    for (auto& c : mCommandTable) {
        if (strncmp(mBuffer.data(),
                    static_cast<const char *>(c.name.data()),
                    c.name.size())
            == 0) {
            command = &c;
        }
    }

    if (command == nullptr) {
        callErrorHandler(CommandError::UNKNOWN_COMMAND);
        return;
    }

    for (auto& argSpec : command->argumentTypes) {
        readIndex = nextNonspace(readIndex);
        if (mBuffer.size() <= readIndex) {
            callErrorHandler(CommandError::WRONG_NUMBER_OF_ARGUMENTS);
            return;
        }
        switch (argSpec) {
        case 'i':
        case 'u': {
            bool neg = false;
            if (argSpec == 'i'
                && mBuffer[readIndex] == '-') {
                readIndex += 1;
                neg = true;
            }
            size_t end = nextSpace(readIndex);
            int64_t n;
            if (! readInt(gsl::cstring_span((char *)(mBuffer.data() + readIndex),
                                            end - readIndex),
                          &n)) {
                callErrorHandler(CommandError::ARGUMENT_SYNTAX_ERROR);
                return;
            }
            if (neg) {
                n = -n;
            }
            if (argSpec == 'i'
                && INT32_MIN <= n
                && n <= INT32_MAX) {
                arguments.emplace_back(CommandArgumentType::int32, n);
            } else if (argSpec == 'u'
                       && 0 <= n
                       && n <= UINT32_MAX) {
                arguments.emplace_back(CommandArgumentType::uint32, n);
            } else {
                callErrorHandler(CommandError::ARGUMENT_SYNTAX_ERROR);
                return;
            }
            readIndex = end;
            break;
        }
        case 's': {
            size_t end = nextSpace(readIndex);
            arguments.emplace_back(CommandArgumentType::bytes,
                                   gsl::span<uint8_t>((uint8_t *) mBuffer.data() + readIndex,
                                                      end - readIndex));
            readIndex = end;
            break;
        }
        default:
            callErrorHandler(CommandError::ARGUMENT_SYNTAX_ERROR);
            return;
        };
    }

    if (readIndex < mBuffer.size() - 1) {    // ignore the space at the end
        callErrorHandler(CommandError::WRONG_NUMBER_OF_ARGUMENTS);
        return;
    }

    command->action(CommandArguments(gsl::cstring_span(mBuffer.data()), arguments));
}

static uint8_t hexToInt(uint8_t c)
{
    return ('0' <= c && c <= '9'
            ? c - '0'
            : ('A' <= c && c <= 'F'
               ? c - 'A' + 10
               : 255));
}

// Computed as an int64 to make it easy to detect overflow.
bool CommandInterpreter::readInt(gsl::cstring_span input, int64_t *valueLoc)
{
    int64_t result = 0;
    uint8_t base = 10;
    size_t length = input.size();
    uint8_t i = 0;

    if (3 <= length
        && input[0] == '0'
        && (input[1] == 'x' || input[1] == 'X')) {
        i += 2;
        base = 16;
    }

    if (length <= i) {
        return false;
    }
  
    for ( ; i < length; i++) {
        uint8_t value = hexToInt(input[i]);
        if (value < base) {
            result = result * base + value;
            if (UINT32_MAX < result) {
                return false;
            }
        } else {
            return false;
        }
    }
  
    *valueLoc = result;
    return true;
}

void CommandInterpreter::readCommands(int fd)
{
    uint8_t buf[4096];
    uint32_t got = readSome(fd, buf, sizeof(buf));
    if (0 < got) {
        processInput({(const char *)buf, got});
    }
}
