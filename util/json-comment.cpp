// Copyright (c) 2022 Richard Kelsey. All rights reserved.

#include <fstream>
#include <iostream>
#include <string>

#include <nlohmann/json.hpp>

// Removes comments of the form "//..." from the string by replacing
// the characters with spaces.  Slashes within strings are ignored.

enum {
    BASE, // no special context
    COMMENT, // within a comment
    SLASH, // previous character was '/'
    STRING, // within a string
    ESCAPE // within a string and previous character was '\'
};

static void removeComments(std::string& str)
{
    int state = BASE;
    for (int i = 0; i < str.size(); i++) {
        char next = str[i];
        switch (state) {

        case BASE:
            if (next == '/') {
                state = SLASH;
            } else if (next == '"') {
                state = STRING;
            }
            break;

        case SLASH:
            if (next == '/') {
                str[i - 1] = ' ';
                str[i] = ' ';
                state = COMMENT;
            } else if (next == '"') {
                state = STRING;
            } else {
                state = BASE;
            }
            break;

        case COMMENT:
            if (next == '\n' || next == '\r') {
                state = BASE;
            } else {
                str[i] = ' ';
            }
            break;

        case STRING:
            if (next == '"') {
                state = BASE;
            } else if (next == '\\') {
                state = ESCAPE;
            }
            break;

        case ESCAPE:
            state = STRING;
            break;
        }
    }
}

nlohmann::json readConfig(std::string filename)
{
    std::ifstream in(filename);
    std::string input((std::istreambuf_iterator<char>(in)),
            std::istreambuf_iterator<char>());
    removeComments(input);
    nlohmann::json config = nlohmann::json::parse(input);
    // fprintf(stderr, "[config: '%s']\n", config.dump().c_str());
    return config;
}
