// Copyright (c) 2022 Richard Kelsey. All rights reserved.

#include <string>

int openSocket(char *name, bool server);
int acceptClient(int serverFd);
uint32_t readSome(int fd, uint8_t *buf, uint32_t max);
void readN(int fd, uint8_t *buf, uint32_t needed);
void writeN(int fd, uint8_t *buf, uint32_t length);
