// Copyright (c) 2022 Richard Kelsey. All rights reserved.

#include <stdio.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <poll.h>
#include <errno.h>
#include <fcntl.h>

#include "socket.hpp"

int openSocket(char *name, bool server)
{
  int fd = socket(AF_UNIX, SOCK_STREAM, 0);
  if (fd == -1) {
    perror("socket() error");
    exit(-1);
  }

  struct sockaddr_un addr;
  memset(&addr, 0, sizeof(addr));
  addr.sun_family = AF_UNIX;
  *addr.sun_path = '\0';
  strncpy(addr.sun_path + 1, name, sizeof(addr.sun_path) - 2);

  if (server) {
    if (bind(fd, (struct sockaddr *)&addr, sizeof(addr)) == -1) {
      perror("bind() error");
      exit(-1);
    }
    if (listen(fd, 5) == -1) {
      perror("listen() error");
      exit(-1);
    }
    int flags = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, flags | O_NONBLOCK);
  } else {
    if (connect(fd, (struct sockaddr *)&addr, sizeof(addr)) == -1) {
      fprintf(stderr, "[failed to connect to %s\n", name);
      perror("connect() error");
      exit(-1);

      int flags = fcntl(fd, F_GETFL, 0);
      fcntl(fd, F_SETFL, flags | O_NONBLOCK);
    }
  }

  return fd;
}

int acceptClient(int serverFd)
{
    int fd = accept(serverFd, NULL, NULL);
    if (0 <= fd) {
        int flags = fcntl(fd, F_GETFL, 0);
        fcntl(fd, F_SETFL, flags | O_NONBLOCK);
        return fd;
    }
    if (errno == EAGAIN
            || errno == EWOULDBLOCK) {
        return -1;
    }
    perror("accept() error");    
    exit(-1);
}

uint32_t readSome(int fd, uint8_t *buf, uint32_t max)
{
      struct pollfd pollfd;
      pollfd.fd = fd;
      pollfd.events = POLLIN;
      while (true) {
          int count = poll(&pollfd, 1, 0);
          if (count == -1) {
              if (errno == EINTR) {
                  continue;
              } else {
                  perror("poll() error");
                  exit(-1);
              }
          }
          if (count == 0) {
              return 0;
          }
          if (pollfd.revents & (POLLERR | POLLNVAL)) {
              fprintf(stderr, "poll reported error\n");
              exit(-1);
          }
          int got = read(fd, buf, max);
          if (got <= 0) {
              perror(got < 0
                      ? "read() error"
                      : "read() on closed fd");
              exit(-1);
          }
          return got;
      }
}

void readN(int fd, uint8_t *buf, uint32_t needed)
{
  while (0 < needed) {
    int got = read(fd, buf, needed);
    if (got <= 0) {
      perror(got < 0
             ? "read() error"
             : "read() on closed fd");
      exit(-1);
    }
    needed -= got;
    buf += got;
  }
}

void writeN(int fd, uint8_t *buf, uint32_t length)
{
  while (0 < length) {
    int sent = write(fd, buf, length);
    if (sent < 0) {
      perror("write() error");
      exit(-1);
    }
    length -= sent;
    buf += sent;
  }
}

void sendCommand(int fd,
                 const std::string& name,
                 const std::initializer_list<int32_t> args)
{
    if (fd < 0) {
        return;
    }
    uint8_t buf[256];
    uint8_t count = snprintf((char *)buf, 256, "%s", name.c_str());
    for (auto arg : args) {
        count += snprintf((char *) (buf + count), 256 - count, " %d", arg);
    }
    buf[count] = '\n';
    writeN(fd, buf, count + 1);
}
