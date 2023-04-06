// Copyright (c) 2022 Richard Kelsey. All rights reserved.

#ifndef UTIL_H
#define UTIL_H

#include <fmt/core.h>

template <typename... Args>
std::runtime_error fmtException(const std::string& formatString, const Args&... args)
{
  return std::runtime_error(fmt::format(formatString, args...));
};

#endif // UTIL_H
