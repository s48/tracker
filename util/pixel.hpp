// Copyright (c) 2022 Richard Kelsey. All rights reserved.

#ifndef PIXEL_HPP
#define PIXEL_HPP

#include <constant.hpp>

#include <array>
#include <cstdint>
#include <functional>
#include <vector>

// Canvas dimensions are floorSize * floorSize.
using Canvas = std::array<uint8_t, floorSize * floorSize>;

// This returns the starting index for the boundary, which might
// not be the original startIndex.
uint32_t walkPixelBoundary(const Canvas& canvas,
                           uint32_t startIndex,
                           std::function<void(uint32_t, uint8_t)> func);

// The offsets to the six neighboring staggered squares, listed clockwise.
const std::array<int, 6> directionOffsets(uint32_t i);

void rotatePath(Canvas& canvas,
                const std::vector<uint8_t>& path,
                uint16_t startX,
                uint16_t startY,
                float angle,
                uint16_t centerX,
                uint16_t centerY);

void drawLine(uint16_t x0, 
              uint16_t y0,
              uint16_t x1,
              uint16_t y1,
              std::function<void(uint8_t, uint8_t)> proc);

// This doesn't handle arcs that go more than 3/4 around the circle.
// Need a wrapper function that divides them into two shorter arcs.

void drawPixelArc(Canvas& canvas,
                  int16_t centerX,
                  int16_t centerY,
                  int16_t radius,
                  int16_t x0,
                  int16_t y0,
                  int16_t x1,
                  int16_t y1);

void drawPixelArc(Canvas& canvas,
                  int16_t centerX,
                  int16_t centerY,
                  int16_t radius,
                  float startAngle,
                  float endAngle);

// Debug stuff.
void printPixels(const Canvas& canvas);
void drawPixelBoundary(Canvas& canvas,
                       uint32_t startIndex,
                       const std::vector<uint8_t>& directions);
void findPixelBoundaryTest();

#endif // PIXEL_HPP
