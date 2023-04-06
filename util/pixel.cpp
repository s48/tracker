// Copyright (c) 2022 Richard Kelsey. All rights reserved.

#include "pixel.hpp"

#include <algorithm>
#include <array>
#include <cassert>
#include <cstdint>
#include <cstring>
#include <math.h>
#include <vector>

#include <stdio.h>
#include <stdlib.h>

extern bool debugPrint;

// The offsets are the staggered square analogy to the 'Moore neighborhood',
// listed clockwise.  There are six neighbors, as opposed to eight with
// orthogonal squares.
//
// The staggered squares look like this:
//
//   +---+   +---+
//   |082|---|202|---+    +---+
//   |---|192|---|312|    |xiy| where i is the array index is mod 10
//   |041|---|261|---|    +---+
//   |---|151|---|371|
//   |000|---|220|---|
//   +---|110|---|330|
//       +---+   +---+

const std::array<int, 6> directionOffsetsEvenX = {
    floorSize,
    floorSize + 1,
    1,
    -floorSize,
    -1,
    floorSize - 1
};

const std::array<int, 6> directionOffsetsOddX = {
    floorSize,
    1,
    -floorSize + 1,
    -floorSize,
    -floorSize - 1,
    -1
};

const std::array<int, 6> directionOffsets(uint32_t i)
{
    return (i & 1
            ? directionOffsetsOddX
            : directionOffsetsEvenX);
}

// Walk the pixels on the boundary of the shape on 'canvas' that
// starts the given index.  The pixel to the left of the starting
// index (at index - 1) must be empty.
//
// From https://en.wikipedia.org/wiki/Moore_neighborhood, adapted
// for staggered squares.

uint32_t walkPixelBoundary(const Canvas&  canvas,
                           uint32_t startPoint,
                           std::function<void(uint32_t, uint8_t)> func)
{
    startPoint = startPoint % floorSize;
    while (canvas[startPoint] == 0) {
        startPoint += floorSize;
    }
    uint32_t boundaryPoint = startPoint;
    // We pretend we arrived at the starting square moving in
    // direction zero (y incrementing), so the first direction to
    // search is four.  This is the next neighbor clockwise from the
    // square we arrived from (which is in direction three, the
    // reciprocal of direction zero).
    int direction = 4;
    auto offsets = directionOffsets(boundaryPoint);

    while (true) {
        int trialPoint = boundaryPoint + offsets[direction];
        // printf("(%d %d) -%d-> (%d %d) %d\n",
        //        boundaryPoint % floorSize,
        //        boundaryPoint / floorSize,
        //        direction,
        //        trialPoint % floorSize,
        //        trialPoint / floorSize,
        //        canvas[trialPoint]);
        // Stop if we are in the start square and checking back to
        // the square we pretended we came in on.  This means we have
        // have checked all directions from the start square and
        // traversed anything that was found.
        if (boundaryPoint == startPoint
            && direction == 3) {
            return startPoint;
        }

        if (canvas[trialPoint]) {
            func(trialPoint, direction);
            // The reciprocal direction is (x + 3) % 6, but we want
            // the next neighbor clockwise from there, so we add one
            // more.
            direction = (direction + 4) % 6;
            boundaryPoint = trialPoint;
            offsets = directionOffsets(boundaryPoint);
        } else {
            direction = (direction + 1) % 6;
        }
    }
}

void drawPixelBoundary(Canvas&  canvas,
                       uint32_t startIndex,
                       const std::vector<uint8_t>& directions)
{
    for (auto& dir : directions) {
        startIndex += directionOffsets(startIndex)[dir];
        canvas[startIndex] = 1;
    }
}

static bool printValues = false;

void printPixels(const Canvas&  canvas)
{
    uint16_t minX = floorSize;
    uint16_t maxX = 0;
    uint16_t minY = floorSize;
    uint16_t maxY = 0;

    for (uint16_t i = 0; i < floorSize; i++) {
        for (uint16_t j = 0; j < floorSize; j++) {
            if (canvas[i + j * floorSize]) {
                minX = std::min(minX, i);
                maxX = std::max(maxX, i);
                minY = std::min(minY, j);
                maxY = std::max(maxY, j);
            }
        }
    }
    fprintf(stderr, "[bounds (%d, %d) (%d, %d)]\n", minX, minY, maxX, maxY);
    for (uint16_t i = minX; i <= maxX; i++) {
        for (uint16_t j = minY; j <= maxY; j++) {
            uint8_t value = canvas[i + j * floorSize];
            if (printValues) {
                fprintf(stderr, "%d", std::min((int) value, 9));
            } else {
                fprintf(stderr, "%c", value ? '#' : ' ');
            }
        }
        fprintf(stderr, "\n");
    }
}

//----------------------------------------------------------------
// Rotating a path
//
// To get a contiguous path after the rotation this treats rotates
// thirteen points from within each pixel and counts a pixel as in the
// rotation if three or more rotated points lie within it.  The
// thirteen points are the center and twelve equally-spaced points
// with a radius of 1/2.

static const std::array<float, 13> xDeltas
{0.0f, 0.5f, 0.43f, 0.25f, 0.0f, -0.25f, -0.43f, -0.5f, -0.43f, -0.25f, 0.0f, 0.25f, 0.43f};
static const std::array<float, 13> yDeltas
{0.0f, 0.0f, 0.25f, 0.43f, 0.5f, 0.43f, 0.25f, 0.0f, -0.25f, -0.43f, -0.5f, -0.43f, -0.25f};

void rotatePath(Canvas&  canvas,
                const std::vector<uint8_t>& path,
                uint16_t startX,
                uint16_t startY,
                float angle,
                uint16_t centerX,
                uint16_t centerY)
{
    float cosine = cos(angle);
    float sine = sin(angle);
    uint32_t pixelIndex = startX + startY * floorSize;
    Canvas scratchPad;
    scratchPad.fill(0);
    for (auto& dir : path) {
        pixelIndex += directionOffsets(pixelIndex)[dir];
        float x1 = (int16_t) ((pixelIndex % floorSize) - centerX);
        float y1 = (int16_t) ((pixelIndex / floorSize) - centerY);
        // Same as in makeVoxels() in static/voxel-data.cpp.
        if (pixelIndex & 1 == 0) {
            y1 -= 0.5;
        }
        for (uint16_t i = 0; i < 13; i++) {
            float x2 = x1 + xDeltas[i];
            float y2 = y1 + yDeltas[i];
            uint16_t x3 = centerX + std::round(cosine * x2 - sine * y2);
            float dy = x3 & 1 ? 0.0 : -0.5;
            uint16_t y3 = centerY + std::round(sine * x2 + cosine * y2 + dy);
            uint32_t index = x3 + y3 * floorSize;
            if (scratchPad[index] == 2) { // three points have hit 'index'.
                canvas[index] = 1;
            } else {
                scratchPad[index] += 1;
            }
        }
    }
}

// This is inefficient, but we only need to draw a few lines and
// I don't want to spend a bunch of time optimizing it.  From
// https://www.redblobgames.com/grids/hexagons/#line-drawing

static float interpolate(float x, float y, float f)
{
    return x + (y - x) * f;
}

static uint16_t hexDistance(int16_t q0, int16_t r0, int16_t q1, int16_t r1)
{
    return (abs(q0 - q1) + abs(q0 + r0 - q1 -r1) + abs(r0 - r1)) / 2;
}

static int16_t xyToR(uint16_t x, uint16_t y)
{
    return y - ((x + (x & 1)) >> 1);
}

static uint16_t qrToY(int16_t q, int16_t r)
{
    return r + ((q + (q & 1)) >> 1);
}

void drawLine(uint16_t x0,
              uint16_t y0,
              uint16_t x1,
              uint16_t y1,
              std::function<void(uint8_t, uint8_t)> proc)
{
    // convert from xy to rqs
    int16_t q0 = x0;
    int16_t r0 = xyToR(x0, y0);
    int16_t s0 = -q0 - r0;
    int16_t q1 = x1;
    int16_t r1 = xyToR(x1, y1);
    int16_t s1 = -q1 - r1;

    uint16_t steps = hexDistance(q0, r0, q1, r1);
    // The 1e-6 avoids exactly bisecting any hexagon edges, which
    // would otherwise make for arbitrary wobbles in some lines.
    float stepSize = 1.0f / (float) steps + 1e-6;
    for (uint16_t i = 0; i <= steps; i++) {
        // interpolate the next hexagon
        float qF = interpolate(q0, q1, i * stepSize);
        float rF = interpolate(r0, r1, i * stepSize);
        float sF = interpolate(s0, s1, i * stepSize);

        // round to integers
        int16_t q = round(qF);
        int16_t r = round(rF);
        int16_t s = round(sF);

        float dq = abs(q - qF);
        float dr = abs(r - rF);
        float ds = abs(s - sF);

        // pick the hexagon closest to the line
        if (dr < dq && ds < dq) {
            q = -r - s;
        } else if (ds < dr) {
            r = -q - s;
        } else {
            s = -q - r;
        }

        // convert back to xy
        uint8_t x = q;
        uint8_t y = qrToY(q, r);
        proc(x, y);
    }
}
