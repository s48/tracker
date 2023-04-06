// Copyright (c) 2022 Richard Kelsey. All rights reserved.

#include "cruft.hpp"
#include "pixel.hpp"
#include "util.hpp"

#include <vector>

static Canvas canvas;

// The image is one pixel wider to allow for shifted rows.
static const uint32_t imageHeight = 2 * floorSize + 1;
static const uint32_t imageWidth = 3 * floorSize;
static uint8_t image[imageHeight * imageWidth];

// For consistency this duplicates setFloorPixel() in display.cpp.
void setPixel(const uint32_t index, uint8_t mark)
{
    uint32_t x = (index % floorSize) * 3; // squares are three pixels wide
    uint32_t y = (index / floorSize) * 6; // squares are three pixels wide and two high
    uint32_t pixelIndex = x + y * floorSize + imageWidth;
    if (((index % floorSize) & 1) == 0) {
        pixelIndex += imageWidth; // move up one pixel
    }
    // Write a block of six pixels.

    for (uint32_t i = 0; i < 2; i++) {
        image[pixelIndex] = mark;
        image[pixelIndex + 1] = mark;
        image[pixelIndex + 2] = mark;
        pixelIndex += imageWidth;
    }
/*
    uint32_t z = index % floorSize;
    image[pixelIndex] = '0' + (z / 100) % 10;
    image[pixelIndex + 1] = '0' + (z / 10) % 10;
    image[pixelIndex + 2] = '0' + (z / 1) % 10;
    z = index / floorSize;
    pixelIndex += imageWidth;
    image[pixelIndex] = '0' + (z / 100) % 10;
    image[pixelIndex + 1] = '0' + (z / 10) % 10;
    image[pixelIndex + 2] = '0' + (z / 1) % 10;
*/
}

void printImage() {
    for (int32_t i = 30 - 1; 10 <= i; i--) {
        printf("%.60s\n", image + i * imageWidth);
    }
}

void printCanvas() {
    memset(image, ' ', sizeof(image));
    for (int32_t i = 15 - 1; 5 <= i; i--) {
        for (int32_t j = 0; j < 20; j++) {
            uint32_t index = i * floorSize + j;
            if (canvas[index]) {
                setPixel(index, '#');
            }
        }
    }
    printImage();
}

static uint8_t count = 0;

static void linePixel(uint8_t x, uint8_t y)
{
    uint32_t index = x + y * floorSize;
    setPixel(index, 'A' + (count % 26));
    canvas[index] = 1;
    count += 1;
}

int main(int argc, char **argv)
{
    memset(image, ' ', sizeof(image));
    drawLine(8, 8, 8, 12, linePixel);
    drawLine(8, 12, 15, 8, linePixel);
    drawLine(15, 8, 8, 8, linePixel);
    printImage();
    std::vector<uint8_t> path;
    uint32_t startIndex = walkPixelBoundary(canvas,
                                            10 + 8 * floorSize,
                                            [&path](uint32_t index, uint8_t direction)
                                                {
                                                    // printf(" %d", direction);
                                                    path.push_back(direction);
                                                });
    printf("\n");
    canvas.fill(0);
    drawPixelBoundary(canvas, startIndex, path);
    printCanvas();
    canvas.fill(0);
    rotatePath(canvas,
               path,
               startIndex % floorSize,
               startIndex / floorSize,
               1.57f,
               10,
               10);
    printCanvas();
    return 0;
}
