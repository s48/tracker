// Copyright (c) 2022 Richard Kelsey. All rights reserved.

#ifndef CAMERA_HPP
#define CAMERA_HPP

#include <array>
#include <cstdint>
#include <cstring>
#include <vector>

#include <math.h>
#include <netinet/ip.h>
#include <sys/socket.h>

//----------------------------------------------------------------
// The terabee camera.
const uint32_t imageWidth = 640;
const uint32_t imageHeight = 480;
const uint32_t imagePixelCount = imageHeight * imageWidth;
using ImageF = std::array<float, imagePixelCount>;
using Image8 = std::array<uint8_t, imagePixelCount>;
using Image16 = std::array<uint16_t, imagePixelCount>;
const float focalLength = 24;     // just a guess, what matters is the
                                  // ratio to the film aperture
const float filmApertureWidth = 48;  // 90 degree angles are easy
const float filmApertureHeight = 36; // from the 4/3 image ratio

// Packet data
const int bytesPerPixel = 2;
const int rowsPerPacket = 5;
const int packetsPerImage = (imageHeight + (rowsPerPacket - 1)) / rowsPerPacket;
const int pixelsPerPacket = imageWidth * rowsPerPacket;
const int maxPacketPayloadSize = bytesPerPixel * pixelsPerPacket;

//----------------------------------------------------------------

class CameraSource {

public:
    CameraSource(int sinkPort);

    void sendImage(uint32_t frameCounter, const ImageF& depthData);

    // Don't copy.
    ~CameraSource() = default;
    CameraSource(const CameraSource&) = delete;
    CameraSource& operator=(const CameraSource&) = delete;
    CameraSource(CameraSource&&) = default;
    CameraSource& operator=(CameraSource&&) = default;

private:
    int mSourceFd;
    sockaddr mSinkAddr;
    socklen_t mSinkAddrLen;
};

//----------------------------------------------------------------

class CameraSink {

public:
    CameraSink(int fd);

    // Reads in a packet and updates 'depthImage'.  Returns true and
    // the frame number if a complete image has been received.
    std::tuple<bool, uint32_t> read(Image16& depthImage);

    CameraSink() = default;

    // Don't copy.
    ~CameraSink() = default;
    CameraSink(const CameraSink&) = delete;
    CameraSink& operator=(const CameraSink&) = delete;
    CameraSink(CameraSink&&) = default;
    CameraSink& operator=(CameraSink&&) = default;

private:
    int mFd;
    uint32_t mCurrentFrame;
    // how many packets of the current frame have arrived
    uint32_t mReceivedPacketCount;
    // which packets have been received for the current frame
    std::vector<bool>mReceivedPacketMask;
};

#endif // CAMERA_HPP
