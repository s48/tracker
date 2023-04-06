// Copyright (c) 2022 Richard Kelsey. All rights reserved.

#include "camera.hpp"
#include "cruft.hpp"

#include <cstdio>
#include <tuple>

#include <netdb.h>
#include <netinet/ip.h>
#include <sys/socket.h>
#include <sys/timeb.h>
#include <sys/un.h>
#include <unistd.h>

extern uint32_t crc32_8bytes (const void* data, size_t length, uint32_t previousCrc32 = 0);

//----------------------------------------------------------------
// Source

CameraSource::CameraSource(int sinkPort)
{
    mSourceFd = socket(AF_INET, SOCK_DGRAM, 0);
    if (mSourceFd == -1) {
        perror("socket() error");
        exit(-1);
    }

    hostent *host = gethostbyname("localhost");
    if (host == nullptr) {
        std::cout << "Unknown host: " << host << "\n";
        exit(-1);
    }
    struct sockaddr_in sinkAddr;
    memset(&sinkAddr, 0, sizeof(sinkAddr));
    sinkAddr.sin_family = AF_INET;
    memcpy(&sinkAddr.sin_addr.s_addr, host->h_addr, host->h_length);
    sinkAddr.sin_port = htons(sinkPort);

    mSinkAddrLen = sizeof(sinkAddr);
    memcpy(&mSinkAddr, &sinkAddr, mSinkAddrLen);
}

static void store32(uint8_t *loc, uint32_t value)
{
    loc[0] = static_cast<uint8_t>(value >> 24);
    loc[1] = static_cast<uint8_t>(value >> 16);
    loc[2] = static_cast<uint8_t>(value >> 8);
    loc[3] = static_cast<uint8_t>(value);
}

void CameraSource::sendImage(uint32_t frameCounter, const ImageF& depthData)
{
    uint8_t packet[8 + maxPacketPayloadSize + 4];
    store32(packet, frameCounter);
    uint32_t pixelIndex = 0;

    for (uint16_t packetCounter = 0; packetCounter < packetsPerImage; packetCounter++) {
        store32(packet + 4, packetCounter);
        uint8_t *writeFinger = packet + 8;
        uint32_t packetPixels = std::min(static_cast<uint32_t>(pixelsPerPacket),
                                         imagePixelCount - pixelIndex);
        for (uint32_t j = 0; j < packetPixels; j++, pixelIndex++) {
            uint16_t depthCm = floatToInt32u(depthData[pixelIndex] * 100);
            *writeFinger++ = static_cast<uint8_t>(depthCm >> 8);
            *writeFinger++ = static_cast<uint8_t>(depthCm);
        }
        size_t length = static_cast<size_t>(writeFinger + 4 - packet);
        store32(writeFinger, crc32_8bytes(packet, length - 4));
        if (sendto(mSourceFd, packet, length, 0, &mSinkAddr, mSinkAddrLen) < 0) {
            perror("sendto() failed");
            exit(0);
        }
    }
}

//----------------------------------------------------------------
// Sink

static uint32_t fetchUint32(const uint8_t *loc)
{
    return static_cast<uint32_t>((loc[0] << 24) | loc[1] << 16 | loc[2] << 8 | loc[3]);
}

CameraSink::CameraSink(int fd) :
    mFd(fd),
    mCurrentFrame(0),
    mReceivedPacketCount(0),
    mReceivedPacketMask(packetsPerImage, false)
{
}

const int headerSize = 8; // frame number and packet number
const int crcSize = 4;

std::tuple<bool, uint32_t> CameraSink::read(Image16& depthImage)
{
    uint8_t packet[headerSize + maxPacketPayloadSize + crcSize];

    ssize_t got = recv(mFd, packet, sizeof(packet), 0);
    if (got < 0) {
        perror("recv() error");
        exit(0);
    }
    if (got < headerSize + crcSize) {
        // This most likely is a result of a problem on the sender.
        fprintf(stderr, "Received short camera packet %ld.\n", got);
        return std::make_tuple(false, 0);
    }

    if (crc32_8bytes(packet, got - crcSize) != fetchUint32(packet + got - crcSize)) {
        fprintf(stderr, "CRC failed\n");
        return std::make_tuple(false, 0);
    }

    uint32_t frameNumber = fetchUint32(packet);
    uint32_t packetNumber = fetchUint32(packet + 4);

    if (packetsPerImage <= packetNumber) {
        fprintf(stderr, "packet number too high\n");
        return std::make_tuple(false, 0);
    }

    if (0 < mReceivedPacketCount
        && frameNumber == mCurrentFrame) {
        if (mReceivedPacketMask[packetNumber]) {
            fprintf(stderr, "Camera %d frame %d duplicate packet %d.\n",
                    mFd,
                    frameNumber,
                    packetNumber);
            return std::make_tuple(false, 0);
        }
    } else if (0x80000000u < static_cast<uint32_t>(frameNumber - mCurrentFrame)) {
        // from an earlier scan, drop it
        return std::make_tuple(false, 0);
    } else {
        if (mReceivedPacketCount != 0
            && mReceivedPacketCount < packetsPerImage) {
            fprintf(stderr, "Camera %d frame %d missing %d packets. Going to frame %d\n",
                    mFd,
                    mCurrentFrame,
                    packetsPerImage - mReceivedPacketCount,
                    frameNumber);
        }
        mCurrentFrame = frameNumber;
        mReceivedPacketCount = 0;
        std::fill(mReceivedPacketMask.begin(), mReceivedPacketMask.end(), false);
    }

    mReceivedPacketCount += 1;
    mReceivedPacketMask[packetNumber] = true;

    if (mReceivedPacketCount == 1) {
        depthImage.fill(0);
    }

    uint32_t pixelIndex = packetNumber * pixelsPerPacket;
    for (uint32_t i = headerSize; i < got - crcSize; i += bytesPerPixel) {
        depthImage[pixelIndex] = static_cast<uint16_t>((packet[i] << 8) | packet[i + 1]);
        pixelIndex += 1;
    }

    return std::make_tuple(mReceivedPacketCount == packetsPerImage, frameNumber);
}
