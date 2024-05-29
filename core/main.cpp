// Copyright (c) 2022 Richard Kelsey. All rights reserved.

#include "camera.hpp"
#include "command.hpp"
#include "fusion.hpp"
#include "robot.hpp"
#include "socket.hpp"
#include "cruft.hpp"
#include "display.hpp"
#include "render.hpp"
#include "raster.hpp"
#include "registration.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <GLFW/glfw3.h>
#include <nlohmann/json.hpp>
#include "ThreadPool.h"

#include <array>
#include <cstdio>
#include <iostream>
#include <fstream>
#include <functional>

#include <fcntl.h> 
#include <netdb.h>
#include <netinet/ip.h>
#include <semaphore.h>
#include <sys/socket.h>
#include <sys/timeb.h>
#include <sys/un.h>
#include <unistd.h>
#include <stdio.h>

extern void initializeRobotProjection(const nlohmann::json& config, 
                                      const std::string& configDirectory);
extern void addRobotProjection(const Robot& robot, const std::vector<float>& pose);

int coreToSimFd = -1;

//----------------------------------------------------------------

struct Pose {
    uint32_t frameNumber;
    std::vector<float> pose;

    Pose(size_t jointCount):
        frameNumber(0xFFFFFFFF),
        pose(jointCount, 0)
    {
    }
};

static const uint32_t poseCount = 8;

static std::vector<Pose> robotPoses;
static uint32_t nextRobotFrame = 0;
static uint32_t latestRobotFrame = 0;
static uint32_t robotFrameDelta = 0;

static void robotCommand(const CommandArguments &arguments) 
{
    robotPoses[nextRobotFrame].frameNumber = arguments.int32At(0);
    latestRobotFrame = arguments.int32At(0);
    for (uint32_t i = 0; i < 6; i++) {
        robotPoses[nextRobotFrame].pose[i] = ((float) arguments.int32At(i + 1)) / 10000.0f;
    }
    // fprintf(stderr, "[got frame %d]\n", robotPoses[nextRobotFrame].frameNumber);
    nextRobotFrame = (nextRobotFrame + 1) % poseCount;
}

static void initializeRobotPoses(size_t jointCount)
{
    for (uint16_t i; i < poseCount; i++) {
        robotPoses.emplace_back(jointCount);
    }
}

static std::vector<float> *getRobotPose(const uint32_t frameNumber)
{
    robotFrameDelta = latestRobotFrame - frameNumber;
    for (uint32_t i = 0; i < poseCount; i++) {
        if (robotPoses[i].frameNumber == frameNumber) {
            return &robotPoses[i].pose;
        }
    }
    fprintf(stderr, "[want %d have %d %d %d %d %d %d %d %d]\n", 
            frameNumber,
            robotPoses[0].frameNumber,
            robotPoses[1].frameNumber,
            robotPoses[2].frameNumber,
            robotPoses[3].frameNumber,
            robotPoses[4].frameNumber,
            robotPoses[5].frameNumber,
            robotPoses[6].frameNumber,
            robotPoses[7].frameNumber);
    return nullptr;
}

static const std::vector<CommandSpec>simCommandTable = {
    { "robot", "iiiiiii",  robotCommand }
};

static CommandInterpreter simCommands = {simCommandTable};

void commandErrorHandler(const char *command,
        CommandError error,
        const char *message)
{
    fprintf(stderr, "Command error: '%s' %d '%s'\n", command, (int) error, message);
}

//----------------------------------------------------------------

static bool freeze = false;

static void freezeCommand(const CommandArguments &arguments)
{
    freeze = arguments.int32At(0) != 0;
}

uint8_t robotSpriteLevel = 0;

static void robotSpriteCommand(const CommandArguments &arguments)
{
    robotSpriteLevel = arguments.int32At(0);
}

Show floorShow;
uint32_t layerToShow;
uint8_t cameraIndex;

static void showCommand(const CommandArguments &arguments)
{
    floorShow = static_cast<Show>(arguments.int32At(0));
    layerToShow = arguments.int32At(1);
    cameraIndex = arguments.int32At(2);
}

const std::vector<CommandSpec>displayCommandTable = {
    { "freeze", "i",   freezeCommand },
    { "sprite", "i",   robotSpriteCommand },
    { "show",   "iii", showCommand }
};

CommandInterpreter displayCommands = {displayCommandTable};

//----------------------------------------------------------------

class FrameInput {
public:
    uint8_t index;  // which camera we are reading from

    // the most recent complete frame
    uint32_t frameNumber = -1;

    // the frame being read
    uint32_t nextFrameNumber = -1;
    bool nextFrameComplete = false;

    CameraSink sink; // where we read from

    Eigen::Vector3f position;
    Eigen::Affine3f cameraToWorld;
    Eigen::Affine3f worldToCamera;
    
    FrameInput(uint8_t index, int clientFd, nlohmann::json& config) :
        index(index),
        sink(clientFd),
        position(toVec<Eigen::Vector3f>(config["position"])),
        cameraToWorld(lookAt(position,
                             toVec<Eigen::Vector3f>(config["target"]),
                             toVec<Eigen::Vector3f>(config["up"]))),
        worldToCamera(cameraToWorld.inverse())
    {
    }

    void startNextImage(uint32_t newFrameNumber) {
        nextFrameComplete = false;
        frameNumber = (nextFrameNumber < newFrameNumber
                       ? newFrameNumber
                       : nextFrameNumber);
    }

    void read(Image16& depthImage) {
        std::tie(nextFrameComplete, nextFrameNumber) = sink.read(depthImage);
    }
};

static std::vector<FrameInput> inputs;

// One lock for the previous frame, held by the image processor, and another
// for the next frame, held by the image reader.  When both the processor and
// reader are finished the reader gets both locks, swaps the image buffers, 
// and releases the lock corresponding to the new complete image.
static std::mutex imageMutexes[2];

// The current set of depth images and their frame number.
static std::vector<Image16> *depthImages;
uint32_t currentFrameNumber;
static bool haveImages = false;  // are there new images to display

static uint32_t readNextFrame(std::vector<Image16>& depthImages);

// Started with imageMutexes[0] already locked.
static void readImages()
{
    uint8_t mutexIndex = 0;

    // We read into one set of images while processing the other and then swap.
    std::vector<Image16> depthImages0(inputs.size());
    std::vector<Image16> depthImages1(inputs.size());
    bool useZero = true;

    while (true) {
        currentFrameNumber = readNextFrame(useZero ? depthImages0 : depthImages1);
        imageMutexes[1 - mutexIndex].lock();
        // We have both the previous and next frame locks and can swap buffers.
        for (auto& input : inputs) {
            input.startNextImage(currentFrameNumber + 1);
        }
        depthImages = useZero ? &depthImages0 : &depthImages1;
        useZero = !useZero;
        haveImages = true;
        imageMutexes[mutexIndex].unlock();
        mutexIndex = 1 - mutexIndex;
    }
}

static uint32_t readNextFrame(std::vector<Image16>& depthImages)
{
    while (true) {
        uint32_t frameNumber = inputs[0].nextFrameNumber;
        for (auto& input : inputs) {
            // Needs to handle rollover.
            if (frameNumber < input.nextFrameNumber) {
                frameNumber = input.nextFrameNumber;
            }
        }
        bool done = true;
        bool print = false;
        for (auto& input : inputs) {
            if (input.nextFrameNumber != frameNumber) {
                // This is needed to sychronize the inputs on startup,
                input.startNextImage(frameNumber);
            }
            if (! (input.nextFrameComplete
                   && input.nextFrameNumber == frameNumber)) {
                input.read(depthImages[input.index]);
                done = false;
            }
            if (input.nextFrameNumber != frameNumber) {
//                    print = true;
            }
        }
        if (print) {
            fprintf(stderr, "[frame %d", frameNumber);
            for (auto& input : inputs) {
                fprintf(stderr, " %d:%c", input.nextFrameNumber,
                        input.nextFrameComplete ? 'y' : 'n');
            }
            fprintf(stderr, "]\n");
        }
        if (done) {
            return frameNumber;
        }
    }
}

// Called by the display code to update raw image displays.
void updateRawImages(bool refresh)
{
    if (!(haveImages || refresh)) {
        return;  // no new images available
    }
    for (uint32_t i = 0; i < inputs.size(); i++) {
        // Multiply by 64 to change the range from 0-10m to the full
        // display range of 0-65k.
        auto& depths = (*depthImages)[i];
        Image16 image;
        for (uint32_t j = 0; j < imagePixelCount; j++) {
            image[j] = depths[j] << 6;
        }
        updateDepthTexture(image, i);
    }
    haveImages = false; // don't redraw until we have a new set of images
}

//----------------------------------------------------------------
// The 'robotLine' is a line on the floor display showing the shortest
// distance between the robot's possible future positions and any
// unidentified blob that's large enough to contain a person.

static bool haveRobotLine = false;
static uint32_t robotLineX0;
static uint32_t robotLineY0;
static uint32_t robotLineX1;
static uint32_t robotLineY1;

void clearRobotLine()
{
    haveRobotLine = false;
}

void setRobotLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    robotLineX0 = x0;
    robotLineY0 = y0;
    robotLineX1 = x1;
    robotLineY1 = y1;
    haveRobotLine = true;
}

//----------------------------------------------------------------
// Keeping timing statistics.
static uint32_t lastImageTime = 0;
static uint32_t lastFrameNumber = -2; // currentFrameNumber starts at -1
static uint32_t frameCount = 0;
static uint32_t statFrameCount = 0;
static uint32_t imageLatencyTimes = 0;
static uint32_t traceTimes = 0;
static uint32_t markTimes = 0;
static uint32_t robotRenderTimes = 0;
static uint32_t projectTimes = 0;
static uint32_t showTimes = 0;
static uint32_t occupyTimes = 0;
static uint32_t totalTimes = 0;

// At top level so that we don't have to keep reallocating space for the
// contents.
static std::vector<uint16_t> blobData{};
std::vector<uint32_t> floorImageData{};

static void processImageSet(ThreadPool &threadPool, Robot& robot) {

    if (!freeze && currentFrameNumber != lastFrameNumber + 1) {
        fprintf(stderr, "[jump %d to %d]\n", lastFrameNumber, currentFrameNumber);
    }
    lastFrameNumber = currentFrameNumber;
    std::vector<float> *pose = getRobotPose(currentFrameNumber);
    if (pose == nullptr) {
        // nothing to do this round
    } else {
        // Update the background scanning, if it's enabled.
        backgroundScan(robot, *pose, (*depthImages)[0]);

        uint32_t startTime = timeMs();
        uint32_t imageLatencyTime = startTime - lastImageTime;

        // Start calculating the robot's possible future positions.  This does
        // not depend on the depth images and can be done in parallel with
        // image processing.
        std::future<uint32_t> projectionFuture(threadPool.enqueue([&]() {
            uint32_t start = timeMs();
            clearFloorImage();
            addRobotProjection(robot, *pose);
            return timeMs() - start;
        }));

        // Use the depth images to mark voxels as occupied, empty, or unknown.
        traceRays(threadPool, const_cast<const std::vector<Image16>&>(*depthImages));
        uint32_t traceTime = timeMs() - startTime;

        // Use the robot's joint angles to mark the voxels it should fill.
        uint32_t sectionStartTime = timeMs();
        renderRobot(robot, *pose);
        uint32_t robotRenderTime = timeMs() - sectionStartTime;

        // Merge adjacent voxels of the same type into blobs.
        sectionStartTime = timeMs();
        Blob *allBlobs = mergeBlobs(threadPool);
        uint32_t markTime = timeMs() - sectionStartTime;

        // Get the result of the robot future projection.
        uint32_t projectTime = projectionFuture.get();
        sectionStartTime = timeMs();

        // Draw the perspective image.  This should be done by a separate
        // process, as it takes a while and isn't part of the core functionality.
        allBlobs->writeToVector(blobData);
        // printf("blobData %ld uint16s\n", blobData.size());
        renderBlobs(blobData);

        // Write out the images as point clouds if asked for by the GUI.
        if (printClouds) {
            for (uint32_t i = 0; i < inputs.size(); i++) {
                writePointCloud(i, (*depthImages)[i]);
            }
            printClouds = false;
        }

        uint32_t occupyTime = timeMs() - sectionStartTime;
        sectionStartTime = timeMs();

        // Update the floor image.  This also isn't part of the core
        // functionality, but it is quick.
        showFloor(allBlobs, floorImageData);

        uint32_t showTime = timeMs() - sectionStartTime;

        uint32_t totalTime = timeMs() - startTime;

        imageLatencyTimes += imageLatencyTime;
        traceTimes += traceTime;
        markTimes += markTime;
        projectTimes += projectTime;
        robotRenderTimes += robotRenderTime;
        showTimes += showTime;
        occupyTimes += occupyTime;
        totalTimes += timeMs() - startTime;
        // fprintf(stdout, "[total time %d robot delta: %d]\n", timeMs() - startTime, robotFrameDelta);
        statFrameCount += 1;
        if (statFrameCount == 200) {
            fprintf(stderr, "[%d frame average latency %.4f total %.4f trace %.4f robot %.4f mark %.4f project %.4f occupy %.4f show %.4f]\n",
                    frameCount,
                    (float) imageLatencyTimes / statFrameCount,
                    (float) totalTimes / statFrameCount,
                    (float) traceTimes / statFrameCount,
                    (float) robotRenderTimes / statFrameCount,
                    (float) markTimes / statFrameCount,
                    (float) projectTimes / statFrameCount,
                    (float) occupyTimes / statFrameCount,
                    (float) showTimes / statFrameCount);
            statFrameCount = 0;
            imageLatencyTimes = 0;
            traceTimes = 0;
            markTimes = 0;
            robotRenderTimes = 0;
            projectTimes = 0;
            showTimes = 0;
            occupyTimes = 0;
            totalTimes = 0;
        }
        lastImageTime = timeMs();
    }
    frameCount += 1;
}

extern void distanceTest();

int main(int argc, char **argv)
{
    bool camerasOnly = false;

    if (argc < 1) {
       fprintf(stderr,"usage %s config-file\n", argv[0]);
       exit(0);
    }

    if (strcmp(argv[1], "--cameras") == 0) {
        argv += 1;
        camerasOnly = true;
    }

    coreToSimFd = openSocket((char *) "rk-core-to-sim", false);
    int displayServerFd = openSocket((char *) "rk-display-to-core", true);
    int displayFd = -1;

    std::string configFileName = argv[1];
    std::string configDirectory =
            configFileName.substr(0, configFileName.find_last_of("/"));
    nlohmann::json config = readConfig(configFileName);

    bool testMode = config.count("test") && config["test"];

    setCellDimensions(config);
    initializeEngine(config, configDirectory);
    initializeRobotProjection(config, configDirectory);
    renderViewpoint = toVec<Eigen::Vector3f>(config["viewpoint"]);
    renderInit(config, configDirectory);

    float robotScaleFactor = 1.0f;
    if (config["robot"]["scale"] != nullptr) {
      robotScaleFactor = config["robot"]["scale"].get<float>();
    }
    Robot robot((configDirectory
                 + "/"
                 + config["robot"]["urdf"].get<std::string>()),
                robotScaleFactor);

    fprintf(stderr, "[%ld links]\n", robot.getLinkCount());
    initializeRobotPoses(robot.getControlCount());

    if (!initDisplay()) {
      return 1;
    }

    ThreadPool threadPool(14);

    auto& cameras = config["cameras"];
    for (int i = 0; i < cameras.size(); i++) {
        auto& camera = cameras[i];

        // Ignore background-only cameras.
        if (camera.count("port") == 0) {
            continue;
        }

        uint16_t port = camera["port"].get<uint16_t>();
        int clientFd = socket(AF_INET, SOCK_DGRAM, 0);
        if (clientFd < 0)  {
            perror("socket() error");
            exit(0);
        }
        
        sockaddr_in myaddr; 
        memset((char *)&myaddr, 0, sizeof(myaddr));
        myaddr.sin_family = AF_INET;
        myaddr.sin_addr.s_addr = htonl(INADDR_ANY);
        myaddr.sin_port = htons(port);
        if (bind(clientFd, (sockaddr *)&myaddr, sizeof(myaddr)) < 0) { 
            perror("bind failed");
            exit(0);
        }

        inputs.emplace_back(i, clientFd, camera);
    }

    lastImageTime = timeMs();

    imageMutexes[0].lock();
    std::thread imageReader(readImages);

    uint8_t mutexIndex = 0;

    fprintf(stderr, "[line %d]\n", __LINE__);
    while (checkDisplay()) {
        simCommands.readCommands(coreToSimFd);

        if (displayFd == -1) {
            displayFd = acceptClient(displayServerFd);
        } else {
            displayCommands.readCommands(displayFd);
        }

        if (imageMutexes[mutexIndex].try_lock()) {
            if (!camerasOnly) {
                processImageSet(threadPool, robot);
            }
            update_display(floorImageData);
            imageMutexes[mutexIndex].unlock();
            mutexIndex = 1 - mutexIndex;
        }
    }


    glfwTerminate();
    return 0;
} 
