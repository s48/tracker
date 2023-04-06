// Copyright (c) 2022 Richard Kelsey. All rights reserved.

// Simulation app.
//
// Keeps track of the robot pose and avatar locations and
// generates depth images.

#include "camera.hpp"
#include "command.hpp"
#include "constant.hpp"
#include "cruft.hpp"
#include "mesh.hpp"
#include "raster.hpp"
#include "robot.hpp"
#include "socket.hpp"
#include "util.hpp"

#include <iostream>
#include <fstream>
#include <chrono>
#include <cstring>

#include <optionparser.h>
#include <signal.h>
#include <unistd.h>
#include <time.h>
#include <sys/timeb.h>

#include "ThreadPool.h"
#include <nlohmann/json.hpp>
#include "gsl-lite.hpp"
#include <Eigen/Core>
#include <Eigen/Geometry>

bool exitFlag = false;

// Exit handler
void exit_handler(int)
{
    exitFlag = true;
}

static void signalHandler(int /* signo */)
{
}

//----------------------------------------------------------------
// Interactive inputs, which are received from the GUI as text commands.

static char mouseMode = 0;
static int32_t previousMouse[2];

static void modeCommand(const CommandArguments &arguments) 
{
    mouseMode = arguments.int32At(0);
    fprintf(stderr, "[mode %d]\n", mouseMode);
}

// The avatar is a human figure that can be moved around the scene.
static Eigen::Affine3f avatarPosition;
static float avatarDelta[2];

// There are three movable screens that can be used to block cameras
// from particular points.
static std::array<bool, 3> screenVisible;
static std::array<Eigen::Affine3f, 3> screenPositions;

// Robot movement.
static float robotDelta[6];
static int32_t robotRotation = -1;  // no rotation

bool debugPrint = false;

static void debugCommand(const CommandArguments &arguments) 
{
    debugPrint = true;
}

static void mouseCommand(const CommandArguments &arguments) 
{
    int32_t x = arguments.int32At(0);
    int32_t y = arguments.int32At(1);
    int32_t scroll = arguments.int32At(2);

    switch (mouseMode) {
    case 0:  // nothing
    case 1:  // nothing
        break;
    case 2:  // avatar
    case 6:  // screen
    case 7:  // screen
    case 8:  // screen
        Eigen::Affine3f *matrix;
        if (mouseMode == 2) {
            matrix = &avatarPosition;
        } else {
            matrix = &screenPositions[mouseMode - 6];
            screenVisible[mouseMode - 6] = true;
        }
        if (x != previousMouse[0]
                || y != previousMouse[1]) {
            // This works out for the way the avatar is facing.
            float deltaX = ((float) (previousMouse[0] - x)) / 100.0f;
            float deltaY = ((float) (previousMouse[1] - y)) / 100.0f;
            deltaX = std::max(-0.5f, std::min(0.5f, deltaX));
            deltaY = std::max(-0.5f, std::min(0.5f, deltaY));
            *matrix = Eigen::Translation3f(-deltaX, deltaY, 0.0f) * *matrix;
        }
        if (scroll != 0) {
          *matrix *= Eigen::AngleAxisf(0.3f * scroll, Eigen::Vector3f{0.0f, 0.0f, 1.0f});
        }
        break;
    case 3:  // avatar arms and rotation
        avatarDelta[0] += (float) (x - previousMouse[0]) / 200.0f;
        avatarDelta[1] -= (float) (y - previousMouse[1]) / 200.0f;
        if (scroll != 0) {
            avatarPosition *= Eigen::AngleAxisf(0.3f * scroll, Eigen::Vector3f{0.0f, 0.0f, 1.0f});
        }
        break;
    case 4:  // robot first three joints
        // fprintf(stderr, "[mouse %d %d %d]\n", x, y, z);
        robotDelta[0] += (float) (x - previousMouse[0]) / 200.0f;
        // the interaction is more intuitive with Y movement negated
        robotDelta[1] -= (float) (y - previousMouse[1]) / 200.0f;
        robotDelta[2] -= (float) scroll / 10.0f;
        break;
    case 5:  // robot last three joints
        robotDelta[3] += (float) (x - previousMouse[0]) / 200.0f;
        robotDelta[4] += (float) (y - previousMouse[1]) / 200.0f;
        robotDelta[5] += (float) scroll / 10.0f;
        break;
    default:
        fprintf(stderr, "[no mode %d]\n", mouseMode);
    }

    // scroll is a delta, +1 or -1, not a position, so it can be used directly
    previousMouse[0] = x;
    previousMouse[1] = y;
}

// Rotate the robot by a fixed amount.
static void rotateCommand(const CommandArguments &arguments)
{
    robotRotation = arguments.int32At(0);
}

// Adjust a camera's aim.  See below.
static void cameraCommand(const CommandArguments &arguments);

static bool sendData = false;
static void stopCommand(const CommandArguments &arguments) 
{
    sendData = false;
}

static void startCommand(const CommandArguments &arguments) 
{
    sendData = true;
}

const std::vector<CommandSpec>displayCommandTable = {
  { "mode",   "i",   modeCommand },
  { "mouse",  "iii", mouseCommand },
  { "rotate", "i",   rotateCommand },
  { "camera", "iii", cameraCommand },
  { "start",  "",    startCommand },
  { "stop",   "",    stopCommand },
  { "debug",  "",    debugCommand }
};

CommandInterpreter displayCommands = {displayCommandTable};

const std::vector<CommandSpec>coreCommandTable = {
  { "rotate", "i",   rotateCommand }
};

CommandInterpreter coreCommands = {coreCommandTable};

void commandErrorHandler(const char *command,
                         CommandError error,
                         const char *message)
{
  fprintf(stderr, "Command error: '%s' %d '%s'\n", command, (int) error, message);
}

//----------------------------------------------------------------

// The index offsets of the eight pixels around a given pixel.
static const int32_t offsets[] = { -1, 1, - (int32_t) imageWidth, imageWidth,
                                   1 - (int32_t) imageWidth,  1 + imageWidth,
                                   -1 - (int32_t) imageWidth, -1 + imageWidth };


struct Scene {
    Robot robot;
    std::vector<float> robotPose;
    Robot avatar;
    std::vector<float> avatarPose;
    Mesh screen;
    Mesh background;

    Scene(const std::string& robotFile,
          float robotScaleFactor,
          const std::string& avatarFile,
          Mesh& background,
          Mesh& screen) :
        robot(robotFile, robotScaleFactor),
        robotPose(robot.getControlCount(), 0),
        avatar(avatarFile),
        avatarPose(avatar.getControlCount(), 0),
        screen(std::move(screen)),
        background(std::move(background))
    {
    }
};

class Camera
{
public:
    uint32_t index;
    Eigen::Vector3f location;
    Eigen::Vector3f rpy;
    Eigen::Affine3f cameraToWorld;
    Eigen::Affine3f worldToCamera;
    const Scene& scene;
    CameraSource source;

    ImageF backgroundDepth;
    ImageF backgroundCosines;
    Image8 backgroundObjectTypes;

    ImageF depthBuffer;
    ImageF cosineBuffer;
    Image8 objectTypeBuffer;
    ImageF frameBuffer;

    random_data randomData;
    uint8_t randomState[16];

    Camera(uint32_t index,
           uint16_t port,
           const Eigen::Vector3f& location,
           const Eigen::Vector3f& rpy,
           const Scene& scene) :
        index(index),
        source(port),
        location(location),
        rpy(rpy),
        scene(scene)
    {
        initstate_r(10, (char *) randomState, sizeof(randomState), &randomData);
        resetAim();
    }

private:
    int32_t localRandom()
    {
        int32_t result;
        random_r(&randomData, &result);
        return result;
    }

    // For simulating a Kinect 2 response from as described in
    // https://arxiv.org/pdf/1608.05204.  That was the only IR TOF data I
    // could find.  It made the image noisier but didn't have much overall
    // effect.  This could be changed to also introduce errors in depth.
    const float requiredEnergy = 0.020;
    const float zeroErrorEnergy = 0.025;
    const float allErrorEnergy = 0.005;

    // The render code stores a one byte 'type' for each pixel.  We use
    // this for the surface's albedo.  From the paper, 80% is a typical
    // albedo.  Some materials, including some clothing, are nearly
    // completely absorbant.
    const uint8_t reflectiveAlbedo = 0.8f * 255.0f;
    const uint8_t absorbentAlbedo = 0;

    float measurement(float depth, uint8_t scaledAlbedo, float cosine)
    {
        float albedo = float(scaledAlbedo) / 255.0f;
        float energy = pow(albedo * (0.75f * cosine + 0.25f) / (depth * depth), 0.87f);
        float delta = ((localRandom() % 100) - 50) * requiredEnergy / 250.0f;
        if (requiredEnergy < energy + delta) {
            return depth;
            // Simulate random inaccuracies.  This is just a guess,
            // and would require some tuning of the voxel hit/miss
            // parameters.
            // return depth + ((localRandom() % 9) - 4) * 0.005;
        } else {
            return 0.0f;
        }
    }

    // The Kinect software has a filter to remove 'flying' pixels that
    // appear on discontinuities.  These two loops set to zero pixels that
    // lie on disontinuities.  The first checks rows and the second checks
    // columns.  Other cameras may do something different.

    void removeFlyingPixels()
    {
        for (int y = 0; y < imageHeight - 2; y++) {
            float *depthRow = depthBuffer.data() + (y + 1) * imageWidth;
            float *frameRow = frameBuffer.data() + y * imageWidth;
            float delta0 = fabs(depthRow[0] - depthRow[1]);
            for (int x = 1; x < imageWidth - 1; x++) {
                float delta1 = fabs(depthRow[x] - depthRow[x + 1]);
                if (depthRow[x] != 0.0
                    && depthRow[x - 1] != 0.0
                    && depthRow[x + 1] != 0.0
                    && 0.03f < fabs(delta0 - delta1)) {
                    frameRow[x + ((localRandom() % 3) * imageWidth)] = 0.0;
                }
                delta0 = delta1;
            }
        }

        for (int x = 1; x < imageWidth - 1; x++) {
            float* lastLoc = &depthBuffer[x];
            float delta0 = fabs(lastLoc[0] - lastLoc[imageWidth]);
            for (int y = 0; y < imageHeight - 1; y++) {
                float* loc = lastLoc + imageWidth;
                float* nextLoc = loc + imageWidth;
                float delta1 = fabs(*loc - *nextLoc);
                if (*lastLoc != 0.0
                    && *loc != 0.0
                    && *nextLoc != 0.0
                    && 0.03f < fabs(delta0 - delta1)) {

                    uint32_t offset = lastLoc - depthBuffer.data();
                    frameBuffer[offset + localRandom() % 3] = 0.0;
                }
                lastLoc = loc;
                delta0 = delta1;
            }
        }
    }

public:
    void resetAim() {
        cameraToWorld = makeRpyXyzTransform(rpy, location);
        worldToCamera = cameraToWorld.inverse();
        renderBackground();
    }

    void renderBackground()
    {
        backgroundDepth.fill(maxCameraRangeInM);
        render(scene.background,
               worldToCamera,
               backgroundDepth,
               backgroundCosines,
               backgroundObjectTypes,
               reflectiveAlbedo);
        // We move the robot up a little.  This should be in the config file.
        render(scene.robot.getBaseMesh(),
               worldToCamera * Eigen::Translation3f(0.0, 0.0, 0.3),
               backgroundDepth,
               backgroundCosines,
               backgroundObjectTypes,
               reflectiveAlbedo);
    }
    
    void renderMesh(const Mesh& mesh,
                    const Eigen::Affine3f& worldToCamera,
                    uint8_t albedo)
    {
        render(mesh, worldToCamera, depthBuffer, cosineBuffer, objectTypeBuffer, albedo);
    }

    void renderScene(const uint32_t frameNumber, const Scene& scene)
    {
        uint32_t renderStart = timeMs();
        memcpy(depthBuffer.data(), backgroundDepth.data(), sizeof(backgroundDepth));

        // renderTime += timeMs() - renderStart;
        // renderStart = timeMs();

        auto transforms = scene.robot.makeLinkTransforms(scene.robotPose);
        for (uint32_t i = 0; i < scene.robot.getLinkCount(); i++) {
            if (scene.robot.getLink(i).mesh != nullptr) {
                auto transform = (worldToCamera
                                  * Eigen::Translation3f(0.0f, 0.0f, 0.254f)
                                  * transforms[i]);
                renderMesh(*scene.robot.getLink(i).mesh, transform, reflectiveAlbedo);
            }
        }

        uint32_t time0 = timeMs();
        uint32_t time1;
        uint32_t time2;
        uint32_t time3;

        // robotTime += timeMs() - renderStart;
        // renderStart = timeMs();

        // render the avatar
        auto avatarTransforms = scene.avatar.makeLinkTransforms(scene.avatarPose);
        auto avatarTransform = worldToCamera * avatarPosition;
        auto mesh = scene.avatar.getBaseMesh();
        renderMesh(mesh, avatarTransform, reflectiveAlbedo);
        for (uint32_t i = 0; i < scene.avatar.getLinkCount(); i++) {
            auto transform = avatarTransform * avatarTransforms[i];
            auto& mesh = scene.avatar.getLink(i).mesh;
            // make the arms invisible to the camera
            renderMesh(*mesh, transform, absorbentAlbedo);
        }

        time1 = timeMs();
        // render the four screens
        for (uint32_t i = 0; i < 3; i++) {
            if (screenVisible[i]) {
                renderMesh(scene.screen,
                           worldToCamera * screenPositions[i],
                           reflectiveAlbedo);
            }
        }
        time2 = timeMs();
        // Translate the depth buffer to a camera image.
        for (int i = 0; i < imageHeight * imageWidth; i++) {
            float depth = depthBuffer[i];
            if (depth == maxCameraRangeInM) {
                // Kinects seem to occasionally return distances for out-of-range
                // pixels.  This is a poor simulation of this behavior.
                frameBuffer[i] = (16 < (localRandom() & 0xFFFF)
                                  ? 0.0
                                  : maxCameraRangeInM);
            } else {
                float cosine;
                uint8_t albedo;
                if (depth == backgroundDepth[i]) {
                    cosine = backgroundCosines[i];
                    albedo = backgroundObjectTypes[i];
                } else {
                    cosine = cosineBuffer[i];
                    albedo = objectTypeBuffer[i];
                }

                if (albedo == 0) {
                    frameBuffer[i] = 0.0f;  // no data returned
                } else if (false) {
                    // more accurate simulation
                    // Using this means adjusting the camera angles.
                    frameBuffer[i] = measurement(depth, albedo, cosine);
                } else {
                    frameBuffer[i] = depth;
                }
            }
        }
        time3 = timeMs();
        removeFlyingPixels();
        uint32_t time4 = timeMs();

        // renderTime += timeMs() - renderStart;
        // renderStart = timeMs();
        source.sendImage(frameNumber, frameBuffer);

        // fprintf(stderr, "[%d: end %d after %d (%d %d %d %d %d %d)]\n", port, frameNumber, timeMs() - renderStart,
        //          time0 - renderStart,
        //          time1 - time0,
        //          time2 - time1,
        //          time3 - time2,
        //          time4 - time3,
        //          timeMs() - time4);
    }
};

// Suitable for calling with a future.
static int renderScene(Camera* camera, const uint32_t frameNumber, const Scene& scene)
{
    camera->renderScene(frameNumber, scene);
    return 0;
}

static bool adjustCameras = false;
static std::vector<Camera> cameras;
static const float cameraStep = 0.10f; // 10cm
static const float cameraAngleStep = M_PI / 64;

static void cameraCommand(const CommandArguments &arguments)
{
    if (!adjustCameras) {
        return;
    }

    int32_t index = arguments.int32At(0);
    int32_t axis = arguments.int32At(1);
    float delta = arguments.int32At(2) * cameraStep;
    float angleDelta = arguments.int32At(2) * cameraAngleStep;
    if (index == -1) {
        for (auto& camera : cameras) {
            printf("%d: \"location\": [%.6f, %.6f, %.6f],\n",
                   camera.index,
                   camera.location[0],
                   camera.location[1],
                   camera.location[2]);
            printf("   \"rpy\": [%.6f, %.6f, %.6f],\n",
                   camera.rpy[0],
                   camera.rpy[1],
                   camera.rpy[2]);
        }
        return;
    }
    if (index < 0 || cameras.size() <= index) {
        return;
    }
    auto& camera = cameras[index];
    switch (axis) {
    case 'X': camera.location[0] += delta; break;
    case 'Y': camera.location[1] += delta; break;
    case 'Z': camera.location[2] += delta; break;
    case 'r': camera.rpy[0] += angleDelta; break;
    case 'p': camera.rpy[1] += angleDelta; break;
    case 'y': camera.rpy[2] += angleDelta; break;
    }
    camera.resetAim();
}

//----------------------------------------------------------------
enum optionIndex { UNKNOWN, HELP, BACKGROUNDS, CONFIG_FILE, CADENCE, ADJUST };

const option::Descriptor usage[] = 
    {{UNKNOWN, 0, "", "", option::Arg::None,
      "USAGE: foo [options]\n\n"
      "Options:"},
     {HELP,        0, "h", "help",       option::Arg::None, "--help, -h  \tPrint usage and exit."},
     {BACKGROUNDS, 0, "", "backgrounds", option::Arg::None,
      "--backgrounds \tWrite background files."},
     {CADENCE, 0, "", "cadence", option::Arg::Optional,
      "--cadence \ttime in ms."},
     {CONFIG_FILE, 0, "c", "configfile", option::Arg::Optional,
      "--configfile filename \tconfiguration file."},
     {ADJUST, 0, "", "adjust", option::Arg::None,
      "--adjust \tadjust camera aim."},
     {0, 0, nullptr, nullptr, nullptr, nullptr}};

int main(int argc, char **argv)
{
    CommandLine cl(argc, argv, usage);
    if (!cl.verify(UNKNOWN, HELP)) {
        return 0;
    }
    const std::vector<option::Option> options = cl.options();

    if (options[CONFIG_FILE] == nullptr) {
        fprintf(stderr, "No config file specfied.");
        return 0;
    }

    uint32_t cadenceMs = 33;
    if (options[CADENCE] != nullptr) {
        cadenceMs = atoi(options[CADENCE].arg);
    }

    bool writeBackgrounds = options[BACKGROUNDS] != nullptr;
    adjustCameras = options[ADJUST] != nullptr;

    const std::string configFileName = options[CONFIG_FILE].arg;
    const std::string configDirectory = 
            configFileName.substr(0, configFileName.find_last_of("/"));
    nlohmann::json config = readConfig(configFileName);

    bool testMode = config.count("test") && config["test"];

    auto& avatarConfig = config["avatar"];
    avatarPosition = Eigen::Translation3f(toVec<Eigen::Vector3f>(avatarConfig["position"]));
    avatarPosition *= Eigen::AngleAxisf(avatarConfig["rotation"], Eigen::Vector3f{0.0f, 0.0f, 1.0f});

    initDepthCorrection();

    // Exit signal catching
    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = exit_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, nullptr);

    if (signal(SIGUSR1, signalHandler) == SIG_ERR) {
        perror("signal() failed");
        return 0;
    }

    Mesh screen;
    Mesh background;

    if (testMode) {
        /*
        background.addTriangle(Eigen::Vector3f(0.0f, 0.0f, 0.0f),
                Eigen::Vector3f(0.020f, 0.0f, 0.0f),
                Eigen::Vector3f(0.020f, 0.020f, 0.0f));

        background.addTriangle(Eigen::Vector3f(1.0f, 0.0f, 0.0f),
                Eigen::Vector3f(1.020f, 0.0f, 0.0f),
                Eigen::Vector3f(1.020f, 0.020f, 0.0f));

        background.addTriangle(Eigen::Vector3f(2.0f, 0.0f, 0.0f),
                Eigen::Vector3f(2.020f, 0.0f, 0.0f),
                Eigen::Vector3f(2.020f, 0.020f, 0.0f));

        background.addTriangle(Eigen::Vector3f(0.0f, 1.0f, 0.0f),
                Eigen::Vector3f(0.020f, 1.0f, 0.0f),
                Eigen::Vector3f(0.020f, 1.020f, 0.0f));

        background.addTriangle(Eigen::Vector3f(-3.685f, -1.851f, 0.0f),
                Eigen::Vector3f(-3.645f, -1.851f, 0.0f),
                Eigen::Vector3f(-3.645f, -1.811f, 0.0f));

        background.addTriangle(Eigen::Vector3f(-3.5f, -2.0f, 2.0f),
                Eigen::Vector3f(-3.5f, -2.04f, 2.04f),
                Eigen::Vector3f(-3.5f, -2.04f, 2.04f));

        background.addTriangle(Eigen::Vector3f(-3.5f, -1.0f, 1.0f),
                Eigen::Vector3f(-3.5f, -1.04f, 1.04f),
                Eigen::Vector3f(-3.5f, -1.04f, 1.04f));
        */

        /*
        background.addTriangle(Eigen::Vector3f(-3.50, -2.0, 3.0),
                Eigen::Vector3f(-3.50, -1.95, 2.95),
                Eigen::Vector3f(-3.50, -1.95, 3.0));
        background.addTriangle(Eigen::Vector3f(-3.50, -2.0, 2.0),
                Eigen::Vector3f(-3.50, -1.95, 1.95),
                Eigen::Vector3f(-3.50, -1.95, 2.0));
        */
        background.addBox(Eigen::Vector3f(-3.50, -2.0, 0.0), Eigen::Vector3f(0.04, 4.0, 3.0));
        // addTable(background, 0.75, -0.5);
    } else {
        background.readStlFile(configDirectory
                               + "/"
                               + config["layout"].get<std::string>());
        screen.addBox(Eigen::Vector3f(-0.5, -0.025, 0.3), Eigen::Vector3f(1.0, 0.05, 1.5));
    }

    screenPositions[0] = Eigen::Translation3f(1.0, -2.8, 0.0);
    screenPositions[1] = Eigen::Translation3f(1.2, -3.0, 0.0)
      * Eigen::AngleAxisf(0.15f, Eigen::Vector3f(0.0f, 0.0f, 1.0f));
    screenPositions[2] = Eigen::Translation3f(1.0, -3.4, 0.0)
      * Eigen::AngleAxisf(-0.15f, Eigen::Vector3f(0.0f, 0.0f, 1.0f));

    float robotScaleFactor = 1.0f;
    if (config["robot"]["scale"] != nullptr) {
      robotScaleFactor = config["robot"]["scale"].get<float>();
    }
    Scene scene((configDirectory 
                 + "/"
                 + config["robot"]["urdf"].get<std::string>()),
                robotScaleFactor,
                configDirectory + "/person/person.urdf",
                background,
                screen);
    for (uint32_t i = 0; i < scene.robotPose.size(); i++) {
        auto& link = scene.robot.getLink(i);
        scene.robotPose[i] = (link.minRadians + link.maxRadians) / 2;
    }
    scene.avatarPose[0] = 0.0;
    scene.avatarPose[1] = 0.5;

    // Adding random noise to the images.
    // std::string imageNoiseFile = configDirectory 
    //         + "/" 
    //         + config["image_noise_file"].get<std::string>();
    //
    // std::ifstream fin(imageNoiseFile, std::ios::in | std::ios::binary);
    // if (!fin.is_open()) {
    //     fprintf(stderr, "Can't open column data file '%s'\n", imageNoiseFile.c_str());
    //     exit(0);
    // }
    //
    // const uint32_t imageNoiseSize = (imageWidth * imageHeight) / 8;     // one bit per pixel
    // const uint32_t noiseFileSize = imageNoiseSize * 8;                  // eight noise images
    // uint8_t *imageNoise = (uint8_t *) malloc(noiseFileSize);
    // fin.read((char *) imageNoise, noiseFileSize);
    // fin.close();

    for (auto& cameraConfig : config["cameras"]) {
        if (0 < cameraConfig.count("port")) {
            Eigen::Vector3f position = toVec<Eigen::Vector3f>(cameraConfig["position"]);
            Eigen::Vector3f rpy = toVec<Eigen::Vector3f>(cameraConfig["rpy"]);
            uint16_t port = cameraConfig["port"];
            cameras.emplace_back(cameras.size(), port, position, rpy, scene);
            auto& camera = cameras.back();
            if (writeBackgrounds) {
              auto backgroundFile = cameraConfig["background"].get<std::string>();
              std::ofstream fout(backgroundFile, std::ios::out | std::ios::binary);
              fout.write((char *) camera.backgroundDepth.data(), sizeof(camera.backgroundDepth));
              fout.close();
            }
        }
    }

    if (writeBackgrounds) {
      return 0;
    }

    uint16_t cameraCount = cameras.size();
    ThreadPool threadPool(cameraCount - 1);

    uint32_t start = timeMs();
    uint32_t frameNumber = 0;
    
    int coreServerFd = openSocket((char *) "rk-core-to-sim", true);
    int coreFd = -1;

    int displayServerFd = openSocket((char *) "rk-display-to-sim", true);
    int displayFd = -1;

    // Robot cloud stuff.
    std::vector<float> robotStartPose(scene.robot.getControlCount(), 0);
    robotStartPose[0] = 0.0f;
    // Next two chosen to give a reasonable starting pose.
    robotStartPose[1] = -0.2;
    robotStartPose[2] = 0.01f;
    robotStartPose[3] = 0.1f;
    robotStartPose[4] = 0.2f;
    robotStartPose[5] = 0.3f;
    Eigen::Vector3f tcpOffset(0.0f, 0.0f, 0.4f);
    RandomFloat random;
    // end of robot cloud stuff

    while (!exitFlag) {
        // fprintf(stderr, "[frame gap %d]\n", timeMs() - start);
        start = timeMs();
        uint32_t renderTime = 0;
        uint32_t robotTime = 0;
        uint32_t sendTime = 0;

        if (displayFd == -1) {
            displayFd = acceptClient(displayServerFd);
        } else {
            displayCommands.readCommands(displayFd);
        }

        if (coreFd == -1) {
            coreFd = acceptClient(coreServerFd);
            sendData = true;
        } else {
            coreCommands.readCommands(coreServerFd);
        }

        if (! sendData) {
            continue;
        }

        // Update the avatar and robot positions from values received in commands.
        for (uint32_t i = 0; i < 2; i++) {
            scene.avatarPose[i] += std::max(-3.0f, std::min(3.0f, avatarDelta[i]));
            scene.avatarPose[i] = std::max(0.0f, std::min(scene.avatarPose[i], 3.2f));
            avatarDelta[i] = 0;
        }

        for (uint32_t i = 0; i < 6; i++) {
            auto& link = scene.robot.getLink(i);
            if (i == 0 && 0 <= robotRotation) {
                float range = link.maxRadians - link.minRadians;
                scene.robotPose[i] = link.minRadians + ((robotRotation % 8) * range) / 8.0f;
            } else {
                float maxVel = link.maxVelocityRadians;
                scene.robotPose[i] += std::max(-maxVel, std::min(maxVel, robotDelta[i]));
                scene.robotPose[i] = std::max(link.minRadians, std::min(scene.robotPose[i], link.maxRadians));
                robotDelta[i] = 0;
            }
        }

        // robotPose[5] = fmod(robotPose[5] + 0.1, 2 * M_PI);

        if (coreFd != -1) {
            // fprintf(stderr, "[pose A %d %.4f %.4f %.4f %.4f %.4f]\n",
            //         frameNumber, scene.robotPose[0],scene.robotPose[1],scene.robotPose[2],scene.robotPose[3],scene.robotPose[4]);
            sendCommand(coreFd,
                    "robot", 
                    { (int32_t) frameNumber,
                      (int32_t) (scene.robotPose[0] * 10000.0f),
                      (int32_t) (scene.robotPose[1] * 10000.0f),
                      (int32_t) (scene.robotPose[2] * 10000.0f),
                      (int32_t) (scene.robotPose[3] * 10000.0f),
                      (int32_t) (scene.robotPose[4] * 10000.0f),
                      (int32_t) (scene.robotPose[5] * 10000.0f) });
        }

        uint32_t startTime = timeMs();
        if (testMode) {
            for (auto& camera : cameras) {
                camera.renderScene(frameNumber, scene);
            }
        } else {
            std::vector<std::future<int>> futures;
            uint32_t start = timeMs();
            for (uint16_t i = 0; i < cameraCount - 1; i++) {
                futures.emplace_back(threadPool.enqueue(renderScene,
                                                        &cameras[i],
                                                        frameNumber,
                                                        std::ref(scene)));
            }
            cameras[cameraCount - 1].renderScene(frameNumber, scene);
            for (auto& future : futures) {
                future.get();
            }
        }

        frameNumber += 1;
        if (testMode) {
            exitFlag = true;
        }
        uint32_t elapsed = timeMs() - start;
        if (elapsed < cadenceMs) {
            timespec sleepTime;
            // fprintf(stderr, "[sleep %d sec %d msec]\n",
            //         (cadenceMs - elapsed) / 1000,
            //         (cadenceMs - elapsed) % 1000);
            sleepTime.tv_sec = (cadenceMs - elapsed) / 1000;
            sleepTime.tv_nsec = ((cadenceMs - elapsed) % 1000) * 1000000;
            nanosleep(&sleepTime, nullptr);
        }
    }

    return 0;
}
