// Copyright (c) 2022 Richard Kelsey. All rights reserved.

#include "camera.hpp"
#include "command.hpp"
#include "fusion.hpp"
#include "robot.hpp"
#include "socket.hpp"
#include "cruft.hpp"
#include "registration.hpp"
#include "render.hpp"
#include "display.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include "ThreadPool.h"

#include <array>
#include <cstdio>
#include <fstream>
#include <functional>

#include <unistd.h>
#include <sys/types.h>
#include <signal.h>

// Socket connections for sending commands.
static int displayToSimFd;
static int displayToCoreFd;

extern void updateRawImages(bool refresh); // update the raw-image textures

// Our one window.
static GLFWwindow* theWindow;

// Screen layout.
//
// We have the floor image, small editions of all of the raw depth
// images, and one larger pane that can show either a perspective
// image or any of the depth images.
//
// They are laid out like this (not to scale):
//
//   +-------------+-----+
//   |             |Floor|
//   | Perspective +-----+
//   |             |blank|
//   +---+---+---+---+---+
//   | 0 | 1 |raw images |
//   +---+---+---+---+---+

// This is all scaled to 1920x1080, as that's what I have.

static const uint32_t windowWidth = 1920;
static const uint32_t windowHeight = 1080;

typedef struct {
  uint32_t x; // location in window, origin is lower left
  uint32_t y;
  uint32_t width;
  uint32_t height;
} Pane;

static Pane mainPane = {0, 120, 1280, 960};
static Pane floorPane = {1280, 120, 640, 960};
static Pane cameraPanes[] = {
  {0,       0, 160, 120},
  {160,     0, 160, 120},
  {160 * 2, 0, 160, 120},
  {160 * 3, 0, 160, 120},
  {160 * 4, 0, 160, 120},
  {160 * 5, 0, 160, 120},
  {160 * 6, 0, 160, 120},
  {160 * 7, 0, 160, 120},
  {160 * 8, 0, 160, 120}
};

//----------------------------------------------------------------
// To make the staggered squares work we write each pixel as a square
// of four screen pixels, with every other column shifted down by one
// pixel.  Colors are 24 bits, one byte each for red, green, and blue.
//
// The height is floorSize + 1 to allow for the extra row of pixels
// needed for staggered squares.
static std::array<uint8_t, floorSize * 2 * (floorSize + 1) * 2 * 3> floorPixels;
// How much of the above array we actually display.  Set once we get
// the configuration data.
static uint32_t floorDisplayWidth;
static uint32_t floorDisplayHeight;

void updateFloorPixels(const std::vector<uint32_t>& floorImageData)
{
    uint32_t dataIndex = 0;
    uint32_t color = 0;
    uint32_t count = 0;
    for (uint32_t x = 0; x < cellSize.x(); x++) {
        for (uint32_t y = 0; y < cellSize.y(); y++) {
            if (count == 0) {
                color = floorImageData[dataIndex] >> 8;
                count = floorImageData[dataIndex] & 0xFF;
                dataIndex += 1;
            } else {
                count -= 1;
            }
            // Convert from floor to texture coordinates.
            uint32_t texX = x * 2; // squares are two pixels wide
            uint32_t texY = y * 4; // squares are two pixels wide and two high
            uint32_t texIndex = (texX + texY * floorDisplayWidth) * 3; // pixels are three bytes
            // For staggered squares.
            if ((x & 1) == 0) {
                texIndex += floorDisplayWidth * 2 * 3; // move up one pixel
            }
            // Write a block of four pixels.
            for (uint32_t i = 0; i < 2; i++) {
                floorPixels[texIndex] = color >> 16;
                floorPixels[texIndex + 1] = color >> 8;
                floorPixels[texIndex + 2] = color;
                floorPixels[texIndex + 3] = color >> 16;
                floorPixels[texIndex + 4] = color >> 8;
                floorPixels[texIndex + 5] = color;
                texIndex += floorDisplayWidth * 2 * 3; // move one row over
            }
        }
    }
}

//----------------------------------------------------------------
// Writing the screen out as a PPM file.  What a mess.  In theory you can
// use glReadPixels() to get the displayed pixels but in practice this
// only works some of the time.  Instead this redraws the image in a
// separate frame buffer and then writes out its pixels.

static void drawWindow(uint32_t width, uint32_t height, bool refresh);
static uint32_t captureCount = 0;
static const uint32_t glRgbaBytes = 4; // GL_RGBA is four bytes per pixel

void screenCapture()
{
    static GLuint framebuffer = 0;
    glGenFramebuffers(1, &framebuffer);
    glBindFramebuffer(GL_FRAMEBUFFER, framebuffer);
    GLuint renderedTexture;
    glGenTextures(1, &renderedTexture);
    glBindTexture(GL_TEXTURE_2D, renderedTexture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, windowWidth, windowHeight, 0,GL_RGBA, GL_UNSIGNED_BYTE, 0);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, renderedTexture, 0);
    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
        fprintf(stderr, "framebuffer is not complete\n");
        return;
    }
    drawWindow(windowWidth, windowHeight, true);
    std::array<uint8_t, windowWidth * windowHeight * glRgbaBytes> pixels;
    glReadPixels(0, 0, windowWidth, windowHeight, GL_RGBA, GL_UNSIGNED_BYTE, pixels.data());
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    std::string filename = "capture" + std::to_string(captureCount) + ".ppm";
    captureCount += 1;
    fprintf(stderr, "[capture '%s']\n", filename.c_str());
    FILE *out = fopen(filename.c_str(), "w");
    fprintf(out, "P3\n%d %d\n%d\n", windowWidth, windowHeight, 255);
    for (size_t i = 0; i < windowHeight; i++) {
        for (size_t j = 0; j < windowWidth; j++) {
            // GL origin is lower left, PPM origin is upper left.
            size_t index = ((windowHeight - i - 1) * windowWidth + j) * glRgbaBytes;
            fprintf(out, "%3d %3d %3d ", pixels[index], pixels[index + 1], pixels[index + 2]);
        }
        fprintf(out, "\n");
    }
    fclose(out);
}

//----------------------------------------------------------------
// GLFW callbacks

void errorCallback(int /* error */, const char* description)
{
    fprintf(stderr, "GLFW error: %s\n", description);
}

// Set to false to exit the program.
static bool running = true;

// These are all interactive controls.  Not all of them currently do
// anything.

#define perspectiveCameraIndex 255
#define backgroundCameraIndex 254
static uint8_t mainCameraIndex = perspectiveCameraIndex;

static Show floorShow = Show::combined;
static uint16_t layerToShow = 0;

static void updateShow()
{
    sendCommand(displayToCoreFd,
                "show",
                {static_cast<int32_t>(floorShow),
                 layerToShow,
                 mainCameraIndex});
}

bool debugPrint = false;
bool printClouds = false;
bool printBlobs = false;

static uint8_t robotSpriteLevel = 0;
static bool freeze = false;
static bool chooseCamera = false;
static bool showBackground = false;

static void (*arrowKeyAction)(int key, int modifiers) = nullptr;

// Choosing which column to examine.
static void columnMode(int key, int modifiers)
{
    switch (key) {
    case GLFW_KEY_UP:
        if (0 < cursorY) {
            cursorY -= 1;
        }
        break;
    case GLFW_KEY_DOWN:
        if (cursorY < floorDisplayWidth) {
            cursorY += 1;
        }
        break;
    case GLFW_KEY_LEFT:
        if (0 < cursorX) {
            cursorX -= 1;
        }
        break;
    case GLFW_KEY_RIGHT:
        if (cursorX < floorDisplayHeight) {
            cursorY += 1;
        }
        break;
    }
}

// Choosing which layer to show.
static void layerMode(int key, int modifiers)
{
    switch (key) {
    case GLFW_KEY_UP:
        if (layerToShow < cellHeight) {
            layerToShow += 1;
            updateShow();
        }
        break;
    case GLFW_KEY_DOWN:
        if (0 < layerToShow) {
            layerToShow -= 1;
            updateShow();
        }
    }
}

static void viewpointMode(int key, int modifiers)
{
    switch (key) {
    case GLFW_KEY_UP:
        renderViewpoint.z() /= 1.1f;
        break;
    case GLFW_KEY_DOWN:
        renderViewpoint.z() *= 1.1f;
        break;
    case GLFW_KEY_LEFT:
        renderViewpoint.x() += 0.2f;
        break;
    case GLFW_KEY_RIGHT:
        renderViewpoint.x() -= 0.2f;
        break;
    }
}

static void cameraLocMode(int key, int modifiers)
{
    int cameraDelta = (modifiers & GLFW_MOD_SHIFT) ? 1 : 8;
    switch (key) {
    case GLFW_KEY_UP:
       break;
    case GLFW_KEY_DOWN:
       break;
    case GLFW_KEY_LEFT:
       break;
    case GLFW_KEY_RIGHT:
       break;
    case GLFW_KEY_PAGE_UP:
       break;
    case GLFW_KEY_PAGE_DOWN:
       break;
    }
}

static void cameraMode(int key, int modifiers, std::string commands)
{
    int cameraDelta = (modifiers & GLFW_MOD_SHIFT) ? 1 : 8;
    char command;
    switch (key) {
    case GLFW_KEY_UP:
        command = commands[0];
        cameraDelta = -cameraDelta;
        break;
    case GLFW_KEY_DOWN:
        command = commands[0];
        break;
    case GLFW_KEY_LEFT:
        command = commands[1];
        break;
    case GLFW_KEY_RIGHT:
        command = commands[1];
        cameraDelta = -cameraDelta;
        break;
    case GLFW_KEY_PAGE_UP:
        command = commands[2];
        cameraDelta = -cameraDelta;
        break;
    case GLFW_KEY_PAGE_DOWN:
        command = commands[2];
        break;
    default:
        return;
    }
    sendCommand(displayToSimFd, "camera", {mainCameraIndex, command, cameraDelta});
}

static void cameraPositionMode(int key, int modifiers)
{
    cameraMode(key, modifiers, "YXZ");
}

static void cameraAngleMode(int key, int modifiers)
{
    cameraMode(key, modifiers, "pyr");
}

static int moviePid;

void keyCallback(GLFWwindow*, // window
                 int key,
                 int,         // scancode
                 int action,
                 int modifiers)
{
    if (action != GLFW_PRESS)  {
        return;
    }
    int cameraDelta = (modifiers & GLFW_MOD_SHIFT) ? 1 : 8;

    switch (key) {

    // If chooseCamera is true the number keys pick which image to show
    // in the large pane:
    //  0: perspective
    //  1-8: raw depth image
    //  9: background image
    // Otherwise they select what the mouse controls.
    case GLFW_KEY_0:    // nothing
    case GLFW_KEY_1:    // nothing
    case GLFW_KEY_2:    // avatar
    case GLFW_KEY_3:    // avatar arms
    case GLFW_KEY_4:    // first three robot joints
    case GLFW_KEY_5:    // last three robot joints
    case GLFW_KEY_6:    // panel
    case GLFW_KEY_7:    // panel
    case GLFW_KEY_8:    // panel
    case GLFW_KEY_9:    // nothing
        if (chooseCamera) {
            if (key == GLFW_KEY_9) {
                showBackground = !showBackground;
            }
            mainCameraIndex = (key == GLFW_KEY_0
                               ? perspectiveCameraIndex
                               : (key == GLFW_KEY_9
                                  ? backgroundCameraIndex
                                  : key - GLFW_KEY_1));
            updateShow();
            chooseCamera = false;
        } else {
            sendCommand(displayToSimFd, "mode", {key - GLFW_KEY_0});
        }
        break;
    case GLFW_KEY_A:
        arrowKeyAction = &cameraAngleMode;
        break;
    case GLFW_KEY_B:
        printBlobs = true;
        break;
    case GLFW_KEY_C:
        screenCapture();
        break;
    case GLFW_KEY_D:
        debugPrint = ! debugPrint;
        if (debugPrint) {
            sendCommand(displayToSimFd, "debug", {});
        }
        break;
    case GLFW_KEY_E:
        arrowKeyAction = &cameraPositionMode;
        break;
    case GLFW_KEY_F:
        if (freeze) {
            freeze = false;
            sendCommand(displayToSimFd, "start", {});
        } else {
            freeze = true;
            sendCommand(displayToSimFd, "stop", {});
        }
        sendCommand(displayToCoreFd, "freeze", {freeze});
        break;
    case GLFW_KEY_G:
        robotSpriteLevel = (robotSpriteLevel + 1) % 3;
        sendCommand(displayToCoreFd, "sprite", {robotSpriteLevel});
        break;
    case GLFW_KEY_H:
      if (0 < moviePid) {
       kill(moviePid, SIGTERM);
      } else {
        moviePid = fork();
        if (moviePid == 0) {
          if (execl("/usr/bin/ffmpeg", "/usr/bin/ffmpeg",
                    "-video_size", "1920x1080",
                    "-framerate", "25",
                    "-f", "x11grab",
                    "-i", ":0.0",
                    "screen-grab.mp4",
                    NULL) < 0) {
            fprintf(stderr, "exec ffmpeg failed\n");
          }
        }
      }
      break;
    case GLFW_KEY_L:
        // Rotate between layer, layerBlobs, and layerCoverage.
        floorShow = (floorShow == Show::layer
                     ? Show::layerBlobs
                     : (floorShow == Show::layerBlobs
                        ? Show::layerCoverage
                        : Show::layer));
        arrowKeyAction = floorShow == Show::layer ? &layerMode : nullptr;
        updateShow();
        break;
    case GLFW_KEY_M:
        // Toggle between columns and maybeOccupied.
        floorShow = (floorShow == Show::combined
                     ? Show::maybeOccupied
                     : Show::combined);
        arrowKeyAction = nullptr;
        updateShow();
        break;
    case GLFW_KEY_O:
        printClouds = true;
        break;
    case GLFW_KEY_P:  // print
        if (examineColumns) {
            printColumn = true;
        } else if (mainCameraIndex < 20) {
            sendCommand(displayToSimFd, "camera", {-1, 0, 0});
        }
        break;
    case GLFW_KEY_Q:
    case GLFW_KEY_ESCAPE: // we might be full screen
        running = false;
        break;
    case GLFW_KEY_R:
        printRays = true;
        break;
    case GLFW_KEY_S:
        startBackgroundScan();
        break;
    case GLFW_KEY_U:
        chooseCamera = true;
        break;
    case GLFW_KEY_X:
        examineColumns = ! examineColumns;
        arrowKeyAction = examineColumns ? &columnMode : nullptr;
        break;
    case GLFW_KEY_Z:
        arrowKeyAction = arrowKeyAction == &viewpointMode
            ? nullptr
            : &viewpointMode;
        fprintf(stderr, "[viewpoint: %.2f %.2f %.2f]\n",
                renderViewpoint.x(),
                renderViewpoint.y(),
                renderViewpoint.z());
        break;
    case GLFW_KEY_UP:
    case GLFW_KEY_DOWN:
    case GLFW_KEY_LEFT:
    case GLFW_KEY_RIGHT:
    case GLFW_KEY_PAGE_UP:
    case GLFW_KEY_PAGE_DOWN:
        if (arrowKeyAction != NULL) {
            arrowKeyAction(key, modifiers);
        }
        break;
    }
}

static bool haveScrollY = false;
static int32_t scrollY;

static void scrollCallback(GLFWwindow* window, double xoffset, double yoffset)
{
    haveScrollY = true;
    scrollY = yoffset;
}

static double lastMouseX;
static double lastMouseY;

static void checkMouse(GLFWwindow* window)
{
    double mouseX;
    double mouseY;
    glfwGetCursorPos(window, &mouseX, &mouseY);
    if (mouseX != lastMouseX
            || mouseY != lastMouseY
            || haveScrollY) {
        sendCommand(displayToSimFd,
                    "mouse",
                    {(int32_t) mouseX, (int32_t) mouseY, haveScrollY ? scrollY : 0});
        lastMouseX = mouseX;
        lastMouseY = mouseY;
        haveScrollY = false;
    }
}

//----------------------------------------------------------------
// Graphics

static GLuint makeTexture(void)
{
    GLuint texture;
    glGenTextures(1, &texture);
    glBindTexture(GL_TEXTURE_2D, texture);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glBindTexture(GL_TEXTURE_2D, 0);
    return texture;
}

static GLuint makeRadianceTexture(void)
{
    GLuint texture = makeTexture();
    glBindTexture(GL_TEXTURE_2D, texture);
    GLint swizzleMask[] = {GL_RED, GL_RED, GL_RED, GL_ZERO};
    glTexParameteriv(GL_TEXTURE_2D, GL_TEXTURE_SWIZZLE_RGBA, swizzleMask);
    glBindTexture(GL_TEXTURE_2D, 0);
    return texture;
}

static GLuint perspectiveTexture;
static GLuint floorTexture;
static GLuint radianceTexture;

static void mapTexture(const Pane& pane)
{
    glEnable(GL_TEXTURE_2D);
    glBegin(GL_QUADS);
    glTexCoord2i(1, 0); glVertex2i(pane.x + pane.width, pane.y);
    glTexCoord2i(1, 1); glVertex2i(pane.x + pane.width, pane.y + pane.height);
    glTexCoord2i(0, 1); glVertex2i(pane.x,              pane.y + pane.height);
    glTexCoord2i(0, 0); glVertex2i(pane.x,              pane.y);
    glEnd();
    glDisable(GL_TEXTURE_2D);
}

static void mapTextureReverseY(const Pane& pane)
{
    glEnable(GL_TEXTURE_2D);
    glBegin(GL_QUADS);
    glTexCoord2i(1, 1); glVertex2i(pane.x + pane.width, pane.y);
    glTexCoord2i(1, 0); glVertex2i(pane.x + pane.width, pane.y + pane.height);
    glTexCoord2i(0, 0); glVertex2i(pane.x,              pane.y + pane.height);
    glTexCoord2i(0, 1); glVertex2i(pane.x,              pane.y);
    glEnd();
    glDisable(GL_TEXTURE_2D);
}

// Swaps around the coordinates to correct the received depth images.
static void mapDepthTexture(const Pane& pane)
{
  glEnable(GL_TEXTURE_2D);
  glBegin(GL_QUADS);
  glTexCoord2i(0, 1); glVertex2i(pane.x,              pane.y + pane.height);
  glTexCoord2i(1, 1); glVertex2i(pane.x,              pane.y);
  glTexCoord2i(1, 0); glVertex2i(pane.x + pane.width, pane.y);
  glTexCoord2i(0, 0); glVertex2i(pane.x + pane.width, pane.y + pane.height);
  glEnd();
  glDisable(GL_TEXTURE_2D);
}

static void mapPerspectiveTexture(void)
{
    glBindTexture(GL_TEXTURE_2D, perspectiveTexture);
    glTexImage2D(GL_TEXTURE_2D,
                 0,
                 GL_RGB,
                 imageWidth,
                 imageHeight,
                 0,
                 GL_RGB,
                 GL_UNSIGNED_BYTE,
                 perspectiveImage.data());
    mapTextureReverseY(mainPane);
    glBindTexture(GL_TEXTURE_2D, 0);
}

static void mapFloorTexture(void)
{
    glBindTexture(GL_TEXTURE_2D, floorTexture);
    glTexImage2D(GL_TEXTURE_2D,
                 0,
                 GL_RGB,
                 floorDisplayWidth * 2,
                 floorDisplayHeight * 2 + 1,
                 0,
                 GL_RGB,
                 GL_UNSIGNED_BYTE,
                 floorPixels.data());
    mapTexture(floorPane);
    glBindTexture(GL_TEXTURE_2D, 0);
}

void updateDepthTexture(Image16& depthImage, uint32_t index)
{
    glBindTexture(GL_TEXTURE_2D, radianceTexture);
    glTexImage2D(GL_TEXTURE_2D,
                 0,
                 GL_RED,
                 imageWidth,
                 imageHeight,
                 0,
                 GL_RED,
                 GL_UNSIGNED_SHORT,
                 depthImage.data());
    mapDepthTexture(cameraPanes[index]);
    if (index == mainCameraIndex) {
        mapDepthTexture(mainPane);
    }
    glBindTexture(GL_TEXTURE_2D, 0);
}

void showBackgroundImage()
{
  glBindTexture(GL_TEXTURE_2D, radianceTexture);
  glTexImage2D(GL_TEXTURE_2D,
               0,
               GL_RED,
               imageWidth,
               imageHeight,
               0,
               GL_RED,
               GL_UNSIGNED_SHORT,
               (showBackground
                ? backgroundImage.data()
                : robotImage.data()));
  mapDepthTexture(mainPane);
  glBindTexture(GL_TEXTURE_2D, 0);
}

static bool printed = false;

static void drawWindow(uint32_t width, uint32_t height, bool refresh)
{
    glViewport(0, 0, width, height);
    glClear(GL_COLOR_BUFFER_BIT);

    updateRawImages(refresh);
    mapFloorTexture();

    if (mainCameraIndex == perspectiveCameraIndex) {
      mapPerspectiveTexture();
    } else if (mainCameraIndex == backgroundCameraIndex) {
      showBackgroundImage();
    }
}

static void windowRefreshCallback(GLFWwindow* window)
{
    int width, height;
    glfwGetFramebufferSize(window, &width, &height);

    if (!printed) {
      fprintf(stderr, "framebuffer size %d %d\n", width, height);
      glfwGetWindowSize(window, &width, &height);
      fprintf(stderr, "window size %d %d\n", width, height);
      float xScale;
      float yScale;
      glfwGetWindowContentScale(window, &xScale, &yScale);
      fprintf(stderr, "scale %.3f %.3f\n", xScale, yScale);
      printed = true;
    }

    drawWindow(width, height, false);
    glfwSwapBuffers(window);
}

// This is the one place where we do need to have a pointer to
// the window, because we're triggering the change.
void update_display(const std::vector<uint32_t>& floorImageData)
{
    updateFloorPixels(floorImageData);
    windowRefreshCallback(theWindow);
}

bool initDisplay(void)
{
    // Round up to mod 8 to make the GL texture code happy.  A
    // smaller alignment might work, but I know this does.
    // Extra +1 for the height to make sure there is room to show
    // the staggered squares.
    floorDisplayWidth = (cellSize.x() + 7) & ~7;
    floorDisplayHeight = (cellSize.y() + + 1 + 7) & ~7;
    floorPane.height = floorPane.width * floorDisplayHeight / floorDisplayWidth;
    floorPane.y = windowHeight - floorPane.height;

    // GL initializations.
    glClearColor(0, 0, 0, 0);
    glShadeModel(GL_FLAT);  // shading mathod: GL_SMOOTH or GL_FLAT
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

    glfwSetErrorCallback(errorCallback);
    if (!glfwInit()) {
        return false;
    }

    // const GLFWvidmode* mode = glfwGetVideoMode(glfwGetPrimaryMonitor());
    // fprintf(stderr, "video mode (%d %d)\n", mode->width, mode->height);
    // return false;

    glfwWindowHint(GLFW_RESIZABLE, GLFW_FALSE);
    GLFWwindow* window = glfwCreateWindow(windowWidth,
                                          windowHeight,
                                          "robot",
                                          glfwGetPrimaryMonitor(), // fullscreen
                                          nullptr);
    if (!window) {
        glfwTerminate();
        return false;
    }

    theWindow = window;
    glfwMakeContextCurrent(window);
    glfwSetKeyCallback(window, keyCallback);
    glfwSetWindowRefreshCallback(window, windowRefreshCallback);
    glfwSetScrollCallback(window, scrollCallback);
    glfwGetCursorPos(window, &lastMouseX, &lastMouseY);

    //match projection to window resolution (could be in reshape callback)
    glMatrixMode(GL_PROJECTION);
    glOrtho(0, windowWidth, 0, windowHeight, -1, 1);
    glMatrixMode(GL_MODELVIEW); // safety, you don't mess with the projection again

    perspectiveTexture = makeTexture();
    floorTexture = makeTexture();
    radianceTexture = makeRadianceTexture();

    displayToSimFd = openSocket((char *) "rk-display-to-sim", false);
    displayToCoreFd = openSocket((char *) "rk-display-to-core", false);

    glewExperimental = GL_TRUE;
    GLenum status = glewInit();
    if (status != GLEW_OK) {
        fprintf(stderr, "Error: %s\n", glewGetErrorString(status));
        return false;
    }

    return true;
}

bool checkDisplay(void)
{
  checkMouse(theWindow);
  glfwPollEvents();
  return running && !glfwWindowShouldClose(theWindow);
}
