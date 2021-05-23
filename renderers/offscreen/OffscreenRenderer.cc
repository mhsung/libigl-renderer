// Copyright (C) 2017 Minhyuk Sung <mhsung@cs.stanford.edu>
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.
//

#include "OffscreenRenderer.h"

#include <utils/google_tools.h>

#define STB_IMAGE_RESIZE_IMPLEMENTATION
#define STB_IMAGE_RESIZE_STATIC
#include <utils/stb/stb_image_resize.h>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <utils/stb/stb_image_write.h>


// Super sampling anti-aliasing (SSAA) factor.
const int kSSAAFactor = 4;
// 4 channels (red, green, blue, alpha).
const int kNumImageChannel = 4;

static void error_callback(int error, const char* description) {
  printf("Error: %s\n", description);
}


OffscreenRenderer::OffscreenRenderer(const int _width, const int _height)
  : GLMeshRenderer(_width, _height),
    ssaa_width_(kSSAAFactor * _width),
    ssaa_height_(kSSAAFactor * _height) {
  initialize_context();
  initialize_opengl();
}

OffscreenRenderer::~OffscreenRenderer() {
  glfwSetWindowShouldClose(window_, GLFW_TRUE);
  while (!glfwWindowShouldClose(window_));
  glfwDestroyWindow(window_);
  glfwTerminate();
}

void OffscreenRenderer::initialize_context() {
  glfwSetErrorCallback(error_callback);

  glfwInitHint(GLFW_COCOA_MENUBAR, GLFW_FALSE);

  if (!glfwInit()) {
    printf("Error: GLFW initialization error.\n");
    exit(EXIT_FAILURE);
  }

  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
  glfwWindowHint(GLFW_VISIBLE, GLFW_TRUE);
  glfwWindowHint(GLFW_DECORATED, GLFW_FALSE);
  glfwWindowHint(GLFW_FOCUSED, GLFW_FALSE);

  window_ = glfwCreateWindow(ssaa_width_, ssaa_height_, "OffscreenRenderer", NULL, NULL);
  if (!window_) {
    printf("Error: GLFW window creation error.\n");
    glfwTerminate();
    exit(EXIT_FAILURE);
  }

  glfwMakeContextCurrent(window_);

  // NOTE: OpenGL error checks have been omitted for brevity
}

bool OffscreenRenderer::snapshot(const std::string& _filename) {
  // Draw mesh.
  render();

  void* frame_buffer;

#if GLFW_USE_OSMESA
  int width, height;
  glfwGetOSMesaColorBuffer(window_, &width, &height, NULL, (void**) &frame_buffer);
  CHECK_EQ(width, ssaa_width_);
  CHECK_EQ(height, ssaa_height_);
#else
  frame_buffer = calloc(4, ssaa_width_ * ssaa_height_);
  glReadPixels(0, 0, ssaa_width_, ssaa_height_, GL_RGBA, GL_UNSIGNED_BYTE, frame_buffer);
#endif

  // Down-sampling.
  const int stride_in_bytes = kNumImageChannel * width_;
  const int ssaa_stride_in_bytes = kNumImageChannel * ssaa_width_;
  unsigned char *pixels = (unsigned char*) malloc(
      kNumImageChannel * width_ * height_);
  stbir_resize_uint8(static_cast<unsigned char *>(frame_buffer),
      ssaa_width_, ssaa_height_, ssaa_stride_in_bytes,
      pixels, width_, height_, stride_in_bytes, kNumImageChannel);

  // Save PNG file.
  const std::string filepath = _filename + std::string(".png");
  stbi_flip_vertically_on_write(true);
  return stbi_write_png(filepath.c_str(), width_, height_, kNumImageChannel,
      pixels, stride_in_bytes);

#if GLFW_USE_OSMESA
  // Here is where there's nothing.
#else
  free(frame_buffer);
#endif
}
