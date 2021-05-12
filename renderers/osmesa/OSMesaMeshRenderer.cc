// Copyright (C) 2017 Minhyuk Sung <mhsung@cs.stanford.edu>
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.
//

#include "OSMesaMeshRenderer.h"

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


OSMesaMeshRenderer::OSMesaMeshRenderer(const int _width, const int _height)
  : GLMeshRenderer(_width, _height),
    ssaa_width_(kSSAAFactor * _width),
    ssaa_height_(kSSAAFactor * _height) {
  initialize_osmesa();
	initialize_opengl();
}

OSMesaMeshRenderer::~OSMesaMeshRenderer() {
  free(frame_buffer_);
  OSMesaDestroyContext(context_);
}

bool OSMesaMeshRenderer::snapshot(const std::string& _filename) {
  // Draw.
  render();

  // Down-sampling.
  const int stride_in_bytes = kNumImageChannel * width_;
  const int ssaa_stride_in_bytes = kNumImageChannel * ssaa_width_;
  unsigned char *pixels = (unsigned char*) malloc(
      kNumImageChannel * width_ * height_);
  stbir_resize_uint8(static_cast<unsigned char *>(frame_buffer_),
      ssaa_width_, ssaa_height_, ssaa_stride_in_bytes,
      pixels, width_, height_, stride_in_bytes, kNumImageChannel);

  // Save PNG file.
  const std::string filepath = _filename + std::string(".png");
  stbi_flip_vertically_on_write(true);
  return stbi_write_png(filepath.c_str(), width_, height_, kNumImageChannel,
      pixels, stride_in_bytes);
}

void OSMesaMeshRenderer::initialize_osmesa() {
  context_ = OSMesaCreateContextExt(OSMESA_RGBA, 16, 0, 0, NULL);
  if (!context_) {
    LOG(ERROR) << "OSMesaCreateContext failed!";
  }

  // Set a high resultion for super sampling anti-aliasing (SSAA).
  const int n_ssaa_bytes = kNumImageChannel * ssaa_width_ * ssaa_height_;

  frame_buffer_ = malloc(n_ssaa_bytes * sizeof(GLubyte));
  if (!frame_buffer_) {
    LOG(ERROR) << "Allocating image buffer failed!";
  }

  if (!OSMesaMakeCurrent(context_, frame_buffer_, GL_UNSIGNED_BYTE,
        ssaa_width_, ssaa_height_)) {
    LOG(ERROR) << "OSMesaMakeCurrent failed!";
  }

  int z, s, a;
  glGetIntegerv(GL_DEPTH_BITS, &z);
  glGetIntegerv(GL_STENCIL_BITS, &s);
  glGetIntegerv(GL_ACCUM_RED_BITS, &a);
  LOG(INFO) << "Depth = " << z << "  Stencil = " << s << "  Accum = " << a;
}

