// Copyright (C) 2017 Minhyuk Sung <mhsung@cs.stanford.edu>
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.
//

#ifndef OFFSCREEN_RENDERER_H
#define OFFSCREEN_RENDERER_H

#include <GLFW/glfw3.h>

#ifdef __APPLE__
#define GL_SILENCE_DEPRECATION
#endif

#if GLFW_USE_OSMESA
  #define GLFW_EXPOSE_NATIVE_OSMESA
  #include <GLFW/glfw3native.h>
#endif

#include "GLMeshRenderer.h"


class OffscreenRenderer : public GLMeshRenderer {
  public:
    OffscreenRenderer(const int _width, const int _height);
    virtual ~OffscreenRenderer();

    // Virtual functions.
    virtual bool snapshot(const std::string& _filename);

  private:
    void initialize_context();

  private:
    GLFWwindow* window_;

    // Super sampling anti-aliasing (SSAA) resolution.
    int ssaa_width_;
    int ssaa_height_;
};

#endif	// OFFSCREEN_RENDERER_H
