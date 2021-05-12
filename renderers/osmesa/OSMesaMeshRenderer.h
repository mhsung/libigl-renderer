// Copyright (C) 2017 Minhyuk Sung <mhsung@cs.stanford.edu>
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.
//

#ifndef OSMESA_MESH_RENDERER_T_H
#define OSMESA_MESH_RENDERER_T_H

#include <GL/osmesa.h>
#include "GLMeshRenderer.h"


class OSMesaMeshRenderer : public GLMeshRenderer {
  public:
    OSMesaMeshRenderer(const int _width, const int _height);
    virtual ~OSMesaMeshRenderer();

    // Virtual functions.
    virtual bool snapshot(const std::string& _filename);

  private:
    void initialize_osmesa();

  private:
    OSMesaContext context_;
    void *frame_buffer_;

    // Super sampling anti-aliasing (SSAA) resolution.
    int ssaa_width_;
    int ssaa_height_;
};

#endif	// OSMESA_MESH_RENDERER_T_H
