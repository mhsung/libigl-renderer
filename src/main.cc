// Copyright (C) 2017 Minhyuk Sung <mhsung@cs.stanford.edu>
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.
//

#define GLOG_NO_ABBREVIATED_SEVERITIES

#include <utils/google_tools.h>
#include "LibiglMesh.h"
#ifdef OFFSCREEN 
#include "OffscreenRenderer.h"
#else
#include "LibiglMeshViewer.h"
#endif


DEFINE_int32(image_width, 1920, "image width.");
DEFINE_int32(image_height, 1080, "image height.");


int main(int argc, char *argv[]) {
  google::ParseCommandLineFlags(&argc, &argv, true);

  LibiglMeshRendererT* renderer = nullptr;

#ifdef OFFSCREEN 
  OffscreenRenderer osmesa_renderer(FLAGS_image_width, FLAGS_image_height);
  renderer = &osmesa_renderer;
#else
  LibiglMeshViewer libigl_renderer(FLAGS_image_width, FLAGS_image_height);
  renderer = &libigl_renderer;
#endif

  LibiglMesh mesh(renderer);
  mesh.parse_arguments_and_run();
}
