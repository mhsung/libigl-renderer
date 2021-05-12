// Copyright (C) 2017 Minhyuk Sung <mhsung@cs.stanford.edu>
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.
//

#ifndef LIBIGL_MESH_H
#define LIBIGL_MESH_H

#include "LibiglMeshT.h"

using namespace Eigen;


class LibiglMesh : public LibiglMeshT {
  public:
    LibiglMesh() : LibiglMeshT() {};
    LibiglMesh(LibiglMeshRendererT* _renderer) : LibiglMeshT(_renderer) {};


  protected:
    virtual void processing();


  // -- Simple mesh processing -- //
  protected:
    void mesh_processing();

    void remove_duplicates();

    void upsample_mesh(
        const double edge_length_tol, const int max_loop_iters = 10);

    void translate_mesh(MatrixXd& _V, const Vector3d& _t);

    void transform_mesh(MatrixXd& _V, const Vector3d& _r, const Vector3d& _t);

    void inverse_transform_mesh(
        MatrixXd& _V, const Vector3d& _r, const Vector3d& _t);

    // Move center to (0,0,0) and scale to bounding box diagonal 1.
    void normalize_mesh(MatrixXd& _V);


  // -- Simple point set processing -- //
  protected:
    void point_set_processing();

    void sample_points_on_mesh(const int _num_points, const bool _with_normals);

    void normalize_points();

    void compute_point_set_center_and_area(
        const std::string& _out_file, bool _centerize);

    void pca_align_points(const std::string& _out_file = "");
};

#endif	// LIBIGL_MESH_H
