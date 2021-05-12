// Copyright (C) 2017 Minhyuk Sung <mhsung@cs.stanford.edu>
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.
//

#ifndef LIBIGL_MESH_VIEWER_H
#define LIBIGL_MESH_VIEWER_H

#include <vector>
#include <string>
#include <Eigen/Core>
#include <igl/viewer/Viewer.h>

#include "LibiglMeshRendererT.h"

using namespace Eigen;


class LibiglMeshViewer : public LibiglMeshRendererT {
  public:
    LibiglMeshViewer(const int _width, const int _height);
    virtual ~LibiglMeshViewer();

    // Virtual functions.
    virtual const Matrix4f& get_projection() const;
    virtual const Matrix4f& get_modelview() const;
    virtual void set_projection(const Matrix4f& _projection);
    virtual void set_modelview(const Matrix4f& _modelview);

    virtual void set_mesh(
        const Eigen::MatrixXd& _V, const Eigen::MatrixXi& _F);
    virtual void set_face_colors(const Eigen::MatrixXf& _FC);
    virtual void set_vertex_normals(const Eigen::MatrixXd& _VN);
    virtual void set_vertex_curvatures(
				const Eigen::MatrixXd& _VPD1, const Eigen::MatrixXd& _VPD2,
				const Eigen::VectorXd& _VPV1, const Eigen::VectorXd& _VPV2);

    virtual void set_point_cloud(const Eigen::MatrixXd& _P);
    virtual void set_point_colors(const Eigen::MatrixXf& _PC);
		virtual void set_point_normals(const Eigen::MatrixXd& _PN);
    virtual void set_point_curvatures(
				const Eigen::MatrixXd& _PPD1, const Eigen::MatrixXd& _PPD2,
				const Eigen::VectorXd& _PPV1, const Eigen::VectorXd& _PPV2);

    virtual void set_primitives(
        const std::vector<PrimitivePtr>& _primitives);
    virtual void set_primitive_colors(const Eigen::MatrixXf& _C);

    virtual void set_symmetries(
        const std::vector<SymmetryPtr>& _symmetries);
    virtual void set_symmetry_colors(const Eigen::MatrixXf& _C);

    virtual void run_loop();
    virtual bool snapshot(const std::string& _filename);

  private:
    igl::viewer::Viewer viewer_;
};

#endif	// LIBIGL_MESH_VIEWER_H
