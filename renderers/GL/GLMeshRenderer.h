// Copyright (C) 2017 Minhyuk Sung <mhsung@cs.stanford.edu>
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.
//

#ifndef GL_MESH_RENDERER_T_H
#define GL_MESH_RENDERER_T_H

#include <vector>
#include <string>
#include <Eigen/Core>

#ifdef __APPLE__
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#else
#include <GL/gl.h>
#include <GL/glu.h>
#endif

#include "LibiglMeshRendererT.h"

using namespace Eigen;


class GLMeshRenderer : public LibiglMeshRendererT {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    GLMeshRenderer(const int _width, const int _height);
    virtual ~GLMeshRenderer();

    // Virtual functions.
    virtual const Matrix4f& get_projection() const;
    virtual const Matrix4f& get_modelview() const;
    virtual void set_projection(const Matrix4f& _projection);
    virtual void set_modelview(const Matrix4f& _modelview);

    virtual void set_mesh(
        const Eigen::MatrixXd& _V, const Eigen::MatrixXi& _F);
    virtual void set_vertex_colors(const Eigen::MatrixXf& _VC);
    virtual void set_face_colors(const Eigen::MatrixXf& _FC);
    virtual void set_vertex_normals(const Eigen::MatrixXd& _VN);
    virtual void clear_mesh();

    virtual void set_point_cloud(const Eigen::MatrixXd& _P);
    virtual void set_point_colors(const Eigen::MatrixXf& _PC);
    virtual void set_point_normals(const Eigen::MatrixXd& _PN);
    virtual void clear_point_cloud();

    virtual void run_loop();

  protected:
    void initialize_opengl();
    static void set_default_material();
    static void set_default_light();
    void render();
    void render_mesh();
    void render_point_cloud();

  protected:
    // Mesh.
    Matrix<double, Dynamic, 3, RowMajor> V_;
    Matrix<int, Dynamic, 3, RowMajor> F_;
    Matrix<double, Dynamic, 3, RowMajor> VN_;
    Matrix<double, Dynamic, 3, RowMajor> FN_;
    Matrix<float, Dynamic, 3, RowMajor> VC_;
    Matrix<float, Dynamic, 3, RowMajor> FC_;

    // Point cloud.
    Matrix<double, Dynamic, 3, RowMajor> P_;
    Matrix<float, Dynamic, 3, RowMajor> PC_;

    Matrix4f projection_;
    Matrix4f modelview_;

    std::vector<std::string> draw_mode_names_;
    int draw_mode_idx_;
};

#endif	// GL_MESH_RENDERER_T_H
