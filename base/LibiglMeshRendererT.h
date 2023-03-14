// Copyright (C) 2017 Minhyuk Sung <mhsung@cs.stanford.edu>
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.
//

#ifndef LIBIGL_MESH_RENDERER_T_H
#define LIBIGL_MESH_RENDERER_T_H

#include <vector>
#include <string>
#include <Eigen/Core>
#include <utils/google_tools.h>

using namespace Eigen;

// Declare input variables.
DECLARE_double(fovy_deg);
DECLARE_double(camera_distance);
DECLARE_double(point_radius);
DECLARE_bool(auto_adjust_camera);


class LibiglMeshRendererT {
  public:
    LibiglMeshRendererT(const int _width, const int _height);
    virtual ~LibiglMeshRendererT() {};

    bool read_projection(const std::string& _filename);
    bool read_modelview(const std::string& _filename);
    bool write_projection(const std::string& _filename) const;
    bool write_modelview(const std::string& _filename) const;
    void reset_projection();
    void reset_modelview();
    void set_window_size(const int _width, const int _height);
    void set_scene_pos(const Vector3f& _center, const float _radius);

    // Camera parameters: azimuch, elevation, theta (in degrees).
    Vector3f get_camera_params() const;
    void set_camera_params(const Vector3f& _camera_params,
        const Vector3f& _center, const float _radius);

    // Virtual functions.
    virtual const Matrix4f& get_projection() const = 0;
    virtual const Matrix4f& get_modelview() const = 0;
    virtual void set_projection(const Matrix4f& _projection) = 0;
    virtual void set_modelview(const Matrix4f& _modelview) = 0;

    virtual void set_mesh(
        const Eigen::MatrixXd& _V, const Eigen::MatrixXi& _F) = 0;
    virtual void set_vertex_colors(const Eigen::MatrixXf& _VC) = 0;
    virtual void set_face_colors(const Eigen::MatrixXf& _FC) = 0;
    virtual void set_vertex_normals(const Eigen::MatrixXd& _VN) = 0;
    virtual void clear_mesh() = 0;

    virtual void set_point_cloud(const Eigen::MatrixXd& _P) = 0;
    virtual void set_point_colors(const Eigen::MatrixXf& _PC) = 0;
    virtual void set_point_normals(const Eigen::MatrixXd& _PN) = 0;
    virtual void clear_point_cloud() = 0;

    void set_point_radius(const float _radius) { point_radius_ = _radius; }

    virtual void run_loop() = 0;
    virtual bool snapshot(const std::string& _filename) = 0;

  protected:
    int width_;
    int height_;

    Vector3f center_;
    float radius_;

    // Point radius for rendering.
    float point_radius_;
};

#endif	// LIBIGL_MESH_RENDERER_T_H

