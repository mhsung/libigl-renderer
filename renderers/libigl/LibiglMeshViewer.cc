// Copyright (C) 2017 Minhyuk Sung <mhsung@cs.stanford.edu>
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.
//

#include "LibiglMeshViewer.h"

#include <igl/avg_edge_length.h>
#include <igl/png/writePNG.h>
#include <utils/google_tools.h>


LibiglMeshViewer::LibiglMeshViewer(const int _width, const int _height)
  : LibiglMeshRendererT(_width, _height) {
  // FIXME(mhsung): Set window size.
	viewer_.core.show_lines = false;
	viewer_.core.show_vertid = true;
}

LibiglMeshViewer::~LibiglMeshViewer() {
}

const Matrix4f& LibiglMeshViewer::get_projection() const {
  return viewer_.core.proj;
}

const Matrix4f& LibiglMeshViewer::get_modelview() const {
  return viewer_.core.model;
}

void LibiglMeshViewer::set_projection(const Matrix4f& _projection) {
  viewer_.core.proj = _projection;
}

void LibiglMeshViewer::set_modelview(const Matrix4f& _modelview) {
  viewer_.core.model = _modelview;
}

void LibiglMeshViewer::set_mesh(
    const Eigen::MatrixXd& _V, const Eigen::MatrixXi& _F) {
  viewer_.data.clear();
  viewer_.data.set_mesh(_V, _F);
}

void LibiglMeshViewer::set_face_colors(const Eigen::MatrixXf& _FC) {
  viewer_.data.set_colors(_FC.cast<double>());
}

void LibiglMeshViewer::set_vertex_normals(const Eigen::MatrixXd& _VN) {
	const auto& V = viewer_.data.V;
	const auto& F = viewer_.data.F;
	const double avg = 0.5 * igl::avg_edge_length(V, F);

	const RowVector3d green(0.2, 0.8, 0.2);
	viewer_.data.add_edges(V, V + _VN * avg, green);
}

void LibiglMeshViewer::set_point_cloud(const Eigen::MatrixXd& _P) {
  viewer_.data.points.resize(_P.rows(), 6);
  viewer_.data.points.leftCols(3) = _P;

  RowVector3d color;
  color << igl::MAYA_GREY(0), igl::MAYA_GREY(1), igl::MAYA_GREY(2);
  viewer_.data.points.rightCols(3).rowwise() = color;
}

void LibiglMeshViewer::set_point_colors(const Eigen::MatrixXf& _PC) {
  CHECK_EQ(_PC.rows(), viewer_.data.points.rows());
  viewer_.data.points.rightCols(3) = _PC.cast<double>();
}

void LibiglMeshViewer::set_point_normals(const Eigen::MatrixXd& _PN) {
	const auto& P = viewer_.data.points.leftCols(3);
	const auto& V = viewer_.data.V;
	const auto& F = viewer_.data.F;
	const double avg = 0.1 * igl::avg_edge_length(V, F);

	const RowVector3d green(0.2, 0.8, 0.2);
	viewer_.data.add_edges(P, P + _PN * avg, green);
}

void LibiglMeshViewer::run_loop() {
  viewer_.launch();
}

bool LibiglMeshViewer::snapshot(const std::string& _filename) {
  // Draw mesh.
  viewer_.draw();

  // Allocate temporary buffers.
  Matrix<unsigned char, Dynamic, Dynamic> R(width_, height_);
  Matrix<unsigned char, Dynamic, Dynamic> G(width_, height_);
  Matrix<unsigned char, Dynamic, Dynamic> B(width_, height_);
  Matrix<unsigned char, Dynamic, Dynamic> A(width_, height_);

  // Draw the scene in the buffers
  viewer_.core.draw_buffer(viewer_.data, viewer_.opengl, false, R, G, B, A);

  // Save it to a PNG.
  return igl::png::writePNG(R, G, B, A, _filename + std::string(".png"));
}
