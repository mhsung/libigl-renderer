// Copyright (C) 2017 Minhyuk Sung <mhsung@cs.stanford.edu>
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.
//

#ifndef LIBIGL_MESH_T_H
#define LIBIGL_MESH_T_H

#include <vector>
#include <string>
#include <Eigen/Core>
#include <utils/google_tools.h>
#include <utils/nlohmann_json/json.hpp>
#include "LibiglMeshRendererT.h"
#include "Primitive.h"
#include "Symmetry.h"

using namespace Eigen;
using json = nlohmann::json;

// Declare input variables.
DECLARE_string(mesh);
DECLARE_string(meshes);
DECLARE_string(face_labels);
DECLARE_string(point_cloud);
DECLARE_string(point_labels);
DECLARE_string(point_values);
DECLARE_string(point_displacement);
DECLARE_string(primitives);
DECLARE_string(symmetries);
DECLARE_double(azimuth_deg);
DECLARE_double(elevation_deg);
DECLARE_double(theta_deg);
DECLARE_string(projection_matrix);
DECLARE_string(modelview_matrix);
DECLARE_string(bbox);
DECLARE_string(snapshot);
DECLARE_string(out_mesh);
DECLARE_string(out_point_cloud);
DECLARE_string(out_point_labels);
DECLARE_string(out_primitives);
DECLARE_string(out_symmetries);
DECLARE_string(out_projection_matrix);
DECLARE_string(out_modelview_matrix);
DECLARE_string(snapshot_prefix_per_primitive);
DECLARE_bool(reorient_faces);
DECLARE_bool(draw_mesh_curvatures);
DECLARE_bool(draw_point_cloud_curvatures);


class LibiglMeshT {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    LibiglMeshT();
    LibiglMeshT(LibiglMeshRendererT* _renderer);

    int n_vertices() const { return V_.rows(); }
    int n_faces() const { return F_.rows(); }
    int n_points() const { return P_.rows(); }

    bool read_mesh(const std::string& _filename);
    bool read_meshes(const std::string& _filenames);

    bool read_point_cloud(const std::string& _filename);
    bool write_point_cloud(const std::string& _filename);

    bool read_face_labels(const std::string& _filename);
    bool write_face_labels(const std::string& _filename);
    void set_face_label_colors();

    bool read_point_labels(const std::string& _filename);
    bool write_point_labels(const std::string& _filename);

    bool read_point_values(const std::string& _filename);
    bool read_point_displacements(const std::string& _filename);

    void set_point_label_colors();
    MatrixXf compute_color_map(const VectorXf& _values);

    void update_bounding_box(const MatrixXd& _P);
    void update_bounding_box();
    bool write_bounding_box(const std::string& _filename);

		bool read_primitives_json(const std::string& _filename);
		bool write_primitives_json(const std::string& _filename);

		bool read_symmetries_json(const std::string& _filename);
		bool write_symmetries_json(const std::string& _filename);

		void compute_mesh_principal_curvatures(bool _render = true);
		void compute_point_cloud_principal_curvatures(bool _render = true);

		void update_point_cloud_neighbors();
		MatrixXd randomly_sample_points_on_primitives(const int n_points);

    void parse_arguments_and_run();

  protected:
    // Virtual function.
    virtual void processing() = 0;

    void pre_processing();
    void post_processing();

    // Mesh properties.
    std::string mesh_name_;   // Includes extension.

    // Mesh.
    MatrixXd V_;
    MatrixXi F_;
    MatrixXf VC_;
    MatrixXf FC_;
    VectorXi VL_;
    VectorXi FL_;
    MatrixXd VN_;
    MatrixXd FN_;
    MatrixXd VPD1_;
    MatrixXd VPD2_;
    VectorXd VPV1_;
    VectorXd VPV2_;

    // Point cloud.
    MatrixXd P_;
    MatrixXf PC_;
    VectorXi PL_;
    MatrixXd PN_;
		MatrixXd PPD1_;
    MatrixXd PPD2_;
    VectorXd PPV1_;
    VectorXd PPV2_;
    MatrixXd PD_;

		// Point cloud neighbors.
		MatrixXi P_neighbor_pids_;

    // Primitives.
    std::vector<PrimitivePtr> primitives_;
    std::vector<SymmetryPtr> symmetries_;

    Vector3d bb_min_;
    Vector3d bb_max_;
    Vector3d center_;
    double radius_;

		// Median of point cloud nearest neighbor distances.
		double median_P_nn_dists_;

		// Points to primitives distances.
		MatrixXd P_to_primitives_dists_;

    // Rendering properties.
    LibiglMeshRendererT* renderer_;
};

#endif	// LIBIGL_MESH_T_H
