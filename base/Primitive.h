// Copyright (C) 2017 Minhyuk Sung <mhsung@cs.stanford.edu>
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.
//

#ifndef PRIMITIVE_H
#define PRIMITIVE_H

#include <memory>
#include <string>
#include <Eigen/Core>
#include <utils/nlohmann_json/json.hpp>

using namespace Eigen;
using json = nlohmann::json;


class Primitive;
typedef std::unique_ptr<Primitive> PrimitivePtr;

class Primitive {
  public:
    Primitive();
    Primitive(const Primitive& _other);
    virtual Primitive* clone() const = 0;

    virtual double area() const = 0;
    virtual double signed_distance(const Vector3d& _p) const = 0;
    virtual Vector3d project(const Vector3d& _p) const = 0;

    virtual void set_default_clipping_params() = 0;
    virtual void compute_clipping_params(const MatrixXd& _P) = 0;
    virtual MatrixXd randomly_sample_points(const int n_points) const = 0;
    virtual void render() const = 0;
    virtual void render_axes() const = 0;

    int get_label() const;
    void set_label(const int _label);

    virtual void to_json(json& _j) const = 0;
    static Primitive* from_json(const json& _j);

  protected:
    int label_;
};

template<class Derived>
class PrimitiveBase : public Primitive {
  public:
    PrimitiveBase(): Primitive() {}
    PrimitiveBase(const PrimitiveBase& _other): Primitive(_other) {}

    virtual PrimitiveBase* clone() const {
      return new Derived(static_cast<const Derived&>(*this));
    }
};

class Plane : public PrimitiveBase<Plane> {
  public:
    Plane();
    Plane(const Vector3d& _n, const Vector3d& _c);
    // @_d: offset. n^T * x + d = 0.
    Plane(const Vector3d& _n, const double _d);
    Plane(const Plane& _other);

    double area() const;
    double signed_distance(const Vector3d& _p) const;
    Vector3d project(const Vector3d& _p) const;

    void set_default_clipping_params();
    void compute_clipping_params(const MatrixXd& _P);
    MatrixXd randomly_sample_points(const int n_points) const;
    void render() const;
    void render_axes() const;

    void to_json(json& _j) const;
    static Plane* from_json(const json& _j);

  private:
    Vector3d n_;	// Normal
    Vector3d c_;  // Center

    // Clipping params.
    Vector3d x_;  // X axis
    Vector3d y_;  // Y axis
    double sx_;	  // X size
    double sy_;	  // Y size
};

// NOTE:
// Currently we do not consider top and bottom circles closing the shape.
class Cylinder : public PrimitiveBase<Cylinder> {
  public:
    Cylinder();
    Cylinder(const Vector3d& _c, const Vector3d& _n, const double _r,
        const double _h);
    Cylinder(const Cylinder& _other);

    double area() const;
    double signed_distance(const Vector3d& _p) const;
    Vector3d project(const Vector3d& _p) const;

    void set_default_clipping_params();
    void compute_clipping_params(const MatrixXd& _P);
    MatrixXd randomly_sample_points(const int n_points) const;
    void render() const;
    void render_axes() const;

    void to_json(json& _j) const;
    static Cylinder* from_json(const json& _j);

  private:
    Vector3d c_;  // Center
    Vector3d n_;  // Axis
    double r_;	  // Radius

    // Clipping params.
    double h_;	  // Height
};

class Sphere : public PrimitiveBase<Sphere> {
  public:
    Sphere();
    Sphere(const Vector3d& _c, const double _r);
    Sphere(const Sphere& _other);

    double area() const;
    double signed_distance(const Vector3d& _p) const;
    Vector3d project(const Vector3d& _p) const;

    void set_default_clipping_params();
    void compute_clipping_params(const MatrixXd& _P);
    MatrixXd randomly_sample_points(const int n_points) const;
    void render() const;
    void render_axes() const;

    void to_json(json& _j) const;
    static Sphere* from_json(const json& _j);

  private:
    Vector3d c_;  // Center
    double r_;	  // Radius
};

// NOTE:
// Currently we do not consider a bottom circle closing the shape.
class Cone : public PrimitiveBase<Cone> {
  public:
    Cone();
    Cone(const Vector3d& _c, const Vector3d& _n, const double _a,
        const double _z_min, const double _z_max);
    Cone(const Cone& _other);

    double radius_top() const;
    double radius_bottom() const;

    double area() const;
    double signed_distance(const Vector3d& _p) const;
    Vector3d project(const Vector3d& _p) const;

    void set_default_clipping_params();
    void compute_clipping_params(const MatrixXd& _P);
    MatrixXd randomly_sample_points(const int n_points) const;
    void render() const;
    void render_axes() const;

    void to_json(json& _j) const;
    static Cone* from_json(const json& _j);

  private:
    Vector3d c_;  // Apex
    Vector3d n_;  // Axis (from the apex to the bottom)
    double a_;	  // Angle

    // Clipping params.
    double z_min_;	  // Distance from apex to top
    double z_max_;	  // Distance from apex to bottom
};

class Torus : public PrimitiveBase<Torus> {
  public:
    Torus();
    Torus(const Vector3d& _c, const Vector3d& _n,
        const double _r1, const double _r2);
    Torus(const Torus& _other);

    double area() const;
    double signed_distance(const Vector3d& _p) const;
    Vector3d project(const Vector3d& _p) const;

    void set_default_clipping_params();
    void compute_clipping_params(const MatrixXd& _P);
    MatrixXd randomly_sample_points(const int n_points) const;
    void render() const;
    void render_axes() const;

    void to_json(json& _j) const;
    static Torus* from_json(const json& _j);

  private:
    Vector3d c_;  // Center
    Vector3d n_;  // Axis
    double r1_;	  // Minor radius
    double r2_;	  // Major radius
};

class Prism : public PrimitiveBase<Prism> {
  public:
    Prism();
    Prism(const Vector3d& _c, const Vector3d& _n,
        const Vector3d& _b, const double _r, const int _N);
    Prism(const Prism& _other);

    double area() const;
    double signed_distance(const Vector3d& _p) const;
    Vector3d project(const Vector3d& _p) const;

    void set_default_clipping_params();
    void compute_clipping_params(const MatrixXd& _P);
    MatrixXd randomly_sample_points(const int n_points) const;
    void render() const;
    void render_axes() const;

    void to_json(json& _j) const;
    static Prism* from_json(const json& _j);

  private:
    Vector3d c_;  // Center
    Vector3d n_;  // Axis
    Vector3d b_;  // Base
    double r_;	  // Radius
    int N_;

    // Clipping params.
    double h_;	  // Height
};

class Box : public PrimitiveBase<Box> {
  public:
    Box();
    Box(const Vector3d& _c, const Vector3d& _x, const Vector3d& _y,
        const double _sx, const double _sy, const double _sz);
    Box(const Box& _other);

    double area() const;
    double signed_distance(const Vector3d& _p) const;
    Vector3d project(const Vector3d& _p) const;

    void set_default_clipping_params();
    void compute_clipping_params(const MatrixXd& _P);
    MatrixXd randomly_sample_points(const int n_points) const;
    void render() const;
    void render_axes() const;

    void to_json(json& _j) const;
    static Box* from_json(const json& _j);

    bool draw_axis_pos_[3];
    bool draw_axis_neg_[3];
    Vector4f axis_pos_color_[3];
    Vector4f axis_neg_color_[3];

  private:
    Vector3d c_;  // Center
    Vector3d x_;  // X axis
    Vector3d y_;  // Y axis
    Vector3d z_;  // Z axis
    double sx_;	  // X size
    double sy_;	  // Y size
    double sz_;	  // Z size
};

#endif	// PRIMITIVE_H

