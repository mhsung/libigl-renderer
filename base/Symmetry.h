// Copyright (C) 2017 Minhyuk Sung <mhsung@cs.stanford.edu>
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.
//

#ifndef SYMMETRY_H
#define SYMMETRY_H

#include <memory>
#include <string>
#include <Eigen/Core>
#include <utils/nlohmann_json/json.hpp>

using namespace Eigen;
using json = nlohmann::json;


class Symmetry;
typedef std::unique_ptr<Symmetry> SymmetryPtr;

class Symmetry {
  public:
    virtual Symmetry* clone() const = 0;

    virtual void render() const = 0;

    virtual void to_json(json& _j) const = 0;
    static Symmetry* from_json(const json& _j);
};

template<class Derived>
class SymmetryBase : public Symmetry {
  virtual SymmetryBase* clone() const {
    return new Derived(static_cast<const Derived&>(*this));
  }
};

class RotationalSymmetry : public SymmetryBase<RotationalSymmetry> {
  public:
    RotationalSymmetry();
    RotationalSymmetry(const Vector3d& _c, const Vector3d& _d);
    RotationalSymmetry(const RotationalSymmetry& _other);

    void render() const;

    void to_json(json& _j) const;
    static RotationalSymmetry* from_json(const json& _j);

  private:
    Vector3d c_;	// Center
    Vector3d d_;	// Direction
};

#endif	// SYMMETRY_H

