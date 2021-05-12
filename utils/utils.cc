// Copyright (C) 2017 Minhyuk Sung <mhsung@cs.stanford.edu>
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.
//

#include <utils/utils.h>

#include <cmath>
#include <utils/simplerandom/simplerandom.h>


namespace Utils {

std::vector<std::string> split_string(
    const std::string& _str, const char delim) {
  std::stringstream sstr(_str);
  std::vector<std::string> tokens;
  std::string token;
  while (std::getline(sstr, token, delim)) {
    tokens.push_back(token);
  }
  return tokens;
}

void hsv2rgb(
    const float h, const float s, const float v,
    float& r, float& g, float& b) {
  float hh, p, q, t, ff;
  long i;

  if (s <= 0.0f) {
    r = v;
    g = v;
    b = v;
    return;
  }

  hh = h * 360.0f;
  if (hh >= 360.0f) hh = 0.0f;
  hh /= 60.0f;
  i = (long) hh;
  ff = hh - i;
  p = v * (1.0f - s);
  q = v * (1.0f - (s * ff));
  t = v * (1.0f - (s * (1.0f - ff)));

  switch (i) {
    case 0:
      r = v;
      g = t;
      b = p;
      break;
    case 1:
      r = q;
      g = v;
      b = p;
      break;
    case 2:
      r = p;
      g = v;
      b = t;
      break;
    case 3:
      r = p;
      g = q;
      b = v;
      break;
    case 4:
      r = t;
      g = p;
      b = v;
      break;
    case 5:
    default:
      r = v;
      g = p;
      b = q;
      break;
  }
}

void random_label_rgb_color(
    const int _label, Eigen::Vector3f* _color) {
  CHECK_NOTNULL(_color);
  CHECK_GE(_label, 0);

  if (_label == 0) {
    _color->setConstant(0.5f);
    return;
  }

  // Reference:
  // http://martin.ankerl.com/2009/12/09/how-to-create-random-colors-programmatically/
  const uint32_t kLabelColoringRandomSeed = 0;
  const float kGoldenRatioConjugate = 0.618033988749895f;

  SimpleRandomCong_t rng_cong;
  simplerandom_cong_seed(&rng_cong, kLabelColoringRandomSeed);

  const float s = static_cast<float>(simplerandom_cong_next(&rng_cong))
                  / std::numeric_limits<uint32_t>::max();
  const float h = std::fmod(s + (_label * kGoldenRatioConjugate), 1.0f);
  float r, g, b;
  hsv2rgb(h, 0.8f, 0.8f, r, g, b);
  (*_color) << r, g, b;
}

}
