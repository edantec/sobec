///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2019-2021, LAAS-CNRS, University of Edinburgh
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#ifndef SOBEC_RANDOM_GENERATOR_HPP_
#define SOBEC_RANDOM_GENERATOR_HPP_

#include <Eigen/Dense>
#include <boost/bind/bind.hpp>

#if __cplusplus >= 201103L
#include <random>
std::mt19937 rng;
#else
#include <boost/nondet_random.hpp>
#include <boost/random.hpp>
boost::random::mt19937 rng;
#endif

namespace sobec {
namespace unittest {

template <typename IntType>
IntType random_int_in_range(IntType first = 0, IntType last = 10) {
#if __cplusplus >= 201103L
  return std::uniform_int_distribution<IntType>(first, last)(rng);
#else
  return boost::random::uniform_int_distribution<IntType>(first, last)(rng);
#endif
}

template <typename RealType>
RealType random_real_in_range(RealType first = 0, RealType last = 1) {
#if __cplusplus >= 201103L
  return std::uniform_real_distribution<RealType>(first, last)(rng);
#else
  return boost::random::uniform_real_distribution<RealType>(first, last)(rng);
#endif
}

bool random_boolean() {
  static auto generator = boost::bind(std::uniform_int_distribution<>(0, 1), std::default_random_engine());
  return generator();
}

}  // namespace unittest
}  // namespace sobec

#endif  // SOBEC_RANDOM_GENERATOR_HPP_
