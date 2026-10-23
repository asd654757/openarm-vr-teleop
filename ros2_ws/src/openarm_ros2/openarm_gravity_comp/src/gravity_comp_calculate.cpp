// Copyright 2026 Enactic, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <stdexcept>
#include <string>
#include <vector>

#include "openarm_gravity_comp/arm_dynamics.hpp"

namespace
{

constexpr int kExpectedArgumentCount = 10;

void print_usage(const char * executable)
{
  std::cerr << "Usage: " << executable
            << " ROBOT_DESCRIPTION.urdf SIDE Q1 Q2 Q3 Q4 Q5 Q6 Q7\n"
            << "SIDE must be left or right; joint positions are radians.\n";
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc != kExpectedArgumentCount) {
    print_usage(argv[0]);
    return 2;
  }

  const std::string side = argv[2];
  if (side != "left" && side != "right") {
    print_usage(argv[0]);
    return 2;
  }

  std::ifstream input(argv[1]);
  if (!input) {
    std::cerr << "Failed to open URDF: " << argv[1] << '\n';
    return 2;
  }
  const std::string robot_description((std::istreambuf_iterator<char>(input)),
    std::istreambuf_iterator<char>());

  std::vector<double> positions;
  positions.reserve(7);
  try {
    for (int i = 3; i < argc; ++i) {
      std::size_t parsed = 0;
      const std::string value = argv[i];
      positions.push_back(std::stod(value, &parsed));
      if (parsed != value.size()) {
        throw std::invalid_argument("trailing characters");
      }
    }
  } catch (const std::exception & error) {
    std::cerr << "Invalid joint position: " << error.what() << '\n';
    return 2;
  }

  openarm_gravity_comp::ArmDynamics dynamics(
    robot_description, "openarm_body_link0",
    "openarm_" + side + "_hand_tcp");
  std::string error;
  if (!dynamics.initialize(error)) {
    std::cerr << error << '\n';
    return 1;
  }

  std::vector<double> efforts;
  if (!dynamics.compute_gravity(positions, efforts, error)) {
    std::cerr << error << '\n';
    return 1;
  }

  std::cout << std::setprecision(17);
  for (std::size_t i = 0; i < efforts.size(); ++i) {
    if (i != 0U) {
      std::cout << ',';
    }
    std::cout << efforts[i];
  }
  std::cout << '\n';
  return 0;
}
