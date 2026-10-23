// Copyright 2026 Enactic, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>

namespace openarm_gravity_comp
{

class ArmDynamics
{
public:
  ArmDynamics(
    std::string robot_description, std::string base_link,
    std::string tip_link);

  bool initialize(std::string & error);
  bool compute_gravity(
    const std::vector<double> & positions,
    std::vector<double> & efforts,
    std::string & error);

  const std::vector<std::string> & joint_names() const {return joint_names_;}

private:
  std::string robot_description_;
  std::string base_link_;
  std::string tip_link_;
  KDL::Chain chain_;
  std::unique_ptr<KDL::ChainDynParam> solver_;
  std::vector<std::string> joint_names_;
};

}  // namespace openarm_gravity_comp
