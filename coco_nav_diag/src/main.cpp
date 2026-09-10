// Copyright (c) 2018 Intel Corporation
//
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.
//
// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License along with this library; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

#include <memory>

// C2-NAV.37 pre-flight fix: this must instantiate the instrumented
// coco_nav_diag::AmclNode fork (amcl_node.hpp/.cpp in this package), not
// upstream's nav2_amcl::AmclNode -- the latter has no diag_enabled/
// diag_output_path/diag_max_events parameters at all, so passing them
// would silently produce plain, uninstrumented AMCL with no diagnostic
// output. See docs/agents/C2-NAV.37_PRE_FLIGHT.md.
#include "coco_nav_diag/amcl_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<coco_nav_diag::AmclNode>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();

  return 0;
}
