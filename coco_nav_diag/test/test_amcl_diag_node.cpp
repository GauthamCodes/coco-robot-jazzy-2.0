// C2-NAV.37 pre-flight regression test.
//
// C2-NAV.36's own executable (coco_nav_diag/src/main.cpp) previously
// included nav2_amcl/amcl_node.hpp (the installed, upstream header) and
// instantiated nav2_amcl::AmclNode instead of this package's own
// coco_nav_diag::AmclNode fork. That compiled and linked cleanly -- the
// unmodified upstream class is exported by the nav2_amcl dependency -- but
// nav2_amcl::AmclNode declares none of diag_enabled/diag_output_path/
// diag_max_events, so amcl_diag would have run plain, uninstrumented AMCL
// and silently produced no diagnostic output regardless of the -p flags
// passed to it. test_diag_recorder.cpp deliberately never constructs
// AmclNode (it exercises DiagRecorder in isolation, see its own header
// comment), so nothing else in this suite would catch a regression of that
// kind. This test constructs the real coco_nav_diag::AmclNode -- exactly as
// main.cpp does -- and checks for the parameters only the instrumented fork
// declares. It requires no map, no TF, no simulator, and no lifecycle
// activation: coco_nav_diag::AmclNode declares these parameters
// unconditionally in its constructor (amcl_node.cpp, see add_parameter calls
// for diag_enabled/diag_output_path/diag_max_events).

#include <gtest/gtest.h>

#include <memory>
#include <string>

#include "coco_nav_diag/amcl_node.hpp"
#include "rclcpp/rclcpp.hpp"

class AmclDiagNodeTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestSuite()
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

TEST_F(AmclDiagNodeTest, ConstructsTheInstrumentedForkWithDiagnosticParameters)
{
  auto node = std::make_shared<coco_nav_diag::AmclNode>();

  ASSERT_TRUE(node->has_parameter("diag_enabled"));
  ASSERT_TRUE(node->has_parameter("diag_output_path"));
  ASSERT_TRUE(node->has_parameter("diag_max_events"));

  EXPECT_FALSE(node->get_parameter("diag_enabled").as_bool());
  EXPECT_EQ(node->get_parameter("diag_output_path").as_string(), std::string(""));
  EXPECT_EQ(node->get_parameter("diag_max_events").as_int(), 200000);
}
