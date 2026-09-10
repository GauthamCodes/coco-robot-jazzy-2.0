// C2-NAV.36 focused unit tests for DiagRecorder.
//
// These deliberately exercise DiagRecorder in isolation -- no rclcpp, no
// AmclNode, no ROS runtime, no simulator -- covering exactly the four
// properties the C2-NAV.36 task called out: disabled mode, the enabled
// logging path, TF-failure-event handling, and deterministic
// serialization/schema. A fifth group covers the bounded-queue/drop
// accounting design ("avoid enormous unbounded logs").

#include <gtest/gtest.h>

#include <unistd.h>

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "coco_nav_diag/diag_recorder.hpp"

namespace
{

std::string readWholeFile(const std::string & path)
{
  std::ifstream in(path, std::ios::binary);
  std::ostringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

std::vector<std::string> splitLines(const std::string & text)
{
  std::vector<std::string> lines;
  std::istringstream iss(text);
  std::string line;
  while (std::getline(iss, line)) {
    if (!line.empty()) {lines.push_back(line);}
  }
  return lines;
}

class TempFile
{
public:
  TempFile()
  {
    char tmpl[] = "/tmp/coco_nav_diag_test_XXXXXX";
    int fd = mkstemp(tmpl);
    if (fd >= 0) {close(fd);}
    path_ = tmpl;
  }
  ~TempFile() {std::remove(path_.c_str());}
  const std::string & path() const {return path_;}

private:
  std::string path_;
};

coco_nav_diag::OdomTfLookupEvent makeSuccessEvent()
{
  coco_nav_diag::OdomTfLookupEvent ev;
  ev.update_index = 42;
  ev.scan_stamp_sec = 100;
  ev.scan_stamp_nanosec = 250000000;
  ev.base_frame_id = "base_footprint";
  ev.odom_frame_id = "odom";
  ev.lookup_success = true;
  ev.odom_x = 1.0;
  ev.odom_y = -2.5;
  ev.odom_yaw = 0.0;
  ev.odom_qx = 0.0;
  ev.odom_qy = 0.0;
  ev.odom_qz = 1.0;
  ev.odom_qw = 0.0;
  ev.odom_pose_stamp_sec = 100;
  ev.odom_pose_stamp_nanosec = 250000000;
  ev.consecutive_failures = 0;
  ev.node_now_sec = 200;
  ev.node_now_nanosec = 500000000;
  return ev;
}

}  // namespace

// ---------------------------------------------------------------------
// Disabled mode
// ---------------------------------------------------------------------

TEST(DiagRecorderDisabled, NeverConfiguredIsDisabledAndSafe)
{
  coco_nav_diag::DiagRecorder rec;
  EXPECT_FALSE(rec.enabled());
  rec.recordOdomTfLookup(makeSuccessEvent());
  EXPECT_EQ(rec.recordedCount(), 0u);
  EXPECT_EQ(rec.droppedCount(), 0u);
}

TEST(DiagRecorderDisabled, ExplicitlyDisabledEmitsNothing)
{
  TempFile tf;
  coco_nav_diag::DiagRecorder rec;
  rec.configure(false, tf.path(), 1000);
  EXPECT_FALSE(rec.enabled());
  rec.recordOdomTfLookup(makeSuccessEvent());
  rec.stop();
  // configure(false, ...) must not even open the file.
  std::ifstream check(tf.path());
  EXPECT_EQ(check.peek(), std::ifstream::traits_type::eof());
}

TEST(DiagRecorderDisabled, EnabledWithEmptyPathStaysDisabled)
{
  std::vector<std::string> warnings;
  coco_nav_diag::DiagRecorder rec;
  rec.configure(
    true, "", 1000,
    [&warnings](const std::string & msg) {warnings.push_back(msg);});
  EXPECT_FALSE(rec.enabled());
  ASSERT_FALSE(warnings.empty());
  EXPECT_NE(warnings[0].find("diag_output_path"), std::string::npos);
  rec.recordOdomTfLookup(makeSuccessEvent());
  EXPECT_EQ(rec.recordedCount(), 0u);
}

// ---------------------------------------------------------------------
// Enabled logging path
// ---------------------------------------------------------------------

TEST(DiagRecorderEnabled, WritesOneJsonlLinePerEvent)
{
  TempFile tf;
  coco_nav_diag::DiagRecorder rec;
  rec.configure(true, tf.path(), 1000, {}, /*async_flush=*/false);
  ASSERT_TRUE(rec.enabled());

  rec.recordOdomTfLookup(makeSuccessEvent());

  coco_nav_diag::MotionDeltaEvent mev;
  mev.update_index = 42;
  mev.is_anchor_init = false;
  mev.anchor_valid = true;
  mev.motion_model_formula_applicable = true;
  mev.motion_model_type = "nav2_amcl::DifferentialMotionModel";
  rec.recordMotionDelta(mev);

  EXPECT_EQ(rec.recordedCount(), 2u);
  rec.flushSync();

  auto lines = splitLines(readWholeFile(tf.path()));
  ASSERT_EQ(lines.size(), 2u);
  EXPECT_NE(lines[0].find("\"type\":\"odom_tf_lookup\""), std::string::npos);
  EXPECT_NE(lines[0].find("\"update_index\":42"), std::string::npos);
  EXPECT_NE(lines[1].find("\"type\":\"motion_delta\""), std::string::npos);
  EXPECT_NE(
    lines[1].find("\"motion_model_type\":\"nav2_amcl::DifferentialMotionModel\""),
    std::string::npos);
  rec.stop();
}

TEST(DiagRecorderEnabled, AsyncBackgroundThreadDrainsOnStop)
{
  TempFile tf;
  coco_nav_diag::DiagRecorder rec;
  rec.configure(true, tf.path(), 1000);  // async_flush defaults to true
  ASSERT_TRUE(rec.enabled());

  for (int i = 0; i < 50; ++i) {
    auto ev = makeSuccessEvent();
    ev.update_index = static_cast<uint64_t>(i);
    rec.recordOdomTfLookup(ev);
  }

  // stop() must join the writer thread only after it has drained
  // everything queued -- no sleep, no polling, should be deterministic.
  rec.stop();

  auto lines = splitLines(readWholeFile(tf.path()));
  EXPECT_EQ(lines.size(), 50u);
  EXPECT_EQ(rec.recordedCount(), 50u);
  EXPECT_EQ(rec.droppedCount(), 0u);
}

// ---------------------------------------------------------------------
// TF lookup failure handling
// ---------------------------------------------------------------------

TEST(DiagRecorderTfFailure, FailureEventCarriesNoStalePoseAndEscapesMessage)
{
  coco_nav_diag::OdomTfLookupEvent ev;
  ev.update_index = 7;
  ev.scan_stamp_sec = 10;
  ev.scan_stamp_nanosec = 0;
  ev.base_frame_id = "base_footprint";
  ev.odom_frame_id = "odom";
  ev.lookup_success = false;
  ev.error_message =
    "Lookup would require extrapolation into the future.  "
    "Requested time is \"10.000\", frame [odom]\\[base_footprint]";
  ev.consecutive_failures = 3;

  std::string s = coco_nav_diag::DiagRecorder::serialize(ev);
  EXPECT_NE(s.find("\"lookup_success\":false"), std::string::npos);
  EXPECT_NE(s.find("\"consecutive_failures\":3"), std::string::npos);
  // A failure event's pose fields default-construct to zero, distinct from
  // "we looked and it really was the origin" only by lookup_success==false
  // -- offline analysis must gate on lookup_success, never treat a zero
  // odom_x/odom_y as a successful reading.
  EXPECT_NE(s.find("\"odom_x\":0.000000000"), std::string::npos);
  // Quote and backslash in the exception text must be escaped so the line
  // stays valid JSON.
  EXPECT_NE(s.find("\\\"10.000\\\""), std::string::npos);
  EXPECT_NE(s.find("\\\\[base_footprint]"), std::string::npos);
  EXPECT_EQ(s.find("\n"), std::string::npos);  // one JSONL line, no raw newline
}

TEST(DiagRecorderTfFailure, RecordedThroughEnabledPath)
{
  TempFile tf;
  coco_nav_diag::DiagRecorder rec;
  rec.configure(true, tf.path(), 1000, {}, /*async_flush=*/false);

  coco_nav_diag::OdomTfLookupEvent ev;
  ev.update_index = 1;
  ev.lookup_success = false;
  ev.error_message = "extrapolation";
  ev.consecutive_failures = 1;
  rec.recordOdomTfLookup(ev);
  rec.flushSync();

  auto lines = splitLines(readWholeFile(tf.path()));
  ASSERT_EQ(lines.size(), 1u);
  EXPECT_NE(lines[0].find("\"lookup_success\":false"), std::string::npos);
  EXPECT_NE(lines[0].find("\"error_message\":\"extrapolation\""), std::string::npos);
  rec.stop();
}

// ---------------------------------------------------------------------
// Deterministic serialization / schema
// ---------------------------------------------------------------------

TEST(DiagRecorderSchema, SerializationIsDeterministic)
{
  auto ev = makeSuccessEvent();
  std::string a = coco_nav_diag::DiagRecorder::serialize(ev);
  std::string b = coco_nav_diag::DiagRecorder::serialize(ev);
  EXPECT_EQ(a, b);

  auto ev2 = makeSuccessEvent();  // independently constructed, same values
  std::string c = coco_nav_diag::DiagRecorder::serialize(ev2);
  EXPECT_EQ(a, c);
}

TEST(DiagRecorderSchema, ExactFieldOrderAndFormat)
{
  auto ev = makeSuccessEvent();
  std::string expected =
    "{\"schema_version\":1,\"type\":\"odom_tf_lookup\",\"update_index\":42,"
    "\"scan_stamp_sec\":100,\"scan_stamp_nanosec\":250000000,"
    "\"base_frame_id\":\"base_footprint\",\"odom_frame_id\":\"odom\","
    "\"lookup_success\":true,\"odom_x\":1.000000000,\"odom_y\":-2.500000000,"
    "\"odom_yaw\":0.000000000,\"odom_qx\":0.000000000,\"odom_qy\":0.000000000,"
    "\"odom_qz\":1.000000000,\"odom_qw\":0.000000000,"
    "\"odom_pose_stamp_sec\":100,\"odom_pose_stamp_nanosec\":250000000,"
    "\"error_message\":\"\",\"consecutive_failures\":0,"
    "\"node_now_sec\":200,\"node_now_nanosec\":500000000}";
  EXPECT_EQ(coco_nav_diag::DiagRecorder::serialize(ev), expected);
}

TEST(DiagRecorderSchema, SchemaVersionConstantMatchesEmittedValue)
{
  auto ev = makeSuccessEvent();
  std::string s = coco_nav_diag::DiagRecorder::serialize(ev);
  std::string prefix =
    "{\"schema_version\":" + std::to_string(coco_nav_diag::kDiagSchemaVersion) + ",";
  EXPECT_EQ(s.compare(0, prefix.size(), prefix), 0);
}

TEST(DiagRecorderSchema, SessionStartLineIsWellFormed)
{
  std::string s = coco_nav_diag::DiagRecorder::serializeSessionStart("amcl", 200000, 12345);
  EXPECT_NE(s.find("\"type\":\"diag_session_start\""), std::string::npos);
  EXPECT_NE(s.find("\"node_name\":\"amcl\""), std::string::npos);
  EXPECT_NE(s.find("\"max_events\":200000"), std::string::npos);
  EXPECT_NE(s.find("\"pid\":12345"), std::string::npos);
}

// ---------------------------------------------------------------------
// Bounded queue / drop accounting
// ---------------------------------------------------------------------

TEST(DiagRecorderBounded, DropsBeyondCapWithoutGrowing)
{
  TempFile tf;
  coco_nav_diag::DiagRecorder rec;
  rec.configure(true, tf.path(), 2, {}, /*async_flush=*/false);
  ASSERT_TRUE(rec.enabled());

  for (int i = 0; i < 5; ++i) {
    auto ev = makeSuccessEvent();
    ev.update_index = static_cast<uint64_t>(i);
    rec.recordOdomTfLookup(ev);
  }

  EXPECT_EQ(rec.recordedCount(), 2u);
  EXPECT_EQ(rec.droppedCount(), 3u);
  EXPECT_EQ(rec.queuedCount(), 2u);

  rec.stop();
  auto lines = splitLines(readWholeFile(tf.path()));
  EXPECT_EQ(lines.size(), 2u);
}

// ---------------------------------------------------------------------
// Misuse: configure() called twice
// ---------------------------------------------------------------------

TEST(DiagRecorderMisuse, SecondConfigureIsNoOpAndWarns)
{
  TempFile tf;
  std::vector<std::string> warnings;
  coco_nav_diag::DiagRecorder rec;
  rec.configure(
    false, tf.path(), 1000,
    [&warnings](const std::string & msg) {warnings.push_back(msg);});
  size_t before = warnings.size();
  rec.configure(
    true, tf.path(), 1000,
    [&warnings](const std::string & msg) {warnings.push_back(msg);});
  EXPECT_FALSE(rec.enabled());  // still reflects the FIRST configure() call
  EXPECT_GT(warnings.size(), before);
}

// No main(): ament_add_gtest links GTEST_MAIN_LIBRARIES by default.
