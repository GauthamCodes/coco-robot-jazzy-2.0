// C2-NAV.36 focused unit tests for DiagRecorder.
//
// These deliberately exercise DiagRecorder in isolation -- no rclcpp, no
// AmclNode, no ROS runtime, no simulator -- covering: disabled mode, the
// enabled logging path, TF-failure-event handling, deterministic
// serialization/schema, the bounded-queue/drop accounting, and (C2-NAV.39)
// the session header/footer that makes a capture's integrity checkable from
// the file alone.

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

bool contains(const std::string & haystack, const std::string & needle)
{
  return haystack.find(needle) != std::string::npos;
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

TEST(DiagRecorderDisabled, ExplicitlyDisabledEmitsNothingNotEvenHeaderOrFooter)
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
  EXPECT_TRUE(contains(warnings[0], "diag_output_path"));
  rec.recordOdomTfLookup(makeSuccessEvent());
  EXPECT_EQ(rec.recordedCount(), 0u);
}

// ---------------------------------------------------------------------
// Enabled logging path
// ---------------------------------------------------------------------

TEST(DiagRecorderEnabled, WritesHeaderEventsAndFooter)
{
  TempFile tf;
  coco_nav_diag::DiagRecorder rec;
  rec.configure(true, tf.path(), 1000, {}, /*async_flush=*/false, "/amcl");
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
  ASSERT_EQ(lines.size(), 3u);
  EXPECT_TRUE(contains(lines[0], "\"type\":\"diag_session_start\""));
  EXPECT_TRUE(contains(lines[0], "\"node_name\":\"/amcl\""));
  EXPECT_TRUE(contains(lines[1], "\"type\":\"odom_tf_lookup\""));
  EXPECT_TRUE(contains(lines[1], "\"update_index\":42"));
  EXPECT_TRUE(contains(lines[2], "\"type\":\"motion_delta\""));
  EXPECT_TRUE(
    contains(lines[2], "\"motion_model_type\":\"nav2_amcl::DifferentialMotionModel\""));

  rec.stop();
  lines = splitLines(readWholeFile(tf.path()));
  ASSERT_EQ(lines.size(), 4u);
  EXPECT_EQ(
    lines[3],
    coco_nav_diag::DiagRecorder::serializeSessionEnd(2, 0, 2, 0, 42));
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
  ASSERT_EQ(lines.size(), 52u);
  EXPECT_TRUE(contains(lines.front(), "\"type\":\"diag_session_start\""));
  EXPECT_EQ(lines.back(), coco_nav_diag::DiagRecorder::serializeSessionEnd(50, 0, 50, 0, 49));
  EXPECT_EQ(rec.recordedCount(), 50u);
  EXPECT_EQ(rec.writtenCount(), 50u);
  EXPECT_EQ(rec.droppedCount(), 0u);
}

TEST(DiagRecorderEnabled, StopIsIdempotentAndWritesExactlyOneFooter)
{
  TempFile tf;
  coco_nav_diag::DiagRecorder rec;
  rec.configure(true, tf.path(), 10, {}, /*async_flush=*/false);
  rec.recordOdomTfLookup(makeSuccessEvent());
  rec.stop();
  rec.stop();
  auto lines = splitLines(readWholeFile(tf.path()));
  ASSERT_EQ(lines.size(), 3u);
  int footers = 0;
  for (const auto & line : lines) {
    if (contains(line, "\"type\":\"diag_session_end\"")) {++footers;}
  }
  EXPECT_EQ(footers, 1);
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
  EXPECT_TRUE(contains(s, "\"lookup_success\":false"));
  EXPECT_TRUE(contains(s, "\"consecutive_failures\":3"));
  // A failure event's pose fields default-construct to zero, distinct from
  // "we looked and it really was the origin" only by lookup_success==false
  // -- offline analysis must gate on lookup_success, never treat a zero
  // odom_x/odom_y as a successful reading.
  EXPECT_TRUE(contains(s, "\"odom_x\":0.000000000"));
  // Quote and backslash in the exception text must be escaped so the line
  // stays valid JSON.
  EXPECT_TRUE(contains(s, "\\\"10.000\\\""));
  EXPECT_TRUE(contains(s, "\\\\[base_footprint]"));
  EXPECT_FALSE(contains(s, "\n"));  // one JSONL line, no raw newline
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
  ASSERT_EQ(lines.size(), 2u);
  EXPECT_TRUE(contains(lines[1], "\"lookup_success\":false"));
  EXPECT_TRUE(contains(lines[1], "\"error_message\":\"extrapolation\""));
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
    "{\"schema_version\":2,\"type\":\"odom_tf_lookup\",\"update_index\":42,"
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
  EXPECT_TRUE(contains(s, "\"type\":\"diag_session_start\""));
  EXPECT_TRUE(contains(s, "\"node_name\":\"amcl\""));
  EXPECT_TRUE(contains(s, "\"max_events\":200000"));
  EXPECT_TRUE(contains(s, "\"pid\":12345"));
}

TEST(DiagRecorderSchema, SessionEndExactFormat)
{
  EXPECT_EQ(
    coco_nav_diag::DiagRecorder::serializeSessionEnd(5, 1, 4, 0, 9),
    "{\"schema_version\":2,\"type\":\"diag_session_end\",\"recorded\":5,"
    "\"dropped\":1,\"written\":4,\"write_failures\":0,\"last_update_index\":9}");
}

// ---------------------------------------------------------------------
// Bounded queue / drop accounting
// ---------------------------------------------------------------------

TEST(DiagRecorderBounded, DropsBeyondCapAndFooterCountsThem)
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
  ASSERT_EQ(lines.size(), 4u);
  // last_update_index is the last RECORDED event, not the last dropped one.
  EXPECT_EQ(lines.back(), coco_nav_diag::DiagRecorder::serializeSessionEnd(2, 3, 2, 0, 1));
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
