// C2-NAV.36 diagnostic instrumentation.
//
// Standalone, ROS-free bounded JSONL recorder. Deliberately has no
// dependency on rclcpp/tf2/geometry_msgs so it can be unit-tested without a
// ROS runtime. AmclNode (coco_nav_diag/amcl_node.hpp) extracts primitive
// fields from ROS types before calling into this class.
//
// Default-disabled: a DiagRecorder that is never configure()'d with
// enabled=true never allocates a queue entry, never opens a file, and never
// starts a thread. record*() calls degrade to a single bool check.
#ifndef COCO_NAV_DIAG__DIAG_RECORDER_HPP_
#define COCO_NAV_DIAG__DIAG_RECORDER_HPP_

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <fstream>
#include <functional>
#include <mutex>
#include <string>
#include <thread>

namespace coco_nav_diag
{

// Schema version for the emitted JSONL. Bump on any field
// addition/removal/rename so offline tooling can detect a mismatch.
constexpr int kDiagSchemaVersion = 1;

// Event recorded at the exact call site of AmclNode::getOdomPose(), i.e. the
// scan-stamped odom -> base_footprint TF lookup AMCL itself performs. One
// event per laserReceived() invocation that reaches this call.
struct OdomTfLookupEvent
{
  uint64_t update_index = 0;          // laserReceived() invocation counter
  int32_t scan_stamp_sec = 0;
  uint32_t scan_stamp_nanosec = 0;
  std::string base_frame_id;          // source frame queried (base_footprint)
  std::string odom_frame_id;          // target frame queried (odom)
  bool lookup_success = false;

  // Valid only when lookup_success is true: the exact transform AMCL
  // consumed (tf_buffer_->transform() output), not a second/re-derived one.
  double odom_x = 0.0;
  double odom_y = 0.0;
  double odom_yaw = 0.0;
  double odom_qx = 0.0;
  double odom_qy = 0.0;
  double odom_qz = 0.0;
  double odom_qw = 0.0;
  int32_t odom_pose_stamp_sec = 0;
  uint32_t odom_pose_stamp_nanosec = 0;

  // Valid only when lookup_success is false.
  std::string error_message;
  int32_t consecutive_failures = 0;

  int32_t node_now_sec = 0;
  uint32_t node_now_nanosec = 0;
};

// Event recorded at the exact call site of the anchor advance / motion-delta
// computation inside laserReceived(), immediately around the (possible)
// call to motion_model_->odometryUpdate(pf_, pose, delta). One event per
// laserReceived() invocation that reaches an odometry-consuming pose (i.e.
// survived the TF lookup).
struct MotionDeltaEvent
{
  uint64_t update_index = 0;
  int32_t scan_stamp_sec = 0;
  uint32_t scan_stamp_nanosec = 0;

  // True on the pf_init_ == false branch: this cycle only sets the anchor,
  // it does not call the motion model. anchor_* / delta_* / delta_rot*
  // fields are not meaningful in this case (anchor_valid is false).
  bool is_anchor_init = false;
  bool anchor_valid = false;

  // pf_odom_pose_ BEFORE this cycle's update (the anchor the motion model
  // actually differenced against, read before updateFilter() can advance
  // it later in the same laserReceived() call).
  double anchor_x = 0.0;
  double anchor_y = 0.0;
  double anchor_yaw = 0.0;
  // The update_index of the laserReceived() cycle that last SET this
  // anchor (0 = none yet). Lets offline analysis look up the anchor's own
  // exact scan_stamp from that update_index's own recorded event, instead
  // of reconstructing it by matching anchor_x/y/yaw against a prior pose_*
  // by value -- exactly "the anchor/reference transform where applicable"
  // C2-NAV.36 asked for. Meaningful only when anchor_valid is true.
  uint64_t anchor_update_index = 0;

  // The scan-stamped odom pose from getOdomPose() this cycle (same values
  // as the paired OdomTfLookupEvent for this update_index).
  double pose_x = 0.0;
  double pose_y = 0.0;
  double pose_yaw = 0.0;

  // Raw odom-frame delta = pose - anchor (shouldUpdateFilter()'s out-param),
  // exactly as passed to motion_model_->odometryUpdate() when it runs.
  double delta_x = 0.0;
  double delta_y = 0.0;
  double delta_yaw = 0.0;

  // Derived via nav2_amcl::angleutils::angle_diff on delta_x/delta_y/
  // delta_yaw and anchor_yaw -- the identical header function and identical
  // input values nav2_amcl::DifferentialMotionModel::odometryUpdate uses
  // internally (old_pose.v[2] == anchor_yaw algebraically, since old_pose =
  // pose - delta = anchor). Only meaningful when motion_model_type ==
  // "nav2_amcl::DifferentialMotionModel"; see motion_model_formula_applicable.
  bool motion_model_formula_applicable = false;
  double delta_rot1 = 0.0;
  double delta_trans = 0.0;
  double delta_rot2 = 0.0;
  std::string motion_model_type;

  // Whether shouldUpdateFilter() judged the threshold exceeded, and whether
  // motion_model_->odometryUpdate() was actually invoked this cycle. These
  // can differ: a freshly-added laser starts with lasers_update_[i] = true
  // regardless of threshold, and a stale true flag from a previous laser
  // can also drive motion_update_invoked without should_update_filter
  // firing this cycle. Do not conflate either with "sensor update ran".
  bool should_update_filter = false;
  bool motion_update_invoked = false;

  int32_t node_now_sec = 0;
  uint32_t node_now_nanosec = 0;
};

// Called with a short human-readable message on: enable requested with no
// output path (capture stays off), open failure, and queue-full drops
// (rate-limited by the caller, not by this class).
using WarnFn = std::function<void (const std::string &)>;

class DiagRecorder
{
public:
  DiagRecorder() = default;
  ~DiagRecorder();

  DiagRecorder(const DiagRecorder &) = delete;
  DiagRecorder & operator=(const DiagRecorder &) = delete;

  // Configure and, if enabled and output_path is non-empty, open the output
  // file and (if async_flush) start the background writer thread. Safe to
  // call at most once; a second call is a no-op (logs via warn).
  //
  // async_flush=true is the production path: record*() only ever enqueues
  // (bounded, drop-counted) and a dedicated thread performs the blocking
  // file I/O, so a slow disk cannot add latency to the AMCL scan callback.
  // async_flush=false is for deterministic tests: record*() still only
  // enqueues; nothing is written until flushSync() or stop() is called
  // explicitly, so tests can assert on queued/dropped counts without a
  // race against a live drain thread.
  void configure(
    bool enabled, const std::string & output_path, size_t max_events,
    WarnFn warn = {}, bool async_flush = true);

  // enabled() reflects whether capture is actually active (enabled==true
  // AND output_path was non-empty AND the file opened successfully). This
  // is the fast-path check record*() uses; when false, record*() is a
  // single branch and returns immediately.
  bool enabled() const {return enabled_.load(std::memory_order_relaxed);}

  void recordOdomTfLookup(const OdomTfLookupEvent & ev);
  void recordMotionDelta(const MotionDeltaEvent & ev);

  // Writes every currently-queued line to disk and flushes the stream.
  // Only meaningful with async_flush=false (the background thread already
  // does this continuously when async_flush=true). No-op if disabled.
  void flushSync();

  // Stops the background thread (if any) after draining the queue, and
  // flushes/closes the output file. Idempotent. Called by the destructor.
  void stop();

  uint64_t recordedCount() const {return recorded_count_.load(std::memory_order_relaxed);}
  uint64_t droppedCount() const {return dropped_count_.load(std::memory_order_relaxed);}
  uint64_t queuedCount() const;

  // Pure, side-effect-free serialization -- exposed for unit tests that
  // want to assert on exact JSONL text without touching a file or thread.
  static std::string serialize(const OdomTfLookupEvent & ev);
  static std::string serialize(const MotionDeltaEvent & ev);
  static std::string serializeSessionStart(
    const std::string & node_name, size_t max_events, uint64_t pid);

private:
  bool tryEnqueue(std::string && line);
  void writerLoop();
  void drainLocked(std::unique_lock<std::mutex> & lock);

  std::atomic<bool> enabled_{false};
  bool configured_{false};
  bool async_flush_{true};
  size_t max_events_{0};
  WarnFn warn_;

  mutable std::mutex mutex_;
  std::condition_variable cv_;
  std::deque<std::string> queue_;
  bool stop_requested_{false};
  std::thread writer_thread_;

  std::ofstream out_;

  std::atomic<uint64_t> recorded_count_{0};
  std::atomic<uint64_t> dropped_count_{0};
};

}  // namespace coco_nav_diag

#endif  // COCO_NAV_DIAG__DIAG_RECORDER_HPP_
