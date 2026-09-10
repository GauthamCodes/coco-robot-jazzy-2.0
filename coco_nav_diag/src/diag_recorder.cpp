#include "coco_nav_diag/diag_recorder.hpp"

#include <cstdio>

namespace coco_nav_diag
{

namespace
{

// Locale-independent-enough fixed-format double. snprintf's "%.9f" is
// sensitive to LC_NUMERIC in principle; every writer in this codebase
// (nav_bench.py, c2nav34_odom.py) already assumes the process locale does
// not change mid-run, so the same assumption is made here and is asserted
// by the recorder's own selftest via exact-string-match on known inputs.
std::string formatDouble(double v)
{
  char buf[64];
  int n = std::snprintf(buf, sizeof(buf), "%.9f", v);
  return std::string(buf, buf + (n > 0 ? n : 0));
}

std::string jsonEscape(const std::string & s)
{
  std::string out;
  out.reserve(s.size() + 8);
  for (unsigned char c : s) {
    switch (c) {
      case '"': out += "\\\""; break;
      case '\\': out += "\\\\"; break;
      case '\n': out += "\\n"; break;
      case '\r': out += "\\r"; break;
      case '\t': out += "\\t"; break;
      default:
        if (c < 0x20) {
          char buf[8];
          std::snprintf(buf, sizeof(buf), "\\u%04x", c);
          out += buf;
        } else {
          out += static_cast<char>(c);
        }
    }
  }
  return out;
}

std::string jsonField(const char * key, const std::string & value, bool trailing_comma = true)
{
  std::string out = "\"";
  out += key;
  out += "\":\"";
  out += jsonEscape(value);
  out += "\"";
  if (trailing_comma) {out += ",";}
  return out;
}

std::string jsonField(const char * key, double value, bool trailing_comma = true)
{
  std::string out = "\"";
  out += key;
  out += "\":";
  out += formatDouble(value);
  if (trailing_comma) {out += ",";}
  return out;
}

std::string jsonField(const char * key, int64_t value, bool trailing_comma = true)
{
  std::string out = "\"";
  out += key;
  out += "\":";
  out += std::to_string(value);
  if (trailing_comma) {out += ",";}
  return out;
}

std::string jsonField(const char * key, bool value, bool trailing_comma = true)
{
  std::string out = "\"";
  out += key;
  out += "\":";
  out += (value ? "true" : "false");
  if (trailing_comma) {out += ",";}
  return out;
}

}  // namespace

DiagRecorder::~DiagRecorder()
{
  stop();
}

void DiagRecorder::configure(
  bool enabled, const std::string & output_path, size_t max_events,
  WarnFn warn, bool async_flush)
{
  if (configured_) {
    if (warn) {warn("DiagRecorder::configure called more than once; ignoring");}
    return;
  }
  configured_ = true;
  warn_ = std::move(warn);
  max_events_ = max_events;
  async_flush_ = async_flush;

  if (!enabled) {
    enabled_.store(false, std::memory_order_relaxed);
    return;
  }
  if (output_path.empty()) {
    if (warn_) {
      warn_(
        "C2-NAV.36 diagnostic capture requested (diag_enabled=true) but "
        "diag_output_path is empty; capture stays disabled (fail-safe)");
    }
    enabled_.store(false, std::memory_order_relaxed);
    return;
  }

  out_.open(output_path, std::ios::out | std::ios::trunc);
  if (!out_.is_open()) {
    if (warn_) {
      warn_("C2-NAV.36 diagnostic capture could not open '" + output_path + "'; capture disabled");
    }
    enabled_.store(false, std::memory_order_relaxed);
    return;
  }

  enabled_.store(true, std::memory_order_relaxed);

  if (async_flush_) {
    writer_thread_ = std::thread(&DiagRecorder::writerLoop, this);
  }
}

bool DiagRecorder::tryEnqueue(std::string && line)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (queue_.size() >= max_events_) {
    dropped_count_.fetch_add(1, std::memory_order_relaxed);
    return false;
  }
  queue_.push_back(std::move(line));
  recorded_count_.fetch_add(1, std::memory_order_relaxed);
  return true;
}

void DiagRecorder::recordOdomTfLookup(const OdomTfLookupEvent & ev)
{
  if (!enabled()) {return;}
  bool ok = tryEnqueue(serialize(ev));
  if (!ok && warn_) {
    warn_("C2-NAV.36 diagnostic queue full; dropping odom_tf_lookup event");
  }
  if (async_flush_) {cv_.notify_one();}
}

void DiagRecorder::recordMotionDelta(const MotionDeltaEvent & ev)
{
  if (!enabled()) {return;}
  bool ok = tryEnqueue(serialize(ev));
  if (!ok && warn_) {
    warn_("C2-NAV.36 diagnostic queue full; dropping motion_delta event");
  }
  if (async_flush_) {cv_.notify_one();}
}

void DiagRecorder::drainLocked(std::unique_lock<std::mutex> & lock)
{
  std::deque<std::string> local;
  local.swap(queue_);
  lock.unlock();
  for (const auto & line : local) {
    out_ << line << '\n';
  }
  out_.flush();
  lock.lock();
}

void DiagRecorder::flushSync()
{
  if (!enabled()) {return;}
  std::unique_lock<std::mutex> lock(mutex_);
  drainLocked(lock);
}

void DiagRecorder::writerLoop()
{
  std::unique_lock<std::mutex> lock(mutex_);
  while (true) {
    cv_.wait(lock, [this] {return !queue_.empty() || stop_requested_;});
    if (queue_.empty() && stop_requested_) {
      break;
    }
    drainLocked(lock);
  }
}

void DiagRecorder::stop()
{
  if (!configured_) {return;}
  bool was_enabled = enabled_.exchange(false, std::memory_order_relaxed);
  if (async_flush_ && writer_thread_.joinable()) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stop_requested_ = true;
    }
    cv_.notify_one();
    writer_thread_.join();
  } else if (was_enabled) {
    std::unique_lock<std::mutex> lock(mutex_);
    drainLocked(lock);
  }
  if (out_.is_open()) {
    out_.flush();
    out_.close();
  }
}

uint64_t DiagRecorder::queuedCount() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return queue_.size();
}

std::string DiagRecorder::serialize(const OdomTfLookupEvent & ev)
{
  std::string s = "{";
  s += jsonField("schema_version", static_cast<int64_t>(kDiagSchemaVersion));
  s += jsonField("type", std::string("odom_tf_lookup"));
  s += jsonField("update_index", static_cast<int64_t>(ev.update_index));
  s += jsonField("scan_stamp_sec", static_cast<int64_t>(ev.scan_stamp_sec));
  s += jsonField("scan_stamp_nanosec", static_cast<int64_t>(ev.scan_stamp_nanosec));
  s += jsonField("base_frame_id", ev.base_frame_id);
  s += jsonField("odom_frame_id", ev.odom_frame_id);
  s += jsonField("lookup_success", ev.lookup_success);
  s += jsonField("odom_x", ev.odom_x);
  s += jsonField("odom_y", ev.odom_y);
  s += jsonField("odom_yaw", ev.odom_yaw);
  s += jsonField("odom_qx", ev.odom_qx);
  s += jsonField("odom_qy", ev.odom_qy);
  s += jsonField("odom_qz", ev.odom_qz);
  s += jsonField("odom_qw", ev.odom_qw);
  s += jsonField("odom_pose_stamp_sec", static_cast<int64_t>(ev.odom_pose_stamp_sec));
  s += jsonField("odom_pose_stamp_nanosec", static_cast<int64_t>(ev.odom_pose_stamp_nanosec));
  s += jsonField("error_message", ev.error_message);
  s += jsonField("consecutive_failures", static_cast<int64_t>(ev.consecutive_failures));
  s += jsonField("node_now_sec", static_cast<int64_t>(ev.node_now_sec));
  s += jsonField("node_now_nanosec", static_cast<int64_t>(ev.node_now_nanosec), false);
  s += "}";
  return s;
}

std::string DiagRecorder::serialize(const MotionDeltaEvent & ev)
{
  std::string s = "{";
  s += jsonField("schema_version", static_cast<int64_t>(kDiagSchemaVersion));
  s += jsonField("type", std::string("motion_delta"));
  s += jsonField("update_index", static_cast<int64_t>(ev.update_index));
  s += jsonField("scan_stamp_sec", static_cast<int64_t>(ev.scan_stamp_sec));
  s += jsonField("scan_stamp_nanosec", static_cast<int64_t>(ev.scan_stamp_nanosec));
  s += jsonField("is_anchor_init", ev.is_anchor_init);
  s += jsonField("anchor_valid", ev.anchor_valid);
  s += jsonField("anchor_update_index", static_cast<int64_t>(ev.anchor_update_index));
  s += jsonField("anchor_x", ev.anchor_x);
  s += jsonField("anchor_y", ev.anchor_y);
  s += jsonField("anchor_yaw", ev.anchor_yaw);
  s += jsonField("pose_x", ev.pose_x);
  s += jsonField("pose_y", ev.pose_y);
  s += jsonField("pose_yaw", ev.pose_yaw);
  s += jsonField("delta_x", ev.delta_x);
  s += jsonField("delta_y", ev.delta_y);
  s += jsonField("delta_yaw", ev.delta_yaw);
  s += jsonField("motion_model_formula_applicable", ev.motion_model_formula_applicable);
  s += jsonField("delta_rot1", ev.delta_rot1);
  s += jsonField("delta_trans", ev.delta_trans);
  s += jsonField("delta_rot2", ev.delta_rot2);
  s += jsonField("motion_model_type", ev.motion_model_type);
  s += jsonField("should_update_filter", ev.should_update_filter);
  s += jsonField("motion_update_invoked", ev.motion_update_invoked);
  s += jsonField("node_now_sec", static_cast<int64_t>(ev.node_now_sec));
  s += jsonField("node_now_nanosec", static_cast<int64_t>(ev.node_now_nanosec), false);
  s += "}";
  return s;
}

std::string DiagRecorder::serializeSessionStart(
  const std::string & node_name, size_t max_events, uint64_t pid)
{
  std::string s = "{";
  s += jsonField("schema_version", static_cast<int64_t>(kDiagSchemaVersion));
  s += jsonField("type", std::string("diag_session_start"));
  s += jsonField("node_name", node_name);
  s += jsonField("max_events", static_cast<int64_t>(max_events));
  s += jsonField("pid", static_cast<int64_t>(pid), false);
  s += "}";
  return s;
}

}  // namespace coco_nav_diag
