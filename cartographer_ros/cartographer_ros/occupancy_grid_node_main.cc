/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <algorithm>
#include <array>
#include <cmath>
#include <map>
#include <set>
#include <cstdio>
#include <cstdint>
#include <fstream>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "absl/synchronization/mutex.h"
#include "cairo/cairo.h"
#include "cartographer/common/port.h"
#include "cartographer/io/image.h"
#include "cartographer/io/submap_painter.h"
#include "cartographer/mapping/id.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"
#include "cartographer_ros/msg_conversion.h"
#include "cartographer_ros/node_constants.h"
#include "cartographer_ros/ros_log_sink.h"
#include "cartographer_ros/submap.h"
#include "cartographer_ros_msgs/SubmapList.h"
#include "cartographer_ros_msgs/SubmapQuery.h"
#include "gflags/gflags.h"
#include "nav_msgs/OccupancyGrid.h"
#include "ros/ros.h"

DEFINE_double(resolution, 0.05,
              "Resolution of a grid cell in the published occupancy grid.");
DEFINE_double(publish_period_sec, 1.0, "OccupancyGrid publishing period.");
DEFINE_bool(include_frozen_submaps, true,
            "Include frozen submaps in the occupancy grid.");
DEFINE_bool(include_unfrozen_submaps, true,
            "Include unfrozen submaps in the occupancy grid.");
DEFINE_bool(incremental, true,
            "Repaint only the region the unfrozen (live) submaps touch and "
            "splice it into the cached grid, instead of repainting every "
            "submap. Set =false to fall back to full repaints.");
DEFINE_bool(selfcheck, false,
            "TEST ONLY: after every incremental repaint also do a full repaint "
            "and log how many cells differ. Costs a full repaint per change.");
DEFINE_string(frozen_grid_cache, "",
              "Path of a cache of the painted FROZEN map. If it exists and its key "
              "matches the frozen submaps in /submap_list, it is loaded and no "
              "frozen texture is ever fetched or held; otherwise the map is built "
              "normally and the cache is (re)written for the next start.");
DEFINE_string(occupancy_grid_topic, cartographer_ros::kOccupancyGridTopic,
              "Name of the topic on which the occupancy grid is published.");

namespace cartographer_ros {
namespace {

using ::cartographer::io::PaintSubmapSlicesResult;
using ::cartographer::io::SubmapSlice;
using ::cartographer::mapping::SubmapId;

// submap_painter.cc keeps its ToEigen(Rigid3d) file-local; same thing here.
Eigen::Affine3d RigidToAffine(const ::cartographer::transform::Rigid3d& r) {
  return Eigen::Translation3d(r.translation()) * r.rotation();
}

void ApplySliceTransform(cairo_t* cr, const SubmapSlice& slice, const double resolution) {
  cairo_scale(cr, 1. / resolution, 1. / resolution);
  const Eigen::Matrix4d homo = RigidToAffine(slice.pose * slice.slice_pose).matrix();
  cairo_matrix_t matrix;
  cairo_matrix_init(&matrix, homo(1, 0), homo(0, 0), -homo(1, 1), -homo(0, 1),
                    homo(0, 3), -homo(1, 3));
  cairo_transform(cr, &matrix);
  cairo_scale(cr, slice.resolution, slice.resolution);
}

// Device-pixel bounding box of one slice in the UNTRANSLATED painter frame
// (world scaled by 1/resolution, y down), the frame PaintSubmapSlices measures
// its canvas in. Same matrix as CairoPaintSubmapSlices, so region compositing
// lands on exactly the pixels a full repaint would.
Eigen::AlignedBox2f SliceDeviceBox(const SubmapSlice& slice, const double resolution) {
  Eigen::AlignedBox2f box;
  auto surface = ::cartographer::io::MakeUniqueCairoSurfacePtr(
      cairo_image_surface_create(CAIRO_FORMAT_ARGB32, 1, 1));
  auto cr = ::cartographer::io::MakeUniqueCairoPtr(cairo_create(surface.get()));
  ApplySliceTransform(cr.get(), slice, resolution);
  const double xs[4] = {0., double(slice.width), 0., double(slice.width)};
  const double ys[4] = {0., 0., double(slice.height), double(slice.height)};
  for (int i = 0; i < 4; ++i) {
    double x = xs[i], y = ys[i];
    cairo_user_to_device(cr.get(), &x, &y);
    box.extend(Eigen::Vector2f(x, y));
  }
  return box;
}

void PaintOneSlice(cairo_t* cr, const SubmapSlice& slice, const double resolution) {
  if (slice.surface == nullptr) return;
  cairo_save(cr);
  ApplySliceTransform(cr, slice, resolution);
  cairo_set_source_surface(cr, slice.surface.get(), 0., 0.);
  cairo_paint(cr);
  cairo_restore(cr);
}

inline bool PixelObserved(const uint32_t packed) { return ((packed >> 8) & 0xff) != 0; }
inline int8_t PixelToValue(const uint32_t packed) {
  const unsigned char color = packed >> 16;
  return PixelObserved(packed)
             ? static_cast<int8_t>(::cartographer::common::RoundToInt((1. - color / 255.) * 100.))
             : -1;
}

// A painted grid with the painter's origin, the form CreateOccupancyGridMsg
// consumes and the form the frozen cache stores.
struct Grid {
  int width = 0, height = 0;
  Eigen::Array2f origin{0.f, 0.f};   // device coords of world (0,0), incl. padding
  std::vector<int8_t> data;          // row 0 = BOTTOM (device y = height-1)
  int8_t at_device(int dx, int dy) const {
    if (dx < 0 || dy < 0 || dx >= width || dy >= height) return -1;
    return data[static_cast<size_t>(height - 1 - dy) * width + dx];
  }
};

const char kCacheMagic[] = "CARTO_FROZEN_GRID_1";

class Node {
 public:
  explicit Node(double resolution, double publish_period_sec);
  ~Node() {}
  Node(const Node&) = delete;
  Node& operator=(const Node&) = delete;

 private:
  void HandleSubmapList(const cartographer_ros_msgs::SubmapList::ConstPtr& msg);
  void DrawAndPublish(const ::ros::WallTimerEvent& timer_event);
  void FetchInto(const SubmapId& id, SubmapSlice* slice, bool* dirty) EXCLUSIVE_LOCKS_REQUIRED(mutex_);
  void FetchPendingLive() EXCLUSIVE_LOCKS_REQUIRED(mutex_);
  void UpdateBox(const SubmapId& id) EXCLUSIVE_LOCKS_REQUIRED(mutex_);
  void FullRepaint() EXCLUSIVE_LOCKS_REQUIRED(mutex_);
  void ResetToFrozenCache() EXCLUSIVE_LOCKS_REQUIRED(mutex_);
  void RegionRepaint() EXCLUSIVE_LOCKS_REQUIRED(mutex_);
  void Grow(const Eigen::AlignedBox2i& want) EXCLUSIVE_LOCKS_REQUIRED(mutex_);
  Eigen::AlignedBox2i LiveBoxGrid() const EXCLUSIVE_LOCKS_REQUIRED(mutex_);
  void SelfCheck() EXCLUSIVE_LOCKS_REQUIRED(mutex_);
  bool PaintFrozenOnly(Grid* out) const EXCLUSIVE_LOCKS_REQUIRED(mutex_);
  void MaybeSaveFrozenCache() EXCLUSIVE_LOCKS_REQUIRED(mutex_);
  bool LoadFrozenCache(const std::string& path, Grid* g, uint64_t* key, int* n) const;
  void SaveFrozenCache(const std::string& path, const Grid& g) const EXCLUSIVE_LOCKS_REQUIRED(mutex_);

  ::ros::NodeHandle node_handle_;
  const double resolution_;

  absl::Mutex mutex_;
  ::ros::ServiceClient client_ GUARDED_BY(mutex_);
  ::ros::Subscriber submap_list_subscriber_ GUARDED_BY(mutex_);
  ::ros::Publisher occupancy_grid_publisher_ GUARDED_BY(mutex_);
  std::map<SubmapId, SubmapSlice> submap_slices_ GUARDED_BY(mutex_);
  std::set<SubmapId> frozen_ids_ GUARDED_BY(mutex_);
  ::ros::WallTimer occupancy_grid_publisher_timer_;
  std::string last_frame_id_;
  ros::Time last_timestamp_;

  // INCREMENTAL REPAINT (2026-09-22). The stock node repainted every submap
  // each period; the 2026-09-08 fork change cached the grid and repainted only
  // on change, which made a large frozen map free to serve but meant any
  // change to a LIVE submap forced a full repaint of the whole frozen map, so
  // the localization launch excluded unfrozen submaps and nothing mapped off
  // the stored map was ever drawn. Now:
  //   full_dirty_  a frozen slice was added, removed, moved or re-fetched (or
  //                there are no frozen slices at all): full repaint, as before.
  //   live_dirty_  only unfrozen slices changed: composite the frozen slices
  //                that intersect the live region plus the live slices into a
  //                small surface covering where the live slices are now AND
  //                where they were last time (so a moved or trimmed live
  //                submap reverts to the frozen content underneath), convert
  //                it, splice it into the cached grid. Painting order is the
  //                map's id order, frozen trajectories before the live one, so
  //                the pixels equal a full repaint's.
  // The grid canvas GROWS when live slices reach past it (rare reallocation
  // with a margin) and never shrinks. Published only when it changed.
  bool full_dirty_ GUARDED_BY(mutex_) = true;
  bool live_dirty_ GUARDED_BY(mutex_) = false;
  std::unique_ptr<nav_msgs::OccupancyGrid> last_msg_ GUARDED_BY(mutex_);
  Eigen::Array2f origin_ GUARDED_BY(mutex_);
  Eigen::AlignedBox2i live_box_prev_ GUARDED_BY(mutex_);
  // Live submaps whose texture version changed since their last fetch: the
  // list arrives at ~3 Hz and the active submap's version bumps on every
  // inserted scan, but the grid repaints once per publish period, so fetch
  // (service call + gunzip + redraw of a multi-megabyte texture) there.
  std::set<SubmapId> pending_live_fetch_ GUARDED_BY(mutex_);
  // Device-pixel footprint per slice, computed when its pose or texture
  // changes; a frozen submap never moves, so exactly once.
  std::map<SubmapId, Eigen::AlignedBox2f> boxes_ GUARDED_BY(mutex_);

  // FROZEN GRID CACHE (2026-09-22). Building the frozen layer means fetching,
  // decompressing and drawing every frozen texture once (tens of seconds at
  // 100 percent for ~800 submaps) and then holding them (4.7 GB). With the
  // cache the painted frozen grid is loaded from disk instead, no frozen
  // texture is fetched or held, and the live region is composited over the
  // frozen VALUES: where a live submap observed a cell the live value wins,
  // elsewhere the frozen value stays. That is the one difference from the
  // texture path, which alpha-blends live over frozen; for a viz map, live
  // winning is the better answer anyway. Keyed by a hash of every frozen
  // submap's id and pose in /submap_list, so a cache from another map is
  // refused and rebuilt, never drawn.
  bool frozen_cached_ GUARDED_BY(mutex_) = false;
  Grid frozen_ GUARDED_BY(mutex_);
  uint64_t frozen_key_ GUARDED_BY(mutex_) = 0;   // from the first list seen
  int frozen_count_ GUARDED_BY(mutex_) = 0;
  bool key_ready_ GUARDED_BY(mutex_) = false;
  uint64_t cache_key_ GUARDED_BY(mutex_) = 0;    // from the loaded file
  int cache_count_ GUARDED_BY(mutex_) = 0;
  bool cache_loaded_ GUARDED_BY(mutex_) = false;
  bool cache_saved_ GUARDED_BY(mutex_) = false;
};

Node::Node(const double resolution, const double publish_period_sec)
    : resolution_(resolution),
      client_(node_handle_.serviceClient<::cartographer_ros_msgs::SubmapQuery>(
          kSubmapQueryServiceName)),
      submap_list_subscriber_(node_handle_.subscribe(
          kSubmapListTopic, kLatestOnlyPublisherQueueSize,
          boost::function<void(
              const cartographer_ros_msgs::SubmapList::ConstPtr&)>(
              [this](const cartographer_ros_msgs::SubmapList::ConstPtr& msg) {
                HandleSubmapList(msg);
              }))),
      occupancy_grid_publisher_(
          node_handle_.advertise<::nav_msgs::OccupancyGrid>(
              FLAGS_occupancy_grid_topic, kLatestOnlyPublisherQueueSize,
              true /* latched */)),
      occupancy_grid_publisher_timer_(
          node_handle_.createWallTimer(::ros::WallDuration(publish_period_sec),
                                       &Node::DrawAndPublish, this)) {
  if (!FLAGS_frozen_grid_cache.empty()) {
    absl::MutexLock locker(&mutex_);
    Grid g; uint64_t key = 0; int n = 0;
    if (LoadFrozenCache(FLAGS_frozen_grid_cache, &g, &key, &n)) {
      frozen_ = std::move(g); cache_key_ = key; cache_count_ = n; cache_loaded_ = true;
      LOG(INFO) << "frozen grid cache read: " << frozen_.width << "x" << frozen_.height
                << ", " << n << " frozen submaps, key " << std::hex << key << std::dec
                << "; will be used if the map's frozen submaps match";
    } else {
      LOG(WARNING) << "no usable frozen grid cache at " << FLAGS_frozen_grid_cache
                   << "; building normally and writing it";
    }
  }
}

// FNV-1a over (trajectory, index, pose to the mm / 0.1 mdeg) of every frozen
// entry, in id order. Same map -> same key, regardless of which node computes it.
uint64_t FrozenKey(const std::map<SubmapId, std::array<int64_t, 3>>& poses) {
  uint64_t h = 1469598103934665603ULL;
  auto mix = [&h](int64_t v) {
    for (int i = 0; i < 8; ++i) { h ^= (v >> (8 * i)) & 0xff; h *= 1099511628211ULL; }
  };
  for (const auto& kv : poses) {
    mix(kv.first.trajectory_id); mix(kv.first.submap_index);
    mix(kv.second[0]); mix(kv.second[1]); mix(kv.second[2]);
  }
  return h;
}

void Node::HandleSubmapList(
    const cartographer_ros_msgs::SubmapList::ConstPtr& msg) {
  absl::MutexLock locker(&mutex_);
  if (occupancy_grid_publisher_.getNumSubscribers() == 0) {
    return;
  }

  // The frozen key from the first list that carries frozen submaps.
  if (!key_ready_) {
    std::map<SubmapId, std::array<int64_t, 3>> poses;
    for (const auto& m : msg->submap) {
      if (!m.is_frozen) continue;
      const auto pose = ToRigid3d(m.pose);
      const double yaw = ::cartographer::transform::GetYaw(pose.rotation());
      poses[SubmapId{m.trajectory_id, m.submap_index}] = {{
          static_cast<int64_t>(std::llround(pose.translation().x() * 1000.0)),
          static_cast<int64_t>(std::llround(pose.translation().y() * 1000.0)),
          static_cast<int64_t>(std::llround(yaw * 1e4))}};
    }
    if (!poses.empty()) {
      frozen_key_ = FrozenKey(poses); frozen_count_ = poses.size(); key_ready_ = true;
      if (cache_loaded_) {
        if (cache_key_ == frozen_key_ && cache_count_ == frozen_count_) {
          frozen_cached_ = true;
          LOG(INFO) << "frozen grid cache MATCHES the loaded map (" << frozen_count_
                    << " frozen submaps): frozen textures will not be fetched";
        } else {
          LOG(ERROR) << "frozen grid cache does NOT match the loaded map (cache "
                     << cache_count_ << " submaps key " << std::hex << cache_key_
                     << ", map " << std::dec << frozen_count_ << " key " << std::hex
                     << frozen_key_ << std::dec << "); ignoring it, building normally, "
                     << "and rewriting it";
          frozen_ = Grid(); cache_loaded_ = false;
        }
      }
    }
  }

  std::set<SubmapId> submap_ids_to_delete;
  for (const auto& pair : submap_slices_) submap_ids_to_delete.insert(pair.first);

  for (const auto& submap_msg : msg->submap) {
    const SubmapId id{submap_msg.trajectory_id, submap_msg.submap_index};
    submap_ids_to_delete.erase(id);
    if ((submap_msg.is_frozen && !FLAGS_include_frozen_submaps) ||
        (!submap_msg.is_frozen && !FLAGS_include_unfrozen_submaps)) {
      continue;
    }
    if (submap_msg.is_frozen && frozen_cached_) {
      continue;   // the cache stands in for every frozen slice
    }
    bool& dirty = submap_msg.is_frozen ? full_dirty_ : live_dirty_;
    if (submap_msg.is_frozen) frozen_ids_.insert(id); else frozen_ids_.erase(id);
    const bool is_new = submap_slices_.count(id) == 0;
    SubmapSlice& submap_slice = submap_slices_[id];
    const auto pose = ToRigid3d(submap_msg.pose);
    const bool moved =
        is_new || !submap_slice.pose.translation().isApprox(pose.translation()) ||
        !submap_slice.pose.rotation().isApprox(pose.rotation());
    if (moved) dirty = true;
    submap_slice.pose = pose;
    submap_slice.metadata_version = submap_msg.submap_version;
    if (submap_slice.surface != nullptr &&
        submap_slice.version == submap_msg.submap_version) {
      if (moved) UpdateBox(id);
      continue;
    }
    if (!submap_msg.is_frozen) {
      pending_live_fetch_.insert(id);   // fetched once per publish period
      continue;
    }
    FetchInto(id, &submap_slice, &dirty);
  }

  for (const auto& id : submap_ids_to_delete) {
    if (frozen_ids_.count(id)) full_dirty_ = true; else live_dirty_ = true;
    frozen_ids_.erase(id);
    pending_live_fetch_.erase(id);
    boxes_.erase(id);
    submap_slices_.erase(id);
  }

  last_timestamp_ = msg->header.stamp;
  last_frame_id_ = msg->header.frame_id;
}

void Node::FetchInto(const SubmapId& id, SubmapSlice* submap_slice, bool* dirty) {
  auto fetched_textures = ::cartographer_ros::FetchSubmapTextures(id, &client_);
  if (fetched_textures == nullptr) return;
  CHECK(!fetched_textures->textures.empty());
  *dirty = true;
  submap_slice->version = fetched_textures->version;
  // First texture only: by convention the highest resolution one.
  const auto fetched_texture = fetched_textures->textures.begin();
  submap_slice->width = fetched_texture->width;
  submap_slice->height = fetched_texture->height;
  submap_slice->slice_pose = fetched_texture->slice_pose;
  submap_slice->resolution = fetched_texture->resolution;
  submap_slice->cairo_data.clear();
  submap_slice->surface = ::cartographer::io::DrawTexture(
      fetched_texture->pixels.intensity, fetched_texture->pixels.alpha,
      fetched_texture->width, fetched_texture->height, &submap_slice->cairo_data);
  UpdateBox(id);
}

void Node::FetchPendingLive() {
  for (const SubmapId& id : pending_live_fetch_) {
    auto it = submap_slices_.find(id);
    if (it == submap_slices_.end()) continue;
    FetchInto(id, &it->second, &live_dirty_);
  }
  pending_live_fetch_.clear();
}

void Node::UpdateBox(const SubmapId& id) {
  auto it = submap_slices_.find(id);
  if (it == submap_slices_.end() || it->second.surface == nullptr) { boxes_.erase(id); return; }
  boxes_[id] = SliceDeviceBox(it->second, resolution_);
}

Eigen::AlignedBox2i Node::LiveBoxGrid() const {
  Eigen::AlignedBox2f box;
  for (const auto& pair : boxes_) {
    if (frozen_ids_.count(pair.first)) continue;
    box.extend(pair.second);
  }
  if (box.isEmpty()) return Eigen::AlignedBox2i();
  const int pad = 5;   // same padding PaintSubmapSlices gives its canvas
  return Eigen::AlignedBox2i(
      Eigen::Vector2i(std::floor(box.min().x() + origin_.x()) - pad,
                      std::floor(box.min().y() + origin_.y()) - pad),
      Eigen::Vector2i(std::ceil(box.max().x() + origin_.x()) + pad,
                      std::ceil(box.max().y() + origin_.y()) + pad));
}

void Node::FullRepaint() {
  auto painted_slices = ::cartographer::io::PaintSubmapSlices(submap_slices_, resolution_);
  last_msg_ = CreateOccupancyGridMsg(painted_slices, resolution_, last_frame_id_, last_timestamp_);
  origin_ = painted_slices.origin;
  live_box_prev_ = LiveBoxGrid();
  full_dirty_ = live_dirty_ = false;
}

// Cached mode: the canvas starts as the frozen grid, then the live region is
// composited on top by RegionRepaint.
void Node::ResetToFrozenCache() {
  last_msg_ = absl::make_unique<nav_msgs::OccupancyGrid>();
  last_msg_->header.frame_id = last_frame_id_;
  last_msg_->info.map_load_time = last_timestamp_;
  last_msg_->info.resolution = resolution_;
  last_msg_->info.width = frozen_.width;
  last_msg_->info.height = frozen_.height;
  last_msg_->info.origin.position.x = -frozen_.origin.x() * resolution_;
  last_msg_->info.origin.position.y = (-frozen_.height + frozen_.origin.y()) * resolution_;
  last_msg_->info.origin.orientation.w = 1.;
  last_msg_->data = frozen_.data;
  origin_ = frozen_.origin;
  live_box_prev_ = Eigen::AlignedBox2i();
  full_dirty_ = false;
  live_dirty_ = true;
  RegionRepaint();
}

void Node::Grow(const Eigen::AlignedBox2i& want) {
  const int W = last_msg_->info.width, H = last_msg_->info.height;
  const int margin = static_cast<int>(50.0 / resolution_);   // 50 m
  const int gl = want.min().x() < 0 ? -want.min().x() + margin : 0;
  const int gt = want.min().y() < 0 ? -want.min().y() + margin : 0;
  const int gr = want.max().x() > W ? want.max().x() - W + margin : 0;
  const int gb = want.max().y() > H ? want.max().y() - H + margin : 0;
  if (!(gl || gt || gr || gb)) return;
  const int nW = W + gl + gr, nH = H + gt + gb;
  std::vector<int8_t> data(static_cast<size_t>(nW) * nH, -1);
  // data row r counts from the BOTTOM (device y = H-1-r): growing at the top
  // appends rows, growing at the bottom shifts rows up by gb.
  for (int r = 0; r < H; ++r) {
    std::copy(last_msg_->data.begin() + static_cast<size_t>(r) * W,
              last_msg_->data.begin() + static_cast<size_t>(r + 1) * W,
              data.begin() + static_cast<size_t>(r + gb) * nW + gl);
  }
  last_msg_->data.swap(data);
  last_msg_->info.width = nW;
  last_msg_->info.height = nH;
  origin_ += Eigen::Array2f(gl, gt);
  last_msg_->info.origin.position.x = -origin_.x() * resolution_;
  last_msg_->info.origin.position.y = (-nH + origin_.y()) * resolution_;
  live_box_prev_.translate(Eigen::Vector2i(gl, gt));
  LOG(INFO) << "occupancy grid grown to " << nW << "x" << nH << " (+" << gl << " left, +"
            << gr << " right, +" << gt << " top, +" << gb << " bottom)";
}

void Node::RegionRepaint() {
  Eigen::AlignedBox2i live = LiveBoxGrid();
  if (!live.isEmpty()) Grow(live);
  Eigen::AlignedBox2i region = live_box_prev_;
  if (!live.isEmpty()) region.extend(live);
  live_box_prev_ = live;
  live_dirty_ = false;
  if (region.isEmpty()) return;
  const int W = last_msg_->info.width, H = last_msg_->info.height;
  region = region.intersection(Eigen::AlignedBox2i(Eigen::Vector2i(0, 0), Eigen::Vector2i(W, H)));
  if (region.isEmpty()) return;
  const int rw = region.sizes().x(), rh = region.sizes().y();
  if (rw <= 0 || rh <= 0) return;

  auto surface = ::cartographer::io::MakeUniqueCairoSurfacePtr(
      cairo_image_surface_create(CAIRO_FORMAT_ARGB32, rw, rh));
  {
    auto cr = ::cartographer::io::MakeUniqueCairoPtr(cairo_create(surface.get()));
    if (frozen_cached_) {
      // TRANSPARENT background: the surface then holds the premultiplied live
      // composite alone, which is blended over the frozen values below with
      // cairo's own OVER rule. (OVER is associative, so blending the live
      // composite onto the frozen layer equals painting each slice in turn.)
      cairo_set_source_rgba(cr.get(), 0.0, 0.0, 0.0, 0.);
      cairo_set_operator(cr.get(), CAIRO_OPERATOR_SOURCE);
      cairo_paint(cr.get());
      cairo_set_operator(cr.get(), CAIRO_OPERATOR_OVER);
    } else {
      cairo_set_source_rgba(cr.get(), 0.5, 0.0, 0.0, 1.);   // "unobserved", as PaintSubmapSlices
      cairo_paint(cr.get());
    }
    // grid pixel = device + origin_; region pixel = grid pixel - region.min
    cairo_translate(cr.get(), origin_.x() - region.min().x(), origin_.y() - region.min().y());
    const Eigen::AlignedBox2f region_dev(
        Eigen::Vector2f(region.min().x() - origin_.x(), region.min().y() - origin_.y()),
        Eigen::Vector2f(region.max().x() - origin_.x(), region.max().y() - origin_.y()));
    // id order = frozen trajectories first, then the live one: same as a full repaint.
    // In cached mode there are no frozen slices here; the frozen VALUES are the base.
    for (const auto& pair : submap_slices_) {
      const SubmapSlice& slice = pair.second;
      if (slice.surface == nullptr) continue;
      if (frozen_ids_.count(pair.first)) {
        const auto bit = boxes_.find(pair.first);
        if (bit == boxes_.end() || !bit->second.intersects(region_dev)) continue;
      }
      PaintOneSlice(cr.get(), slice, resolution_);
    }
    cairo_surface_flush(surface.get());
  }
  const uint32_t* px = reinterpret_cast<uint32_t*>(cairo_image_surface_get_data(surface.get()));
  const int stride = cairo_image_surface_get_stride(surface.get()) / 4;
  for (int y = 0; y < rh; ++y) {
    const int dev_y = region.min().y() + y;
    int8_t* row = last_msg_->data.data() + static_cast<size_t>(H - 1 - dev_y) * W + region.min().x();
    const uint32_t* src = px + static_cast<size_t>(y) * stride;
    if (frozen_cached_) {
      // Exactly what a full repaint does, per pixel: the live layer (premultiplied
      // ARGB: A = confidence, R = intensity, G = observed flag) composited OVER the
      // frozen layer, whose painted colour is recovered from the cached value
      // (value = (1 - R/255) * 100, so R = 255 * (1 - value/100)); an unknown
      // frozen cell is the painter's background, R = 128, unobserved. A weak
      // live observation therefore barely moves a confident frozen cell, and a
      // "maybe occupied" first sighting no longer paints a grey patch over a
      // known-free floor. Rounding through the 0..100 value costs at most +-1.
      for (int x = 0; x < rw; ++x) {
        const int dev_x = region.min().x() + x;
        const uint32_t l = src[x];
        const int a_l = (l >> 24) & 0xff, r_l = (l >> 16) & 0xff, g_l = (l >> 8) & 0xff;
        const int8_t v_f = frozen_.at_device(dev_x - origin_.x() + frozen_.origin.x(),
                                             dev_y - origin_.y() + frozen_.origin.y());
        const double r_f = v_f >= 0 ? 255.0 * (1.0 - v_f / 100.0) : 128.0;
        const bool observed = g_l != 0 || v_f >= 0;
        if (!observed) { row[x] = -1; continue; }
        const double r_out = r_l + r_f * (255 - a_l) / 255.0;
        row[x] = static_cast<int8_t>(std::min(100, std::max(0,
            ::cartographer::common::RoundToInt((1.0 - r_out / 255.0) * 100.0))));
      }
    } else {
      for (int x = 0; x < rw; ++x) row[x] = PixelToValue(src[x]);
    }
  }
}

// Frozen slices only, painted the way PaintSubmapSlices would paint them
// (same bbox rule, same 5 px padding, same id order), so the cached grid is
// exactly the frozen part of a full repaint.
bool Node::PaintFrozenOnly(Grid* out) const {
  Eigen::AlignedBox2f box;
  for (const SubmapId& id : frozen_ids_) {
    auto it = submap_slices_.find(id);
    if (it == submap_slices_.end() || it->second.surface == nullptr) return false;
    const auto bit = boxes_.find(id);
    if (bit == boxes_.end()) return false;
    box.extend(bit->second);
  }
  if (box.isEmpty()) return false;
  const int pad = 5;
  out->width = std::ceil(box.sizes().x()) + 2 * pad;
  out->height = std::ceil(box.sizes().y()) + 2 * pad;
  out->origin = Eigen::Array2f(-box.min().x() + pad, -box.min().y() + pad);
  auto surface = ::cartographer::io::MakeUniqueCairoSurfacePtr(
      cairo_image_surface_create(CAIRO_FORMAT_ARGB32, out->width, out->height));
  {
    auto cr = ::cartographer::io::MakeUniqueCairoPtr(cairo_create(surface.get()));
    cairo_set_source_rgba(cr.get(), 0.5, 0.0, 0.0, 1.);
    cairo_paint(cr.get());
    cairo_translate(cr.get(), out->origin.x(), out->origin.y());
    for (const auto& pair : submap_slices_) {
      if (!frozen_ids_.count(pair.first)) continue;
      PaintOneSlice(cr.get(), pair.second, resolution_);
    }
    cairo_surface_flush(surface.get());
  }
  const uint32_t* px = reinterpret_cast<uint32_t*>(cairo_image_surface_get_data(surface.get()));
  const int stride = cairo_image_surface_get_stride(surface.get()) / 4;
  out->data.assign(static_cast<size_t>(out->width) * out->height, -1);
  for (int y = 0; y < out->height; ++y) {
    int8_t* row = out->data.data() + static_cast<size_t>(out->height - 1 - y) * out->width;
    const uint32_t* src = px + static_cast<size_t>(y) * stride;
    for (int x = 0; x < out->width; ++x) row[x] = PixelToValue(src[x]);
  }
  return true;
}

// File: one text header line, then raw int8 cells.
//   CARTO_FROZEN_GRID_1 <count> <key hex> <width> <height> <resolution> <origin_x> <origin_y>\n
void Node::SaveFrozenCache(const std::string& path, const Grid& g) const {
  const std::string tmp = path + ".tmp";
  std::ofstream f(tmp, std::ios::binary);
  if (!f) { LOG(ERROR) << "cannot write frozen grid cache " << tmp; return; }
  f << kCacheMagic << ' ' << frozen_count_ << ' ' << std::hex << frozen_key_ << std::dec << ' '
    << g.width << ' ' << g.height << ' ' << resolution_ << ' ' << g.origin.x() << ' '
    << g.origin.y() << '\n';
  f.write(reinterpret_cast<const char*>(g.data.data()), g.data.size());
  f.close();
  if (std::rename(tmp.c_str(), path.c_str()) != 0) { LOG(ERROR) << "cannot rename " << tmp; return; }
  LOG(INFO) << "frozen grid cache written: " << path << " (" << g.width << "x" << g.height << ", "
            << frozen_count_ << " frozen submaps, " << g.data.size() / 1048576 << " MB)";
}

bool Node::LoadFrozenCache(const std::string& path, Grid* g, uint64_t* key, int* n) const {
  std::ifstream f(path, std::ios::binary);
  if (!f) return false;
  std::string line;
  if (!std::getline(f, line)) return false;
  std::istringstream hs(line);
  std::string magic; double res = 0, ox = 0, oy = 0;
  hs >> magic >> *n >> std::hex >> *key >> std::dec >> g->width >> g->height >> res >> ox >> oy;
  if (!hs || magic != kCacheMagic) { LOG(WARNING) << "bad frozen grid cache header in " << path; return false; }
  if (std::fabs(res - resolution_) > 1e-9) {
    LOG(WARNING) << "frozen grid cache resolution " << res << " != " << resolution_ << "; ignoring it";
    return false;
  }
  g->origin = Eigen::Array2f(ox, oy);
  g->data.resize(static_cast<size_t>(g->width) * g->height);
  f.read(reinterpret_cast<char*>(g->data.data()), g->data.size());
  if (static_cast<size_t>(f.gcount()) != g->data.size()) { LOG(WARNING) << "short frozen grid cache " << path; return false; }
  return true;
}

// Once every frozen slice has its texture, write the cache (unless it was the
// one we loaded).
void Node::MaybeSaveFrozenCache() {
  if (FLAGS_frozen_grid_cache.empty() || cache_saved_ || frozen_cached_ || !key_ready_) return;
  if (frozen_ids_.empty()) return;
  Grid g;
  if (!PaintFrozenOnly(&g)) return;   // some frozen texture not fetched yet; try next period
  SaveFrozenCache(FLAGS_frozen_grid_cache, g);
  cache_saved_ = true;
  // Switch over now rather than on the next start: drop every frozen texture
  // (that is the 4.7 GB) and composite the live region over the cached
  // values from here on. Same effect as a restart with the cache present.
  frozen_ = std::move(g);
  frozen_cached_ = true;
  for (const SubmapId& id : frozen_ids_) { submap_slices_.erase(id); boxes_.erase(id); }
  frozen_ids_.clear();
  full_dirty_ = false;
  ResetToFrozenCache();
  LOG(INFO) << "switched to the frozen grid cache: frozen textures released, "
            << "live region now composited over cached values";
}

void Node::SelfCheck() {
  if (frozen_cached_) {
    LOG_EVERY_N(INFO, 60) << "selfcheck: not available with a cached frozen grid (no full repaint to compare against)";
    return;
  }
  auto painted = ::cartographer::io::PaintSubmapSlices(submap_slices_, resolution_);
  auto ref = CreateOccupancyGridMsg(painted, resolution_, last_frame_id_, last_timestamp_);
  const int Wr = ref->info.width, Hr = ref->info.height;
  const int W = last_msg_->info.width, H = last_msg_->info.height;
  const int dx = std::lround(origin_.x() - painted.origin.x());
  const int dy = std::lround(origin_.y() - painted.origin.y());
  size_t compared = 0, mismatched = 0;
  for (int y = 0; y < Hr; ++y) {
    const int my_y = y + dy;
    if (my_y < 0 || my_y >= H) continue;
    for (int x = 0; x < Wr; ++x) {
      const int my_x = x + dx;
      if (my_x < 0 || my_x >= W) continue;
      ++compared;
      if (ref->data[static_cast<size_t>(Hr - 1 - y) * Wr + x] !=
          last_msg_->data[static_cast<size_t>(H - 1 - my_y) * W + my_x]) ++mismatched;
    }
  }
  LOG(INFO) << "occupancy grid selfcheck: " << mismatched << " of " << compared
            << " cells differ from a full repaint (ref " << Wr << "x" << Hr
            << ", incremental " << W << "x" << H << ")";
}

void Node::DrawAndPublish(const ::ros::WallTimerEvent& unused_timer_event) {
  absl::MutexLock locker(&mutex_);
  if (last_frame_id_.empty()) return;
  if (occupancy_grid_publisher_.getNumSubscribers() == 0) return;
  FetchPendingLive();
  bool changed = false;
  if (frozen_cached_) {
    if (last_msg_ == nullptr) { ResetToFrozenCache(); changed = true; }
    else if (live_dirty_) { RegionRepaint(); changed = true; }
  } else {
    if (submap_slices_.empty()) return;
    if (last_msg_ == nullptr || full_dirty_ || frozen_ids_.empty() || !FLAGS_incremental) {
      if (last_msg_ == nullptr || full_dirty_ || live_dirty_) { FullRepaint(); changed = true; }
    } else if (live_dirty_) {
      RegionRepaint();
      if (FLAGS_selfcheck) SelfCheck();
      changed = true;
    }
    const bool was_cached = frozen_cached_;
    MaybeSaveFrozenCache();
    if (frozen_cached_ && !was_cached) changed = true;
  }
  // Publish only when something changed: the topic is latched, so a later
  // subscriber still gets the current grid, and republishing an identical
  // 17-megacell message every period was pure serialization and socket traffic.
  if (!changed || last_msg_ == nullptr) return;
  last_msg_->header.stamp = last_timestamp_;
  occupancy_grid_publisher_.publish(*last_msg_);
}

}  // namespace
}  // namespace cartographer_ros

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  CHECK(FLAGS_include_frozen_submaps || FLAGS_include_unfrozen_submaps)
      << "Ignoring both frozen and unfrozen submaps makes no sense.";

  ::ros::init(argc, argv, "cartographer_occupancy_grid_node");
  ::ros::start();

  cartographer_ros::ScopedRosLogSink ros_log_sink;
  ::cartographer_ros::Node node(FLAGS_resolution, FLAGS_publish_period_sec);

  ::ros::spin();
  ::ros::shutdown();
}
