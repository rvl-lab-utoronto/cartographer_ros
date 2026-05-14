-- Copyright 2018 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

options = {
  tracking_frame = "base_link",
  pipeline = {
    {
      action = "min_max_range_filter", -- Filters all points that are farther away from their 'origin' as 'max_range' or closer than 'min_range'
      min_range = 0.5,
      max_range = 200, -- default 60
    },
    {
      action = "vertical_range_filter",
      min_z = 0.2,
      max_z = 2.7,
    },
    {
      action = "voxel_filter_and_remove_moving_objects", -- Voxel filters the data and only passes on points that we believe are on non-moving objects.
      voxel_size = 0.05
    },
    {
      action = "write_ros_map",
      range_data_inserter = {
        insert_free_space = true,
        hit_probability = 0.55,
        miss_probability = 0.49,
      },
      filestem = "map",
      resolution = 0.25,
    }
  }
}

return options
