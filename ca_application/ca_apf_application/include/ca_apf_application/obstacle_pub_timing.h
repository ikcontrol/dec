/*
   Copyright 2023 IKERLAN S. Coop.

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
*/


#ifndef APP_APF_COLLISION_AVOIDANCE_OBSTACLE_PUB_TIMING_H
#define APP_APF_COLLISION_AVOIDANCE_OBSTACLE_PUB_TIMING_H

#include <chrono>  // NOLINT
#include <string>
#include <array>
#include <fstream>

#define OBSTACLES_PUB_TIMING_N 5000
#define OBSTACLES_PUB_TIMINGS_PATH "/home/ikerlan/obstacle_pub_timing.csv"

using namespace std::chrono;  // NOLINT

namespace app_apf_collision_avoidance
{
struct ObstaclesPubTimingEntry
{
  int64_t between_calls;
  int64_t duration;
  int64_t post_proc_duration;
  int64_t distances_duration;
};

template <size_t N>
class ObstaclesPubTimer
{
private:
  std::string path_;

  std::array<ObstaclesPubTimingEntry, N> data_;
  size_t count_;

  system_clock::time_point call_time_;
  system_clock::time_point last_call_time_;
  system_clock::time_point post_proc_end_time_;

public:
  ObstaclesPubTimer();
  explicit ObstaclesPubTimer(const std::string path);
  ~ObstaclesPubTimer();

  void justCalled();
  void postprocEnded();
  void justEnded();

  void save();
};

template <size_t N>
ObstaclesPubTimer<N>::ObstaclesPubTimer()
{
  path_ = "timings.csv";
  count_ = 0;
  last_call_time_ = high_resolution_clock::now();
}

template <size_t N>
ObstaclesPubTimer<N>::ObstaclesPubTimer(const std::string path)
{
  path_ = path;
  count_ = 0;
  last_call_time_ = high_resolution_clock::now();
}

template <size_t N>
ObstaclesPubTimer<N>::~ObstaclesPubTimer() {}

template <size_t N>
void ObstaclesPubTimer<N>::justCalled()
{
  call_time_ = high_resolution_clock::now();
}

template <size_t N>
void ObstaclesPubTimer<N>::postprocEnded()
{
  post_proc_end_time_ = high_resolution_clock::now();
}

template <size_t N>
void ObstaclesPubTimer<N>::justEnded()
{
  if (count_ < N)
  {
    auto now = high_resolution_clock::now();

    auto from_last_call = duration_cast<nanoseconds>(call_time_ - last_call_time_);
    auto postproc_duration = duration_cast<nanoseconds>(post_proc_end_time_ - call_time_);
    auto distances_duration = duration_cast<nanoseconds>(now - post_proc_end_time_);
    auto duration = duration_cast<nanoseconds>(now - call_time_);

    ObstaclesPubTimingEntry e;
    e.between_calls = from_last_call.count();
    e.duration = duration.count();
    e.distances_duration = distances_duration.count();
    e.post_proc_duration = postproc_duration.count();

    data_[count_] = e;
    ++count_;

    last_call_time_  = call_time_;
  }
  else if (count_ == N)
  {
    std::cout << "Saving times to " << path_ << std::endl;
    save();
    ++count_;
  }
}

template <size_t N>
void ObstaclesPubTimer<N>::save()
{
  std::ofstream outFile(path_);

  outFile << "period,duration,postproc_duration,distances_proc_duration" << "\n";
  for (int i = 0; i < count_; i++)
  {
    const auto& e = data_[i];
    outFile << e.between_calls << "," << e.duration << ","
            << e.post_proc_duration << "," << e.distances_duration << "\n";
  }

  outFile.close();
}

}  // namespace app_apf_collision_avoidance

#endif  // APP_APF_COLLISION_AVOIDANCE_OBSTACLE_PUB_TIMING_H
