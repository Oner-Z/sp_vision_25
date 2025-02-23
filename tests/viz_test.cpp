#include <chrono>
#include <thread>

using namespace std::chrono_literals;
#include "io/ros2/visualizer.hpp"
#include "tasks/mono_loc/arena.hpp"
#include "tools/exiter.hpp"
int main()
{
  mono_loc::Arena arena("configs/radar.yaml");
  io::Visualizer visualizer;
  tools::Exiter exiter;
  while (!exiter.exit()) {
    visualizer.visualize(arena);
    std::this_thread::sleep_for(1s);
    /* code */
  }
}