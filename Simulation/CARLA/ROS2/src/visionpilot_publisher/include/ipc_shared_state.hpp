//
// Created by atanasko on 2/12/26.
//

#ifndef VISIONPILOT_PUBLISHER_IPC_BRIDGE_HPP
#define VISIONPILOT_PUBLISHER_IPC_BRIDGE_HPP

#include <cstddef>

struct ControlState {
  double steering;
  double velocity;
};

class IpcSharedState {
public:
  IpcSharedState(const char* name);
  ~IpcSharedState();

  ControlState* get();

private:
  const char* name_;
  int fd_;
  void* ptr_;
};

#endif //VISIONPILOT_PUBLISHER_IPC_BRIDGE_HPP