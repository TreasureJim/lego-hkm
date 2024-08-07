#include "juliet_comms.hpp"
#include <cassert>
#include <mutex>

motion_command previous_command;
motion_command current_command;
motion_command next_command;

// assumes there are commands in queue
void get_commands() {
}

void execute_command() {
  // TODO
  assert(false);
}

void robot_thread_func() {
  while(true) {
    // check if queue is empty
    {
      std::unique_lock queue_lock(motion_queue_mutex);

      if (motion_queue.empty()) {
        queue_lock.unlock();
        // wait for notification
        motion_queue_trigger.wait(queue_lock);
        get_commands();
      } else {
        get_commands();
      }
    }
    
    execute_command();
  }
}
