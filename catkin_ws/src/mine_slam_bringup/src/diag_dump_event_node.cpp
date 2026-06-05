#include <ros/ros.h>

#include "mine_slam_bringup/DumpEvent.h"
#include "mine_slam_bringup/event_dump.h"

namespace mine_slam_bringup {

class DiagDumpEventNode {
 public:
  DiagDumpEventNode() : private_nh_("~") {
    private_nh_.param("service_name", service_name_, std::string("/diag/dump_event"));
    service_ = nh_.advertiseService(service_name_, &DiagDumpEventNode::callback, this);
  }

 private:
  bool callback(DumpEvent::Request& request, DumpEvent::Response& response) {
    EventDumpRequest dump_request;
    dump_request.session_dir = request.session_dir;
    dump_request.output_root = request.output_root;
    dump_request.event_id = request.event_id;
    dump_request.event_time_s = request.event_time_s;
    dump_request.window_before_s = request.window_before_s;
    dump_request.window_after_s = request.window_after_s;
    dump_request.reason = request.reason;
    const EventDumpResult result = createEventDump(dump_request);
    response.success = result.success;
    response.message = result.message;
    response.event_dir = result.event_dir;
    return true;
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::ServiceServer service_;
  std::string service_name_;
};

}  // namespace mine_slam_bringup

int main(int argc, char** argv) {
  ros::init(argc, argv, "diag_dump_event");
  mine_slam_bringup::DiagDumpEventNode node;
  ros::spin();
  return 0;
}
