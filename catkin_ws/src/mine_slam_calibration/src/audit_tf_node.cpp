#include <ros/ros.h>

#include "mine_slam_calibration/AuditTf.h"
#include "mine_slam_calibration/extrinsics.h"

namespace mine_slam_calibration {

class AuditTfNode {
 public:
  AuditTfNode() : private_nh_("~") {
    private_nh_.param("service_name", service_name_, std::string("/calib/audit_tf"));
    private_nh_.param("default_extrinsics_file", default_extrinsics_file_,
                      std::string(""));
    service_ = nh_.advertiseService(service_name_, &AuditTfNode::callback, this);
  }

 private:
  bool callback(AuditTf::Request& request, AuditTf::Response& response) {
    const std::string path =
        request.extrinsics_file.empty() ? default_extrinsics_file_ : request.extrinsics_file;
    if (path.empty()) {
      response.ok = false;
      response.message = "extrinsics_file is empty";
      response.issue_count = 1;
      response.report = "ok=false\nissue_count=1\nissue[0]=extrinsics_file is empty\n";
      return true;
    }
    const AuditReport report = auditExtrinsicsFile(path);
    response.ok = report.ok;
    response.message = report.ok ? "extrinsics audit passed" : "extrinsics audit failed";
    response.issue_count = static_cast<uint32_t>(report.issues.size());
    response.report = report.text;
    return true;
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::ServiceServer service_;
  std::string service_name_;
  std::string default_extrinsics_file_;
};

}  // namespace mine_slam_calibration

int main(int argc, char** argv) {
  ros::init(argc, argv, "audit_tf");
  mine_slam_calibration::AuditTfNode node;
  ros::spin();
  return 0;
}
