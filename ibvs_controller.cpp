#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

// ViSP visual servoing library
#include <visp3/vs/vpServo.h>
#include <visp3/visual_features/vpFeaturePoint.h>
#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpColVector.h>

class IBVSController : public rclcpp::Node
{
public:
  IBVSController() : Node("ibvs_controller")
  {
    // ── Camera intrinsics (pixels) ──────────────────────────────────────────
    // These match the xacro: 640x480, hfov=1.0472 rad
    // px = py = (width/2) / tan(hfov/2) ≈ 554
    double px = 554.0, py = 554.0;
    double u0 = 320.0, v0 = 240.0;   // principal point = image centre
    cam_.initPersProjWithoutDistortion(px, py, u0, v0);

    // ── ViSP servo task setup ───────────────────────────────────────────────
    // EYEINHAND_CAMERA: camera is mounted on the end-effector
    task_.setServo(vpServo::EYEINHAND_CAMERA);
    // Use the CURRENT interaction matrix (recomputed each step)
    task_.setInteractionMatrixType(vpServo::CURRENT);
    // Lambda (gain): higher = faster but may oscillate. Start with 0.3
    task_.setLambda(0.3);

    // Desired feature = ball centroid at image centre (normalised coordinates)
    // When the ball is at (u0, v0) in pixels, normalised x=0, y=0
    s_desired_.set_xyZ(0.0, 0.0, 1.0);  // Z=1.0 is a depth estimate in metres

    // Add ONE point feature: current vs desired
    task_.addFeature(s_current_, s_desired_);

    // ── ROS2 subscribers and publishers ────────────────────────────────────
    // Receive centroid from the CAMShift tracker
    centroid_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/ball_centroid", 10,
      std::bind(&IBVSController::centroidCallback, this, std::placeholders::_1));

    // Send joint velocity commands to ros2_control
    // Topic name: check with "ros2 topic list" after launching Gazebo
    velocity_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/forward_velocity_controller/commands", 10);

    RCLCPP_INFO(this->get_logger(),
      "IBVS controller ready. Waiting for /ball_centroid ...");
    RCLCPP_INFO(this->get_logger(),
      "Desired image position: centre (%.0f, %.0f)", u0, v0);
  }

  ~IBVSController()
  {
    task_.kill();  // clean up ViSP task
  }

private:
  void centroidCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    // ── Read pixel coordinates from tracker ────────────────────────────────
    double u_pixels = msg->point.x;   // horizontal pixel position
    double v_pixels = msg->point.y;   // vertical pixel position
    double Z = msg->point.z;          // depth in metres (from tracker)

    // Use a safe fallback depth if tracker doesn't provide it
    if (Z < 0.05 || Z > 5.0) {
      Z = 1.0;   // 1 metre default — adjust based on your scene
    }

    // ── Convert pixel → normalised image plane coordinates ─────────────────
    // x_norm = (u - u0) / px,  y_norm = (v - v0) / py
    double x_norm = (u_pixels - cam_.get_u0()) / cam_.get_px();
    double y_norm = (v_pixels - cam_.get_v0()) / cam_.get_py();

    // ── Update current feature ─────────────────────────────────────────────
    s_current_.set_xyZ(x_norm, y_norm, Z);

    // ── ViSP computes: vc = -lambda * Ls_pinv * error ──────────────────────
    // This returns a 6-element vector: [vx, vy, vz, wx, wy, wz]
    // (camera linear + angular velocities)
    vpColVector v_cam = task_.computeControlLaw();

    // ── Publish as joint velocity command ──────────────────────────────────
    // NOTE: Ideally you multiply by the robot Jacobian inverse here.
    // For this first version, we map camera velocity directly to joints.
    // The forward_velocity_controller expects one value per joint (6 joints).
    std_msgs::msg::Float64MultiArray cmd;
    cmd.data.resize(6, 0.0);

    // Clamp velocities for safety (max 0.5 rad/s per joint)
    const double MAX_VEL = 0.5;
    for (int i = 0; i < 6 && i < static_cast<int>(v_cam.size()); ++i) {
      cmd.data[i] = std::clamp(v_cam[i], -MAX_VEL, MAX_VEL);
    }

    velocity_pub_->publish(cmd);

    // Log the error every 30 callbacks (~1 second at 30Hz)
    static int log_counter = 0;
    if (++log_counter >= 30) {
      log_counter = 0;
      double e_u = u_pixels - cam_.get_u0();
      double e_v = v_pixels - cam_.get_v0();
      RCLCPP_INFO(this->get_logger(),
        "Feature error: (%.1f, %.1f) pixels | Camera vel: [%.3f %.3f %.3f]",
        e_u, e_v, v_cam[0], v_cam[1], v_cam[2]);
    }
  }

  // ROS2 communication
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr centroid_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr velocity_pub_;

  // ViSP objects
  vpServo          task_;       // the servo task (computes control law)
  vpFeaturePoint   s_current_;  // current image feature (updated each frame)
  vpFeaturePoint   s_desired_;  // desired image feature (set at init, fixed)
  vpCameraParameters cam_;      // camera intrinsic parameters
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IBVSController>());
  rclcpp::shutdown();
  return 0;
}