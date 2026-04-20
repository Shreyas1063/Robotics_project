import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import cv2
import numpy as np


class CAMShiftKalmanTracker(Node):

    def __init__(self):
        super().__init__('camshift_kalman_tracker')
        self.bridge = CvBridge()

        # ── Subscribe to the Gazebo camera topic ────────────────────────────
        # This topic name comes from the xacro remapping we set
        self.sub = self.create_subscription(
        Image,
        '/eye_in_hand_camera/image_raw',
        self.image_callback,
        10
)

        # ── Publish the tracked ball centroid ───────────────────────────────
        self.pub = self.create_publisher(
            PointStamped,
            '/ball_centroid',
            10
        )

        # ── Kalman filter: 4 states (x, y, vx, vy), 2 measurements (x, y) ──
        self.kf = cv2.KalmanFilter(4, 2)
        # Measurement matrix: we observe x and y directly
        self.kf.measurementMatrix = np.array(
            [[1, 0, 0, 0],
             [0, 1, 0, 0]], np.float32)
        # Transition matrix: constant velocity model
        self.kf.transitionMatrix = np.array(
            [[1, 0, 1, 0],   # x  = x  + vx
             [0, 1, 0, 1],   # y  = y  + vy
             [0, 0, 1, 0],   # vx = vx
             [0, 0, 0, 1]],  # vy = vy
            np.float32)
        self.kf.processNoiseCov    = np.eye(4, dtype=np.float32) * 0.01
        self.kf.measurementNoiseCov = np.eye(2, dtype=np.float32) * 0.1
        self.kf.errorCovPost       = np.eye(4, dtype=np.float32)

        # ── CAMShift state ───────────────────────────────────────────────────
        self.track_window = None
        self.roi_hist     = None
        self.initialized  = False

        # Termination criteria for CAMShift iterations
        self.term_crit = (
            cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT,
            10,   # max iterations
            1     # epsilon
        )

        # Count frames to log progress
        self.frame_count = 0
        self.get_logger().info('CAMShift tracker started. Waiting for camera...')

    def init_tracker(self, frame):
        """
        Called on the first few frames.
        Finds the blue ball by HSV colour thresholding,
        then initialises the CAMShift histogram.
        """
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Blue in HSV: hue ~100-130, high saturation, medium-high value
        # You may need to tune these if the blue looks different in your Gazebo
        lower_blue = np.array([100, 100,  50])
        upper_blue = np.array([130, 255, 255])
        mask = cv2.inRange(hsv, lower_blue, upper_blue)

        # Find the largest blue blob
        M = cv2.moments(mask)
        if M['m00'] < 300:   # too small — ball not visible yet
            return

        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
        r  = 35   # half-size of the initial tracking window in pixels

        x = max(0, cx - r)
        y = max(0, cy - r)
        w = min(frame.shape[1] - x, 2 * r)
        h = min(frame.shape[0] - y, 2 * r)
        self.track_window = (x, y, w, h)

        # Build the hue histogram of the ball ROI
        roi      = hsv[y:y+h, x:x+w]
        roi_mask = mask[y:y+h, x:x+w]
        self.roi_hist = cv2.calcHist([roi], [0], roi_mask, [180], [0, 180])
        cv2.normalize(self.roi_hist, self.roi_hist, 0, 255, cv2.NORM_MINMAX)

        # Initialise Kalman state at ball centre
        self.kf.statePre  = np.array([[cx], [cy], [0], [0]], dtype=np.float32)
        self.kf.statePost = np.array([[cx], [cy], [0], [0]], dtype=np.float32)

        self.initialized = True
        self.get_logger().info(
            f'Ball detected at ({cx}, {cy}). Tracker initialised!')

    def image_callback(self, msg):
        """Called at 30 Hz when a new camera frame arrives."""
        self.frame_count += 1

        # Convert ROS Image message → OpenCV BGR image
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # ── Try to initialise if not done yet ───────────────────────────────
        if not self.initialized:
            self.init_tracker(frame)
            if self.frame_count % 30 == 0:
                self.get_logger().warn(
                    'Waiting to detect blue ball... '
                    'Make sure ball is visible in camera view.')
            return

        # ── CAMShift tracking ────────────────────────────────────────────────
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        dst = cv2.calcBackProject([hsv], [0], self.roi_hist, [0, 180], 1)

        # CamShift returns the rotated bounding box + updated track_window
        ret, self.track_window = cv2.CamShift(dst, self.track_window, self.term_crit)
        x, y, w, h = self.track_window
        cx_meas = float(x + w // 2)
        cy_meas = float(y + h // 2)

        # Check if tracking window has collapsed (lost track)
        if w < 5 or h < 5:
            self.get_logger().warn('Tracking window collapsed — occlusion?')
            # Fall through to use Kalman prediction only

        # ── Kalman filter: correct with measurement, then predict ────────────
        measured = np.array([[cx_meas], [cy_meas]], dtype=np.float32)
        if w >= 5 and h >= 5:
            self.kf.correct(measured)   # update with real measurement
        predicted = self.kf.predict()   # get best estimate for NEXT frame

        px = int(predicted[0])
        py = int(predicted[1])

        # ── Publish centroid ─────────────────────────────────────────────────
        pt = PointStamped()
        pt.header         = msg.header   # keep original timestamp
        pt.point.x        = float(px)    # pixel column (u)
        pt.point.y        = float(py)    # pixel row (v)
        pt.point.z        = 1.0          # depth placeholder (metres)
        # Note: replace 1.0 with actual depth from depth camera if available
        self.pub.publish(pt)


def main(args=None):
    rclpy.init(args=args)
    node = CAMShiftKalmanTracker()
    rclpy.spin(node)
    rclpy.shutdown()