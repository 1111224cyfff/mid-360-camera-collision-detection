#!/usr/bin/env python3

import math
import sys

import rospy
import tf2_ros
from sensor_msgs.msg import CompressedImage, PointCloud2


class AlignmentAudit:
    def __init__(self):
        self.image_topic = rospy.get_param("~image_topic", "/hikrobot_camera/rgb/leveled/compressed")
        self.pointcloud_topic = rospy.get_param("~pointcloud_topic", "/merged_pointcloud_leveled")
        self.source_frame = rospy.get_param("~source_frame", "hikrobot_camera")
        self.parent_frame = rospy.get_param("~parent_frame", "body_leveled")
        self.output_frame = rospy.get_param("~output_frame", "hikrobot_camera_leveled")
        self.expected_reference_frame = rospy.get_param("~expected_reference_frame", self.parent_frame)
        self.message_timeout_sec = float(rospy.get_param("~message_timeout_sec", 5.0))
        self.tf_timeout_sec = float(rospy.get_param("~tf_timeout_sec", 5.0))
        self.translation_warn_threshold_m = float(rospy.get_param("~translation_warn_threshold_m", 5.0))
        self.rotation_orthogonality_tol = float(rospy.get_param("~rotation_orthogonality_tol", 1e-3))
        self.rotation_det_tol = float(rospy.get_param("~rotation_det_tol", 1e-3))

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(max(10.0, self.tf_timeout_sec + 1.0)))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def run(self):
        rospy.loginfo("camera_lidar_alignment_audit: waiting for image on %s", self.image_topic)
        image_msg = self._wait_for_message(self.image_topic, CompressedImage, self.message_timeout_sec)
        rospy.loginfo("camera_lidar_alignment_audit: waiting for point cloud on %s", self.pointcloud_topic)
        cloud_msg = self._wait_for_message(self.pointcloud_topic, PointCloud2, self.message_timeout_sec)

        overall_ok = True
        findings = []

        image_ok, image_findings = self._check_image_frame(image_msg)
        overall_ok &= image_ok
        findings.extend(image_findings)

        cloud_ok, cloud_findings = self._check_pointcloud_frame(cloud_msg)
        overall_ok &= cloud_ok
        findings.extend(cloud_findings)

        tf_ok, tf_findings = self._check_tf_chain(cloud_msg.header.frame_id, image_msg.header.frame_id)
        overall_ok &= tf_ok
        findings.extend(tf_findings)

        for level, message in findings:
            if level == "error":
                rospy.logerr(message)
            elif level == "warn":
                rospy.logwarn(message)
            else:
                rospy.loginfo(message)

        if overall_ok:
            rospy.loginfo("camera_lidar_alignment_audit: PASS")
            return 0

        rospy.logerr("camera_lidar_alignment_audit: FAIL")
        return 1

    def _wait_for_message(self, topic, message_type, timeout_sec):
        try:
            return rospy.wait_for_message(topic, message_type, timeout=timeout_sec)
        except rospy.ROSException as exc:
            raise RuntimeError("timed out waiting for %s on %s: %s" % (message_type.__name__, topic, exc))

    def _check_image_frame(self, image_msg):
        findings = []
        if image_msg.header.frame_id == self.output_frame:
            findings.append(("info", "Image frame matches output_frame: %s" % self.output_frame))
            return True, findings

        findings.append((
            "error",
            "Image frame mismatch: got %s but expected %s. camera_frame_transform may be disabled or publishing the wrong frame."
            % (image_msg.header.frame_id, self.output_frame),
        ))
        if image_msg.header.frame_id == self.source_frame:
            findings.append((
                "warn",
                "Image frame is still source_frame %s, so the republished leveled image is not the one being checked."
                % self.source_frame,
            ))
        return False, findings

    def _check_pointcloud_frame(self, cloud_msg):
        findings = []
        cloud_frame = cloud_msg.header.frame_id
        if cloud_frame == self.expected_reference_frame:
            findings.append(("info", "Point cloud frame matches expected reference frame: %s" % self.expected_reference_frame))
            return True, findings

        findings.append((
            "warn",
            "Point cloud frame is %s, not %s. Alignment can still be valid if TF connects them, but you are comparing across an extra frame hop."
            % (cloud_frame, self.expected_reference_frame),
        ))
        return True, findings

    def _check_tf_chain(self, cloud_frame, image_frame):
        findings = []
        overall_ok = True

        parent_tf = self._lookup_transform(self.parent_frame, self.output_frame)
        if parent_tf is None:
            findings.append((
                "error",
                "Missing TF from %s to %s. The camera extrinsic TF is not available at runtime."
                % (self.parent_frame, self.output_frame),
            ))
            overall_ok = False
        else:
            transform_ok, transform_findings = self._check_transform_sanity(parent_tf, self.parent_frame, self.output_frame)
            overall_ok &= transform_ok
            findings.extend(transform_findings)

        cloud_to_image_tf = self._lookup_transform(cloud_frame, image_frame)
        if cloud_to_image_tf is None:
            findings.append((
                "error",
                "Missing TF from point cloud frame %s to image frame %s. The two data streams are not comparable in one TF tree."
                % (cloud_frame, image_frame),
            ))
            overall_ok = False
        else:
            findings.append((
                "info",
                "TF is available from point cloud frame %s to image frame %s."
                % (cloud_frame, image_frame),
            ))

        if cloud_frame != self.parent_frame:
            cloud_to_parent_tf = self._lookup_transform(cloud_frame, self.parent_frame)
            if cloud_to_parent_tf is None:
                findings.append((
                    "error",
                    "Point cloud frame %s cannot be transformed into parent frame %s, so the camera parent frame is not connected to the cloud reference."
                    % (cloud_frame, self.parent_frame),
                ))
                overall_ok = False
            else:
                findings.append((
                    "info",
                    "Point cloud frame %s is connected to camera parent frame %s through TF."
                    % (cloud_frame, self.parent_frame),
                ))

        return overall_ok, findings

    def _lookup_transform(self, target_frame, source_frame):
        deadline = rospy.Time.now() + rospy.Duration(self.tf_timeout_sec)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            try:
                return self.tf_buffer.lookup_transform(target_frame, source_frame, rospy.Time(0), rospy.Duration(0.2))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.sleep(0.1)
        return None

    def _check_transform_sanity(self, tf_msg, parent_frame, child_frame):
        findings = []
        translation = tf_msg.transform.translation
        rotation = tf_msg.transform.rotation
        translation_norm = math.sqrt(
            translation.x * translation.x + translation.y * translation.y + translation.z * translation.z
        )
        findings.append((
            "info",
            "TF %s <- %s translation norm: %.4f m" % (parent_frame, child_frame, translation_norm),
        ))
        if translation_norm > self.translation_warn_threshold_m:
            findings.append((
                "warn",
                "TF %s <- %s translation norm %.4f m is unusually large for a camera-lidar rig."
                % (parent_frame, child_frame, translation_norm),
            ))

        rotation_matrix = self._rotation_matrix_from_quaternion(rotation.x, rotation.y, rotation.z, rotation.w)
        orthogonality_error = self._orthogonality_error(rotation_matrix)
        determinant = self._determinant3(rotation_matrix)
        findings.append((
            "info",
            "TF %s <- %s rotation orthogonality error: %.6g, determinant: %.6f"
            % (parent_frame, child_frame, orthogonality_error, determinant),
        ))

        ok = True
        if orthogonality_error > self.rotation_orthogonality_tol:
            findings.append((
                "error",
                "Rotation matrix for TF %s <- %s is not orthonormal enough; error %.6g exceeds %.6g."
                % (parent_frame, child_frame, orthogonality_error, self.rotation_orthogonality_tol),
            ))
            ok = False
        if abs(determinant - 1.0) > self.rotation_det_tol:
            findings.append((
                "error",
                "Rotation matrix for TF %s <- %s has determinant %.6f, expected near 1.0."
                % (parent_frame, child_frame, determinant),
            ))
            ok = False
        return ok, findings

    def _rotation_matrix_from_quaternion(self, x, y, z, w):
        xx = x * x
        yy = y * y
        zz = z * z
        xy = x * y
        xz = x * z
        yz = y * z
        wx = w * x
        wy = w * y
        wz = w * z
        return [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ]

    def _orthogonality_error(self, matrix):
        transpose = [
            [matrix[0][0], matrix[1][0], matrix[2][0]],
            [matrix[0][1], matrix[1][1], matrix[2][1]],
            [matrix[0][2], matrix[1][2], matrix[2][2]],
        ]
        product = self._matmul3(matrix, transpose)
        error = 0.0
        for row in range(3):
            for col in range(3):
                target = 1.0 if row == col else 0.0
                error = max(error, abs(product[row][col] - target))
        return error

    def _matmul3(self, left, right):
        result = []
        for row in range(3):
            result_row = []
            for col in range(3):
                result_row.append(
                    left[row][0] * right[0][col] + left[row][1] * right[1][col] + left[row][2] * right[2][col]
                )
            result.append(result_row)
        return result

    def _determinant3(self, matrix):
        return (
            matrix[0][0] * (matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1])
            - matrix[0][1] * (matrix[1][0] * matrix[2][2] - matrix[1][2] * matrix[2][0])
            + matrix[0][2] * (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0])
        )


def main():
    rospy.init_node("camera_lidar_alignment_audit")
    try:
        return AlignmentAudit().run()
    except RuntimeError as exc:
        rospy.logerr("camera_lidar_alignment_audit: %s", exc)
        return 1


if __name__ == "__main__":
    sys.exit(main())