#ifndef AEGIS_GRPC__ADMITTANCE_REFERENCE_HPP_
#define AEGIS_GRPC__ADMITTANCE_REFERENCE_HPP_

#include <Eigen/Geometry>

namespace aegis_grpc {

/**
 * Limits applied to the TCP reference of the admittance chain.
 */
struct AdmittanceReferenceLimits {
  double max_step_lin{0.01};    ///< Max translation per call [m].
  double max_step_rot{0.05};    ///< Max rotation per call [rad].
  double max_offset_lin{0.02};  ///< Max translation between reference and measured pose [m].
  double max_offset_rot{0.1};   ///< Max rotation between reference and measured pose [rad].
  /// Max change of the translation step between consecutive calls [m], i.e. max_accel * dt^2
  /// (negative = disabled). Bounds the commanded acceleration of the reference.
  double max_step_change_lin{-1.0};
  bool workspace_enabled{false};
  Eigen::Vector3d workspace_min{Eigen::Vector3d::Constant(-1e9)};  ///< Workspace box [m].
  Eigen::Vector3d workspace_max{Eigen::Vector3d::Constant(1e9)};
};

/**
 * Cartesian TCP reference generator for the admittance chain (no ROS dependencies).
 *
 * The reference is integrated from the previous reference, NOT from the measured pose, so the
 * admittance spring is not absorbed by the next command. The measured pose is only used to
 * bound the reference-to-measured offset, which bounds the steady-state contact force
 * to roughly stiffness * max_offset.
 */
class AdmittanceReference {
 public:
  explicit AdmittanceReference(const AdmittanceReferenceLimits& limits = AdmittanceReferenceLimits());

  /// Set the reference to the given pose (e.g. the measured TCP pose when enabling).
  void Reset(const Eigen::Isometry3d& pose);

  /**
   * Move the reference by a displacement expressed in the root frame.
   * @param translation [m]
   * @param rotation rotation vector [rad] (applied as R_ref = Exp(rotation) * R_ref)
   * @param measured measured TCP pose in the root frame
   * @return true if any limit was applied
   */
  bool Step(const Eigen::Vector3d& translation, const Eigen::Vector3d& rotation, const Eigen::Isometry3d& measured);

  /// Move the reference towards an absolute target with the same limits as Step().
  bool StepTowards(const Eigen::Isometry3d& target, const Eigen::Isometry3d& measured);

  /// Candidate reference computed by Step()/StepTowards(); call Commit() to accept it.
  const Eigen::Isometry3d& Candidate() const { return candidate_; }
  /// Accept the last candidate (e.g. after IK and collision checks succeeded).
  void Commit() {
    last_step_ = candidate_.translation() - reference_.translation();
    reference_ = candidate_;
  }
  /// The candidate was rejected: the reference stands still, so the next step starts from rest.
  void Hold() { last_step_.setZero(); }
  const Eigen::Isometry3d& Reference() const { return reference_; }
  const AdmittanceReferenceLimits& Limits() const { return limits_; }

  /// Rotation vector (axis * angle) of a rotation matrix, angle in [0, pi].
  static Eigen::Vector3d RotationVector(const Eigen::Matrix3d& rotation);
  /// Scale `v` down to `max_norm` if needed. Returns true if it was scaled.
  static bool ClipNorm(Eigen::Vector3d& v, double max_norm);

 private:
  AdmittanceReferenceLimits limits_;
  Eigen::Isometry3d reference_{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d candidate_{Eigen::Isometry3d::Identity()};
  Eigen::Vector3d last_step_{Eigen::Vector3d::Zero()};  ///< Committed translation of the last call.
};

}  // namespace aegis_grpc
#endif  // AEGIS_GRPC__ADMITTANCE_REFERENCE_HPP_
