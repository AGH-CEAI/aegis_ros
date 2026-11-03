#include "aegis_grpc/admittance_reference.hpp"

#include <algorithm>
#include <cmath>

namespace aegis_grpc {

AdmittanceReference::AdmittanceReference(const AdmittanceReferenceLimits& limits) : limits_(limits) {}

void AdmittanceReference::Reset(const Eigen::Isometry3d& pose) {
  reference_ = pose;
  candidate_ = pose;
  last_step_.setZero();
}

Eigen::Vector3d AdmittanceReference::RotationVector(const Eigen::Matrix3d& rotation) {
  const Eigen::AngleAxisd aa(rotation);
  return aa.angle() * aa.axis();
}

bool AdmittanceReference::ClipNorm(Eigen::Vector3d& v, double max_norm) {
  const double n = v.norm();
  if (max_norm >= 0.0 && n > max_norm && n > 0.0) {
    v *= max_norm / n;
    return true;
  }
  return false;
}

bool AdmittanceReference::Step(const Eigen::Vector3d& translation,
                               const Eigen::Vector3d& rotation,
                               const Eigen::Isometry3d& measured) {
  bool clipped = false;

  // 1) Per-call rate limit.
  Eigen::Vector3d dp = translation;
  Eigen::Vector3d dr = rotation;
  clipped |= ClipNorm(dp, limits_.max_step_lin);
  clipped |= ClipNorm(dr, limits_.max_step_rot);

  // 1b) Acceleration limit: the step may differ from the last committed one by at most
  // max_step_change_lin. Both lie within max_step_lin, so the result does too.
  if (limits_.max_step_change_lin >= 0.0) {
    Eigen::Vector3d change = dp - last_step_;
    if (ClipNorm(change, limits_.max_step_change_lin)) {
      clipped = true;
      dp = last_step_ + change;
    }
  }

  // 2) Integrate from the previous REFERENCE (not from the measured pose).
  Eigen::Vector3d p = reference_.translation() + dp;
  Eigen::Matrix3d r = reference_.linear();
  const double dr_angle = dr.norm();
  if (dr_angle > 0.0) {
    r = Eigen::AngleAxisd(dr_angle, dr / dr_angle).toRotationMatrix() * r;
  }

  // 3) Workspace box.
  if (limits_.workspace_enabled) {
    const Eigen::Vector3d p_box = p.cwiseMax(limits_.workspace_min).cwiseMin(limits_.workspace_max);
    clipped |= !p_box.isApprox(p);
    p = p_box;
  }

  // 4) Bounded offset from the measured pose (bounds the contact force).
  Eigen::Vector3d offset = p - measured.translation();
  if (ClipNorm(offset, limits_.max_offset_lin)) {
    clipped = true;
    p = measured.translation() + offset;
  }
  Eigen::Vector3d rot_offset = RotationVector(r * measured.linear().transpose());
  if (ClipNorm(rot_offset, limits_.max_offset_rot)) {
    clipped = true;
    const double angle = rot_offset.norm();
    const Eigen::Matrix3d r_offset =
        angle > 0.0 ? Eigen::AngleAxisd(angle, rot_offset / angle).toRotationMatrix() : Eigen::Matrix3d::Identity();
    r = r_offset * measured.linear();
  }

  candidate_.setIdentity();
  candidate_.linear() = r;
  candidate_.translation() = p;
  return clipped;
}

bool AdmittanceReference::StepTowards(const Eigen::Isometry3d& target, const Eigen::Isometry3d& measured) {
  const Eigen::Vector3d dp = target.translation() - reference_.translation();
  const Eigen::Vector3d dr = RotationVector(target.linear() * reference_.linear().transpose());
  return Step(dp, dr, measured);
}

}  // namespace aegis_grpc
