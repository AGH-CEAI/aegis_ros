#include <gtest/gtest.h>

#include <cmath>

#include "aegis_grpc/admittance_reference.hpp"

using aegis_grpc::AdmittanceReference;
using aegis_grpc::AdmittanceReferenceLimits;

namespace {

Eigen::Isometry3d Pose(double x, double y, double z) {
  Eigen::Isometry3d p = Eigen::Isometry3d::Identity();
  p.translation() = Eigen::Vector3d(x, y, z);
  return p;
}

AdmittanceReferenceLimits Limits() {
  AdmittanceReferenceLimits l;
  l.max_step_lin = 0.01;
  l.max_step_rot = 0.05;
  l.max_offset_lin = 0.02;
  l.max_offset_rot = 0.1;
  return l;
}

}  // namespace

TEST(AdmittanceReference, StepWithinLimitsIsApplied) {
  AdmittanceReference ref(Limits());
  ref.Reset(Pose(0.5, 0.0, 0.3));
  const bool clipped = ref.Step({0.005, 0.0, 0.0}, Eigen::Vector3d::Zero(), Pose(0.5, 0.0, 0.3));
  EXPECT_FALSE(clipped);
  EXPECT_NEAR(ref.Candidate().translation().x(), 0.505, 1e-12);
  // Not committed yet
  EXPECT_NEAR(ref.Reference().translation().x(), 0.5, 1e-12);
  ref.Commit();
  EXPECT_NEAR(ref.Reference().translation().x(), 0.505, 1e-12);
}

TEST(AdmittanceReference, StepIsRateLimited) {
  AdmittanceReference ref(Limits());
  ref.Reset(Pose(0.0, 0.0, 0.0));
  EXPECT_TRUE(ref.Step({0.1, 0.0, 0.0}, {0.0, 0.0, 1.0}, Pose(0.0, 0.0, 0.0)));
  EXPECT_NEAR(ref.Candidate().translation().norm(), 0.01, 1e-12);
  EXPECT_NEAR(AdmittanceReference::RotationVector(ref.Candidate().linear()).norm(), 0.05, 1e-9);
}

TEST(AdmittanceReference, IntegratesFromReferenceNotFromMeasurement) {
  // The measured pose lags behind (e.g. contact deflection); the reference keeps advancing
  // until the bounded offset is reached instead of being re-based on the measurement.
  AdmittanceReference ref(Limits());
  ref.Reset(Pose(0.0, 0.0, 0.0));
  const auto measured = Pose(0.0, 0.0, 0.0);
  ref.Step({0.01, 0.0, 0.0}, Eigen::Vector3d::Zero(), measured);
  ref.Commit();
  EXPECT_NEAR(ref.Reference().translation().x(), 0.01, 1e-12);
  ref.Step({0.01, 0.0, 0.0}, Eigen::Vector3d::Zero(), measured);
  ref.Commit();
  EXPECT_NEAR(ref.Reference().translation().x(), 0.02, 1e-12);
}

TEST(AdmittanceReference, OffsetFromMeasurementIsBounded) {
  AdmittanceReference ref(Limits());
  ref.Reset(Pose(0.0, 0.0, 0.0));
  const auto measured = Pose(0.0, 0.0, 0.0);  // robot blocked by a rigid obstacle
  for (int i = 0; i < 10; ++i) {
    ref.Step({0.01, 0.0, 0.0}, Eigen::Vector3d::Zero(), measured);
    ref.Commit();
  }
  EXPECT_NEAR(ref.Reference().translation().x(), 0.02, 1e-12);  // max_offset_lin

  // Same for orientation
  ref.Reset(Pose(0.0, 0.0, 0.0));
  for (int i = 0; i < 10; ++i) {
    ref.Step(Eigen::Vector3d::Zero(), {0.0, 0.0, 0.05}, measured);
    ref.Commit();
  }
  EXPECT_NEAR(AdmittanceReference::RotationVector(ref.Reference().linear()).z(), 0.1, 1e-9);
}

TEST(AdmittanceReference, WorkspaceBoxClampsPosition) {
  auto limits = Limits();
  limits.max_offset_lin = 1.0;
  limits.workspace_enabled = true;
  limits.workspace_min = Eigen::Vector3d(-0.1, -0.1, 0.05);
  limits.workspace_max = Eigen::Vector3d(0.1, 0.1, 0.5);
  AdmittanceReference ref(limits);
  ref.Reset(Pose(0.0, 0.0, 0.055));
  EXPECT_TRUE(ref.Step({0.0, 0.0, -0.01}, Eigen::Vector3d::Zero(), Pose(0.0, 0.0, 0.055)));
  EXPECT_NEAR(ref.Candidate().translation().z(), 0.05, 1e-12);
}

TEST(AdmittanceReference, StepTowardsTargetUsesSameLimits) {
  AdmittanceReference ref(Limits());
  ref.Reset(Pose(0.0, 0.0, 0.0));
  Eigen::Isometry3d target = Pose(0.0, 0.003, 0.0);
  target.linear() = Eigen::AngleAxisd(0.02, Eigen::Vector3d::UnitX()).toRotationMatrix();
  EXPECT_FALSE(ref.StepTowards(target, Pose(0.0, 0.0, 0.0)));
  EXPECT_TRUE(ref.Candidate().isApprox(target, 1e-9));

  EXPECT_TRUE(ref.StepTowards(Pose(1.0, 0.0, 0.0), Pose(0.0, 0.0, 0.0)));
  EXPECT_NEAR(ref.Candidate().translation().norm(), 0.01, 1e-12);
}

TEST(AdmittanceReference, StepChangeIsAccelerationLimited) {
  AdmittanceReferenceLimits l = Limits();
  l.max_step_change_lin = 0.002;
  AdmittanceReference ref(l);
  ref.Reset(Pose(0.0, 0.0, 0.0));
  // From rest the step grows by at most 2 mm per call: 2, 4, 6, 8, 10 (max_step_lin) mm.
  double x = 0.0;
  for (const double expected : {0.002, 0.004, 0.006, 0.008, 0.01, 0.01}) {
    ref.Step({0.01, 0.0, 0.0}, Eigen::Vector3d::Zero(), Pose(x, 0.0, 0.0));
    ref.Commit();
    EXPECT_NEAR(ref.Reference().translation().x() - x, expected, 1e-12);
    x = ref.Reference().translation().x();
  }
  // Stopping decelerates the same way.
  EXPECT_TRUE(ref.Step(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Pose(x, 0.0, 0.0)));
  ref.Commit();
  EXPECT_NEAR(ref.Reference().translation().x() - x, 0.008, 1e-12);
}

TEST(AdmittanceReference, StepWithinAccelerationLimitIsNotClipped) {
  AdmittanceReferenceLimits l = Limits();
  l.max_step_change_lin = 0.002;
  AdmittanceReference ref(l);
  ref.Reset(Pose(0.0, 0.0, 0.0));
  EXPECT_FALSE(ref.Step({0.0015, 0.0, 0.0}, Eigen::Vector3d::Zero(), Pose(0.0, 0.0, 0.0)));
}

TEST(AdmittanceReference, HoldAndResetRestartFromRest) {
  AdmittanceReferenceLimits l = Limits();
  l.max_step_change_lin = 0.002;
  AdmittanceReference ref(l);
  ref.Reset(Pose(0.0, 0.0, 0.0));
  for (int i = 0; i < 5; ++i) {
    ref.Step({0.01, 0.0, 0.0}, Eigen::Vector3d::Zero(), ref.Reference());
    ref.Commit();
  }
  ref.Hold();
  const double x = ref.Reference().translation().x();
  ref.Step({0.01, 0.0, 0.0}, Eigen::Vector3d::Zero(), ref.Reference());
  EXPECT_NEAR(ref.Candidate().translation().x() - x, 0.002, 1e-12);
  ref.Reset(Pose(0.0, 0.0, 0.0));
  ref.Step({0.01, 0.0, 0.0}, Eigen::Vector3d::Zero(), Pose(0.0, 0.0, 0.0));
  EXPECT_NEAR(ref.Candidate().translation().x(), 0.002, 1e-12);
}
