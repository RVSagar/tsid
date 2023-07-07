//
// Copyright (c) 2017 CNRS
//
// This file is part of tsid
// tsid is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
// tsid is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// General Lesser Public License for more details. You should have
// received a copy of the GNU Lesser General Public License along with
// tsid If not, see
// <http://www.gnu.org/licenses/>.
//

#include "tsid/tasks/task-angular-momentum-equality.hpp"
#include "tsid/robots/robot-wrapper.hpp"
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/centroidal.hpp>

namespace tsid {
namespace tasks {
using namespace math;
using namespace trajectories;
using namespace pinocchio;

TaskAMEquality::TaskAMEquality(const std::string& name, RobotWrapper& robot)
    : TaskMotion(name, robot), m_constraint(name, 6, robot.nv()) {
  m_Kp.setZero(6);
  m_Kd.setZero(6);
  m_L_error.setZero(6);
  m_dL_error.setZero(6);
  m_L.setZero(6);
  m_dL.setZero(6);
  m_dL_des.setZero(6);
  m_drift.setZero(6);
  m_ref.resize(6);
  m_mask.resize(6);
  m_mask.fill(1.);
  setMask(m_mask); 
}

void TaskAMEquality::setMask(math::ConstRefVector mask)
{
  assert(mask.size() == 6);
  TaskMotion::setMask(mask);
  int n = dim();
  m_constraint.resize(n, m_robot.nv());
  m_L_error_masked.resize(n);
  m_L_masked.resize(n);
  m_dL_des_masked.resize(n);
  m_drift_masked.resize(n);
}

int TaskAMEquality::dim() const { return int(m_mask.sum()); }

const Vector& TaskAMEquality::Kp() { return m_Kp; }

const Vector& TaskAMEquality::Kd() { return m_Kd; }

void TaskAMEquality::Kp(ConstRefVector Kp) {
  PINOCCHIO_CHECK_INPUT_ARGUMENT(Kp.size() == 6,
                                 "The size of the Kp vector needs to equal 6");
  m_Kp = Kp;
}

void TaskAMEquality::Kd(ConstRefVector Kd) {
  PINOCCHIO_CHECK_INPUT_ARGUMENT(Kd.size() == 6,
                                 "The size of the Kd vector needs to equal 6");
  m_Kd = Kd;
}

void TaskAMEquality::setReference(const TrajectorySample& ref) { m_ref = ref; }

const TrajectorySample& TaskAMEquality::getReference() const { return m_ref; }

const Vector & TaskAMEquality::getDesiredMomentumDerivative() const
{
  return m_dL_des_masked;
}

Vector TaskAMEquality::getdMomentum(ConstRefVector dv) const
{
  return m_constraint.matrix()*dv - m_drift_masked;
}

const Vector & TaskAMEquality::momentum_error() const
{
  return m_L_error_masked;
}

const Vector& TaskAMEquality::momentum() const { return m_L; }

const Vector & TaskAMEquality::momentum_masked() const
{
  return m_L_masked;
}

const Vector& TaskAMEquality::momentum_ref() const { return m_ref.getValue(); }

const Vector& TaskAMEquality::dmomentum_ref() const {
  return m_ref.getDerivative();
}

const ConstraintBase& TaskAMEquality::getConstraint() const {
  return m_constraint;
}

const ConstraintBase& TaskAMEquality::compute(const double, ConstRefVector,
                                              ConstRefVector v, Data& data) {
  // Compute errors
  // Get momentum jacobian
  const Matrix6x& J_am = m_robot.momentumJacobian(data);
  m_L = J_am * v;
  m_L_error = m_L - m_ref.getValue();

  m_dL_des = -m_Kp.cwiseProduct(m_L_error) + m_ref.getDerivative();

#ifndef NDEBUG
//      std::cout<<m_name<<" errors: "<<m_L_error.norm()<<" "
//        <<m_dL_error.norm()<<std::endl;
#endif

  m_drift.head(3) = pinocchio::computeCentroidalMomentumTimeVariation(m_robot.model(), const_cast<Data&>(data)).linear();
  m_drift.tail(3) = m_robot.angularMomentumTimeVariation(data);

  int idx = 0;
    for (int i = 0; i < 6; i++) {
      if (m_mask(i) != 1) continue;
    
      m_constraint.matrix().row(idx) = J_am.row(i);
      m_constraint.vector().row(idx) = (m_dL_des - m_drift).row(i);      

      m_L_error_masked(idx) = m_L_error(i);
      m_L_masked(idx) = m_L(i);
      m_dL_des_masked(idx) = m_dL_des(i);
      m_drift_masked(idx) = m_drift(i);

      idx += 1;
    }

  // m_constraint.setMatrix(J_am);
  // m_constraint.setVector(m_dL_des - m_drift);

  return m_constraint;
}

}  // namespace tasks
}  // namespace tsid
