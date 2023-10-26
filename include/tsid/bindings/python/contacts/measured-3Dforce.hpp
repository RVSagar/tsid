//
// Copyright (c) 2022 CNRS INRIA LORIA
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

#ifndef __tsid_python_measured_3d_force_hpp__
#define __tsid_python_measured_3d_force_hpp__

#include "tsid/bindings/python/fwd.hpp"

#include "tsid/contacts/measured-3Dforce.hpp"
#include "tsid/robots/robot-wrapper.hpp"

namespace tsid {
namespace python {
namespace bp = boost::python;

template <typename Measured3Dforce>
struct Measured3DforcePythonVisitor
    : public boost::python::def_visitor<Measured3DforcePythonVisitor<Measured3Dforce> > {
  template <class PyClass>

  void visit(PyClass &cl) const {
  cl.def(bp::init<std::string, robots::RobotWrapper &, std::string>(
              (bp::arg("name"), bp::arg("robot"), bp::arg("frameName")),
              "Default Constructor"))
      .add_property("name", &Measured3DforcePythonVisitor::name, "return name")

      .add_property("computeJointTorques",
              bp::make_function(
                  &Measured3DforcePythonVisitor::computeJointTorques,
                  bp::return_value_policy<bp::copy_const_reference>()))

      .add_property("getMeasuredContactForce",
              bp::make_function(
                  &Measured3DforcePythonVisitor::getMeasuredContactForce,
                  bp::return_value_policy<bp::copy_const_reference>()))

      .def("setMeasuredContactForce", &Measured3DforcePythonVisitor::setMeasuredContactForce,
            bp::args("fext"))
      .def("useLocalFrame", &Measured3DforcePythonVisitor::useLocalFrame,
            bp::args("local_frame"));
  }

  static std::string name(Measured3Dforce &self) {
    std::string name = self.name();
    return name;
  }

  static const math::Vector &computeJointTorques(Measured3Dforce &self, pinocchio::Data &data) {
    return self.computeJointTorques(data);
  }

  static const math::Vector3 &getMeasuredContactForce(Measured3Dforce &self) {
    return self.getMeasuredContactForce();
  }

  static void setMeasuredContactForce(Measured3Dforce &self, const math::Vector3 &fext) {
    return self.setMeasuredContactForce(fext);
  }

  static void useLocalFrame(Measured3Dforce &self, bool local_frame) {
    return self.useLocalFrame(local_frame);
  }

  static void expose(const std::string &class_name) {
    std::string doc = "Measured3Dforce info.";
    bp::class_<Measured3Dforce>(class_name.c_str(), doc.c_str(), bp::no_init)
        .def(Measured3DforcePythonVisitor<Measured3Dforce>());
  }


};
}  // namespace python
}  // namespace tsid

#endif  // ifndef __tsid_python_measured_3d_force_hpp__