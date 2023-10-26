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

#ifndef __tsid_python_measured_6d_wrench_hpp__
#define __tsid_python_measured_6d_wrench_hpp__

#include "tsid/bindings/python/fwd.hpp"

#include "tsid/contacts/measured-6Dwrench.hpp"
#include "tsid/robots/robot-wrapper.hpp"

namespace tsid {
namespace python {
namespace bp = boost::python;

template <typename Measured6Dwrench>
struct Measured6DwrenchPythonVisitor
    : public boost::python::def_visitor<Measured6DwrenchPythonVisitor<Measured6Dwrench> > {
  template <class PyClass>

  void visit(PyClass &cl) const {
  cl.def(bp::init<std::string, robots::RobotWrapper &, std::string>(
              (bp::arg("name"), bp::arg("robot"), bp::arg("frameName")),
              "Default Constructor"))
      .add_property("name", &Measured6DwrenchPythonVisitor::name, "return name")

      .add_property("computeJointTorques",
              bp::make_function(
                  &Measured6DwrenchPythonVisitor::computeJointTorques,
                  bp::return_value_policy<bp::copy_const_reference>()))

      .add_property("getMeasuredContactForce",
              bp::make_function(
                  &Measured6DwrenchPythonVisitor::getMeasuredContactForce,
                  bp::return_value_policy<bp::copy_const_reference>()))

      .def("setMeasuredContactForce", &Measured6DwrenchPythonVisitor::setMeasuredContactForce,
            bp::args("fext"))
      .def("useLocalFrame", &Measured6DwrenchPythonVisitor::useLocalFrame,
            bp::args("local_frame"));
  }

  static std::string name(Measured6Dwrench &self) {
    std::string name = self.name();
    return name;
  }

  static const math::Vector &computeJointTorques(Measured6Dwrench &self, pinocchio::Data &data) {
    return self.computeJointTorques(data);
  }

  static const math::Vector6 &getMeasuredContactForce(Measured6Dwrench &self) {
    return self.getMeasuredContactForce();
  }

  static void setMeasuredContactForce(Measured6Dwrench &self, const math::Vector6 &fext) {
    return self.setMeasuredContactForce(fext);
  }

  static void useLocalFrame(Measured6Dwrench &self, bool local_frame) {
    return self.useLocalFrame(local_frame);
  }

  static void expose(const std::string &class_name) {
    std::string doc = "Measured6Dwrench info.";
    bp::class_<Measured6Dwrench>(class_name.c_str(), doc.c_str(), bp::no_init)
        .def(Measured6DwrenchPythonVisitor<Measured6Dwrench>());
  }

};
}  // namespace python
}  // namespace tsid

#endif  // ifndef __tsid_python_measured_6d_wrench_hpp__