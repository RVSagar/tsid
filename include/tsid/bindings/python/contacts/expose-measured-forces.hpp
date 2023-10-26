//
// Copyright (c) 2018 CNRS, NYU, MPI Tübingen
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

#ifndef __tsid_python_expose_measured_forces_hpp__
#define __tsid_python_expose_measured_forces_hpp__

#include "tsid/bindings/python/contacts/measured-3Dforce.hpp"
#include "tsid/bindings/python/contacts/measured-6Dwrench.hpp"

namespace tsid {
namespace python {
void exposeMeasured6Dwrench();
void exposeMeasured3Dforce();

inline void exposeMeasuredForces() {
  exposeMeasured6Dwrench();
  exposeMeasured3Dforce();
}

}  // namespace python
}  // namespace tsid
#endif  // ifndef __tsid_python_expose_measured_forces_hpp__
