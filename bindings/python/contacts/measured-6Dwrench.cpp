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

#include "tsid/bindings/python/contacts/measured-6Dwrench.hpp"
#include "tsid/bindings/python/contacts/expose-measured-forces.hpp"

namespace tsid
{
  namespace python
  {
    void exposeMeasured6Dwrench()
    {
      Measured6DwrenchPythonVisitor<tsid::contacts::Measured6Dwrench>::expose("Measured6Dwrench");
    }
  }
}