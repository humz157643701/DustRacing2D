// This file is part of Dust Racing (DustRAC).
// Copyright (C) 2012 Jussi Lind <jussi.lind@iki.fi>
//
// DustRAC is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// DustRAC is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with DustRAC. If not, see <http://www.gnu.org/licenses/>.

#ifndef CENTRIFUGALFORCEGENERATOR_HPP
#define CENTRIFUGALFORCEGENERATOR_HPP

#include "MiniCore/Core/MCForceGenerator"

//! Adds centrifugal force to the cars.
class CentrifugalForceGenerator : public MCForceGenerator
{
public:

    //! Constructor.
    CentrifugalForceGenerator();

    //! \reimp
    virtual void updateForce(MCObject & object);
};

#endif // CENTRIFUGALFORCEGENERATOR_HPP