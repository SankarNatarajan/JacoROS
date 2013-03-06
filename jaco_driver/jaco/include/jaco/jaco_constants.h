/*
 * Copyright (c) 2011  DFKI GmbH, Bremen, Germany
 *
 *  This file is free software: you may copy, redistribute and/or modify it
 *  under the terms of the GNU General Public License as published by the
 *  Free Software Foundation, either version 3 of the License, or (at your
 *  option) any later version.
 *
 *  This file is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *
 *
 *  Author: Sankaranarayanan Natarajan / sankar.natarajan@dfki.de
 *
 *  FILE --- jaco_constants.h
 *
 *  PURPOSE ---   Constants regarding for Jaco manipulator
 */

#ifndef JACO_CONSTANTS_H_
#define JACO_CONSTANTS_H_

namespace kinova
{
	// degree of freedom
	const size_t DOF = 6;

	// joint numbers
	const size_t NUM_JOINTS = 6;

	// finger joint numbers
	const size_t NUM_FINGER_JOINTS = 3;


} // namespace kinova

#endif /* JACO_CONSTANTS_H_ */
