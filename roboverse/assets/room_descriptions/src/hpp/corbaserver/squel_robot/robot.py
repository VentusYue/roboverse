#!/usr/bin/env python
# Copyright (c) 2014 CNRS
# Author: Mylene Campana
#
# This file is part of hpp-corbaserver.
# hpp-corbaserver is free software: you can redistribute it
# and/or modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation, either version
# 3 of the License, or (at your option) any later version.
#
# hpp-corbaserver is distributed in the hope that it will be
# useful, but WITHOUT ANY WARRANTY; without even the implied warranty
# of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# hpp-corbaserver.  If not, see
# <http://www.gnu.org/licenses/>.

# Call urdf description of an universal 6-DoF arm robot.

from hpp.corbaserver.robot import Robot as Parent

class Robot (Parent):
    packageName = "room_description"
    urdfName = "squel"
    urdfSuffix = ""
    srdfSuffix = ""

    def __init__ (self, robotName):
        Parent.__init__ (self, robotName, "freeflyer")
        self.tf_root = "Pelvis"
        # same as the urdf root name
