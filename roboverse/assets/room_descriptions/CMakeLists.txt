# Copyright (c) 2014-2015 CNRS
# Author: Mylene Campana
#
# This file is part of room_description.
# room_description is free software: you can redistribute it
# and/or modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation, either version
# 3 of the License, or (at your option) any later version.
#
# room_description is distributed in the hope that it will be
# useful, but WITHOUT ANY WARRANTY; without even the implied warranty
# of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# hpp-hrp2.  If not, see
# <http://www.gnu.org/licenses/>.

# Requires at least CMake 2.6 to configure the package.
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

INCLUDE(cmake/base.cmake)
INCLUDE(cmake/python.cmake)

SET(PROJECT_NAME room_description)
SET(PROJECT_DESCRIPTION "Data specific to room environment for hpp-corbaserver")
SET(PROJECT_URL "")

SETUP_PROJECT()

FINDPYTHON()
ADD_REQUIRED_DEPENDENCY ("hpp-corbaserver >= 3")

SET(CATKIN_PACKAGE_SHARE_DESTINATION
  ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME})

install(FILES
  urdf/room.urdf
  urdf/room_only_meshes.urdf
  urdf/walls.urdf
  urdf/squel.urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf)

#meshes
install(FILES
  meshes/bureauAngulaire_sized.dae
  meshes/bureau_angulaire.dae
  meshes/pc_portable_sized2.dae
  meshes/lampe_a_pied_sized2.dae
  meshes/fauteuil_sized2.dae
  meshes/chaise_bureau2_sized2.dae
  meshes/table_verre_sized.dae
  meshes/commode_sized2.dae
  meshes/vieux_livres_sized.dae
  meshes/desklamp_sized2.dae
  meshes/beer_sized2.dae
  meshes/beer_bottle.dae
  meshes/etagere_bois_sized2.dae
  meshes/etagere_bois.dae
  meshes/table_basse.dae
  meshes/carpet.dae
  meshes/wall_carpet.dae
  meshes/table_basse_truquee.dae
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/meshes)

#install(FILES meshes/alcohol_1_beer2(1).png
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/meshes
#  )
#install(FILES meshes/Wood_700_x_466_px_290214_fg_(copy).jpg
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/meshes
#  )

install(FILES srdf/squel.srdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/srdf
  )

install (FILES
  src/hpp/corbaserver/squel_robot/robot.py
  src/hpp/corbaserver/squel_robot/__init__.py
  DESTINATION ${PYTHON_SITELIB}/hpp/corbaserver/squel_robot)
SETUP_PROJECT_FINALIZE()
