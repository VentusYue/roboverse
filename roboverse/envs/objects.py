import pybullet_data
import pybullet as p
import os
import roboverse.bullet as bullet
import numpy as np

CUR_PATH = os.path.dirname(os.path.realpath(__file__))
ASSET_PATH = os.path.join(CUR_PATH, '../assets')
SHAPENET_ASSET_PATH = os.path.join(ASSET_PATH, 'bullet-objects/ShapeNetCore')
SHAPENET_SCALE = 0.5
"""
NOTE: Use this file only for core objects, add others to bullet/object_utils.py
This file will likely be deprecated in the future.
"""


def table():
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    # table_id = p.loadURDF('table/table.urdf',
    #                     #   basePosition=[.75, -.2, -1],
    #                       basePosition=[.65, -.2, -1],

    #                       baseOrientation=[0, 0, 0.707107, 0.707107],
    #                       globalScaling=1.0)
    table_path = os.path.join(ASSET_PATH, 'bullet-objects/table/table_test.urdf') #table.urdf
    # print(f"load table_path: {table_path}")

    table_id = p.loadURDF(table_path,
                        #   basePosition=[.75, -.2, -1],
                          basePosition=[.63, -.23, -1],

                          baseOrientation=[0, 0, 0.707107, 0.707107],
                          globalScaling=1.0)

    return table_id


def tray(base_position=(.60, 0.3, -.37), scale=0.5):
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    tray_id = p.loadURDF(
                         'tray/tray.urdf',
                         basePosition=base_position,
                         baseOrientation=[0, 0, 0.707107, 0.707107],
                         globalScaling=scale
                         )

    # tray_path = os.path.join(ASSET_PATH, 'bullet-objects/tray/tray.urdf') #table.urdf
    # tray_id = p.loadURDF(
    #                     tray_path,
    #                      basePosition=base_position,
    #                      baseOrientation=[0, 0, 0.707107, 0.707107],
    #                      globalScaling=scale
    #                      )
    return tray_id


def widow250(basePosition=[0.6, 0.0, -0.4]):
    widow250_path = os.path.join(ASSET_PATH,
                                 'interbotix_descriptions/urdf/wx250s.urdf')
    widow250_id = p.loadURDF(widow250_path,
                             basePosition=basePosition,
                             baseOrientation=bullet.deg_to_quat([0., 0., 0])
                             )
    return widow250_id

#(0.5, -0.3, -0.385)
def monitor(basePosition=(1.6, -0.1, -0.42), scale=32):
    monitor_path = os.path.join(ASSET_PATH,
                                 'bullet-objects/monitor/monitor.urdf')
    monitor_id = p.loadURDF(monitor_path,
                             basePosition=basePosition,
                             baseOrientation=bullet.deg_to_quat([0, 0., 180]),
                             globalScaling=scale
                             )
    return monitor_id

def monitor_v1(basePosition=(0.3, -0.3, 0.), scale=1.5):
    monitor_path = os.path.join(ASSET_PATH,
                                 'bullet-objects/monitor_v1/monitor.urdf')
    monitor_id = p.loadURDF(monitor_path,
                             basePosition=basePosition,
                             baseOrientation=bullet.deg_to_quat([90, 0., 180.0 ]),
                             globalScaling=scale,
                            #  flags = p.URDF_USE_MATERIAL_COLORS_FROM_MTL,

                             )
    return monitor_id


def monitor_v2(basePosition=(0.5, -0.3, 0.), scale=3):
    monitor_path = os.path.join(ASSET_PATH,
                                 'bullet-objects/monitor_v2/model.urdf')
    monitor_id = p.loadURDF(monitor_path,
                             basePosition=basePosition,
                             baseOrientation=bullet.deg_to_quat([90, 0., 0. ]),
                             globalScaling=scale
                             )
    return monitor_id

def books(basePosition=(1.2, -0.35, -0.37), scale=1.8):
    books_path = os.path.join(ASSET_PATH,
                                 'bullet-objects/books/books.urdf')
    books_id = p.loadURDF(books_path,
                             basePosition=basePosition,
                             baseOrientation=bullet.deg_to_quat([90, 0., 0]),
                             globalScaling=scale,
                             flags = p.URDF_USE_MATERIAL_COLORS_FROM_MTL,

                             )
    return books_id

def laptop(basePosition=(1.2, 0.2, -0.38), scale=1.0):
    laptop_path = os.path.join(ASSET_PATH,
                                 'bullet-objects/laptop/laptop.urdf')
    laptop_id = p.loadURDF(laptop_path,
                             basePosition=basePosition,
                             baseOrientation=bullet.deg_to_quat([90.0, 0., 180]),
                             globalScaling=scale,
                            #  flags = p.URDF_USE_MATERIAL_COLORS_FROM_MTL,
                             )
    return laptop_id


def lamp(basePosition=(-0.2, -0.2, -0.37), scale=0.07):
    lamp_path = os.path.join(ASSET_PATH,
                                 'bullet-objects/lamp/lamp.urdf')
    lamp_id = p.loadURDF(lamp_path,
                             basePosition=basePosition,
                             baseOrientation=bullet.deg_to_quat([90.0, 0., 180]),
                             globalScaling=scale,
                             flags = p.URDF_USE_MATERIAL_COLORS_FROM_MTL,
                             )
    return lamp_id

def lamp_v1(basePosition=(-0.2, -0.2, -0.37), scale=0.07):
    lamp_path = os.path.join(ASSET_PATH,
                                 'bullet-objects/lamp_v1/lamp.sdf')
    lamp_id = p.loadSDF(lamp_path,
                            #  basePosition=basePosition,
                            #  baseOrientation=bullet.deg_to_quat([90.0, 0., 180]),
                             globalScaling=scale,
                            #  flags = p.URDF_USE_MATERIAL_COLORS_FROM_MTL,
                             )
    return lamp_id

def officedesk(basePosition=(0.6, 0.25, -1.34), scale=0.0024):
    officedesk_path = os.path.join(ASSET_PATH,
                                 'bullet-objects/officedesk/desk.urdf')
    officedesk_id = p.loadURDF(officedesk_path,
                             basePosition=basePosition,
                             baseOrientation=bullet.deg_to_quat([90, 0., 0.]),
                             globalScaling=scale,
                            #  URDF_USE_MATERIAL_COLORS_FROM_MTL=True,
                            # flags = p.URDF_USE_MATERIAL_COLORS_FROM_MTL,
                             )
    return officedesk_id

def officedesk_v1(basePosition=(0.6, 0.25, -1.34), scale=4):
    officedesk_path = os.path.join(ASSET_PATH,
                                 'bullet-objects/officedesk_v1/desk.urdf')
    officedesk_id = p.loadURDF(officedesk_path,
                             basePosition=basePosition,
                             baseOrientation=bullet.deg_to_quat([90, 0., 0.]),
                             globalScaling=scale,
                            #  URDF_USE_MATERIAL_COLORS_FROM_MTL=True,
                            # flags = p.URDF_USE_MATERIAL_COLORS_FROM_MTL,
                             )
    return officedesk_id

def load_shapenet_trashcan(object_position=(-0.2, 0, -1.58),
                         object_quat=(1, -1, 0, 0),  scale=1.0):
    filepath_visual = os.path.join(ASSET_PATH,
                                 'bullet-objects/shapenetnew/trash/models/model_normalized.obj')
    filepath_collision = os.path.join(ASSET_PATH,
    'bullet-objects/shapenetnew/trash/vhacd/model.obj')

    dir_name = '03211117'
    object_name = '50934056d4c1735dc9d02d4e580c2722'

    # dir_name = '02747177'
    # object_name = '4dbbece412ef64b6d2b12aa6a0f050b3'
                    # 4dbbece412ef64b6d2b12aa6a0f050b3
    # filepath_collision = os.path.join(
    #     SHAPENET_ASSET_PATH,
    #     'ShapeNetCore_vhacd/{0}/{1}/model.obj'.format(dir_name, object_name))
    # filepath_visual = os.path.join(
    #     SHAPENET_ASSET_PATH,
    #     'ShapeNetCore.v2/{0}/{1}/models/model_normalized.obj'.format(
    #         dir_name, object_name))
    scale = SHAPENET_SCALE * scale 
    collisionid = p.createCollisionShape(p.GEOM_MESH,
                                         fileName=filepath_collision,
                                         meshScale=scale * np.array([1, 1, 1]))
    visualid = p.createVisualShape(p.GEOM_MESH, fileName=filepath_visual,
                                   meshScale=scale * np.array([1, 1, 1]))
    body = p.createMultiBody(0.05, collisionid, visualid)
    p.resetBasePositionAndOrientation(body, object_position, object_quat)
    return body



def room(basePosition=(-0.4, 0,-1.4 ), scale=0.8):
    room_path = os.path.join(ASSET_PATH,
                                 'room_descriptions/urdf/room_v0.urdf')
    room_id = p.loadURDF(room_path,
                        globalScaling=scale,
                         basePosition=basePosition,
                         baseOrientation=bullet.deg_to_quat([0, 0., -90]),
                            )
    return room_id


def room_v1(basePosition=(1.5, 0.6,-0.5), scale=0.6):
    room_path = os.path.join(ASSET_PATH,
                                 'room_descriptions/urdf/room_v1.urdf')
    room_id = p.loadURDF(room_path,
                        globalScaling=scale,
                         basePosition=basePosition,
                         baseOrientation=bullet.deg_to_quat([0, 0., -90]),
                            )
    return room_id


def keyboard(basePosition=(1.6, 0.1, -0.375), scale=1.2):
    keyboard_path = os.path.join(ASSET_PATH,
                                 'bullet-objects/keyboard/keyboard.urdf')
    keyboard_id = p.loadURDF(keyboard_path,
                        globalScaling=scale,
                         basePosition=basePosition,
                         baseOrientation=bullet.deg_to_quat([0, 0., 0]),
                            )
    return keyboard_id


def desktop(basePosition=(0.75, -0.1, -1.29), scale=0.028):
    keyboard_path = os.path.join(ASSET_PATH,
                                 'bullet-objects/desktop/desktop.urdf')
    keyboard_id = p.loadURDF(keyboard_path,
                        globalScaling=scale,
                         basePosition=basePosition,
                         baseOrientation=bullet.deg_to_quat([0, 0., 0]),
                            )
    return keyboard_id