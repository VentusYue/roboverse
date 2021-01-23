import pybullet_data
import pybullet as p
import os
import roboverse.bullet as bullet

CUR_PATH = os.path.dirname(os.path.realpath(__file__))
ASSET_PATH = os.path.join(CUR_PATH, '../assets')
SHAPENET_ASSET_PATH = os.path.join(ASSET_PATH, 'bullet-objects/ShapeNetCore')

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

def monitor(basePosition=(0.5, -0.3, -0.42), scale=32):
    monitor_path = os.path.join(ASSET_PATH,
                                 'bullet-objects/monitor/monitor.urdf')
    monitor_id = p.loadURDF(monitor_path,
                             basePosition=basePosition,
                             baseOrientation=bullet.deg_to_quat([0, 0., 180]),
                             globalScaling=scale
                             )
    return monitor_id

def books(basePosition=(1.2, -0.35, -0.37), scale=1.8):
    books_path = os.path.join(ASSET_PATH,
                                 'bullet-objects/books/books.urdf')
    books_id = p.loadURDF(books_path,
                             basePosition=basePosition,
                             baseOrientation=bullet.deg_to_quat([90, 0., 0]),
                             globalScaling=scale
                             )
    return books_id

def laptop(basePosition=(1.2, 0.2, -0.38), scale=1.0):
    laptop_path = os.path.join(ASSET_PATH,
                                 'bullet-objects/laptop/laptop.urdf')
    laptop_id = p.loadURDF(laptop_path,
                             basePosition=basePosition,
                             baseOrientation=bullet.deg_to_quat([90.0, 0., 180]),
                             globalScaling=scale
                             )
    return laptop_id


def lamp(basePosition=(-0.2, -0.2, -0.37), scale=0.07):
    lamp_path = os.path.join(ASSET_PATH,
                                 'bullet-objects/lamp/lamp.urdf')
    lamp_id = p.loadURDF(lamp_path,
                             basePosition=basePosition,
                             baseOrientation=bullet.deg_to_quat([90.0, 0., 180]),
                             globalScaling=scale
                             )
    return lamp_id


def officedesk(basePosition=(-0.2, 0, -1.58), scale=0.003):
    officedesk_path = os.path.join(ASSET_PATH,
                                 'bullet-objects/officedesk/desk.urdf')
    officedesk_id = p.loadURDF(officedesk_path,
                             basePosition=basePosition,
                             baseOrientation=bullet.deg_to_quat([90, 0., 0.]),
                             globalScaling=scale
                             )
    return officedesk_id
