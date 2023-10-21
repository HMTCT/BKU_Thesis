#Global config

import os
import sys

ROS_NODE_NAME = "fruits_classification"

"""
TRT_ENGINE_PATH = os.path.join(sys.path[0], "waste_v5_160.trt")
TRT_INPUT_SIZE = 160
TRT_CLASS_NAMES = ('Banana Peel', 'Broken Bones', 'Cigarette End', 'Disposable Chopsticks',
                   'Ketchup', 'Marker', 'Oral Liquid Bottle', 'Plate',
                   'Plastic Bottle', 'Storage Battery', 'Toothbrush', 'Umbrella')
TRT_NUM_CLASSES = 12
WASTE_CLASSES = {
    'food_waste': ('Banana Peel', 'Broken Bones', 'Ketchup'),
    'hazardous_waste': ('Marker', 'Oral Liquid Bottle', 'Storage Battery'),
    'recyclable_waste': ('Plastic Bottle', 'Toothbrush', 'Umbrella'),
    'residual_waste': ('Plate', 'Cigarette End', 'Disposable Chopsticks'),
}
COLORS = {
    'recyclable_waste': (0, 0, 255),
    'hazardous_waste': (255, 0, 0),
    'food_waste': (0, 255, 0),
    'residual_waste': (80, 80, 80)
}
TARGET_POSITIONS = {
    'recyclable_waste': (163, -60, 65, 65),
    'hazardous_waste': (163, -10, 65, 85),
    'food_waste': (163, -10 + 52, 65, 100),
    'residual_waste': (163, -10 + 52 * 2, 65, 118)
}
"""

"""
TRT_ENGINE_PATH = os.path.join(sys.path[0], "train_data_fruits_demo.trt")
TRT_INPUT_SIZE = 160
TRT_CLASS_NAMES = ('Chom Chom')
TRT_NUM_CLASSES = 1
WASTE_CLASSES = {
    'chomchom': ('Chom Chom')
}
COLORS = {
    'chomchom': (0, 255, 255)
}
TARGET_POSITIONS = {
    'chomchom': (163, -60, 65, 65)
}
"""

#TRT_ENGINE_PATH = os.path.join(sys.path[0], "last_training_fruits.trt")

TRT_ENGINE_PATH = os.path.join(sys.path[0], "last_training_fruits.trt")
TRT_INPUT_SIZE = 160
#TRT_CLASS_NAMES = ('Banana', 'Lemon')
TRT_CLASS_NAMES = ('Banana', 'Lemon','mangosteen','apple','greenMS')
TRT_NUM_CLASSES = 5
WASTE_CLASSES = {
    'banana': ('Banana'),
    'lemon': ('Lemon'),
    'mangosteen': ('mangosteen'),
    'apple': ('apple'),
    'greenms': ('greenMS')
}
COLORS = {
    'banana': (0, 255, 0),
    'lemon': (255, 0, 0),
    'mangosteen': (255, 0, 0),
    'apple': (255, 0, 0),
    'greenms' : (255, 0, 0)
}
TARGET_POSITIONS = {
    'banana': 1,
    'lemon': -1,
    'mangosteen': -1,
    'apple': 1,
    'greenms':1
}
GRIPPER_ANGLES = {
    'banana': 80,
    'lemon': 90,
    'mangosteen': 90,
    'apple': 90,
    'greenms' : 90
}