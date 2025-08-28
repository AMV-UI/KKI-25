#!/usr/bin/env python3
import cv2
import numpy as np


def enlightment(frame, gamma=0.7):
    """
    Take image(BGR) returns enchanted image(BGR) using Contrast Limited Adaptive Histogram Equalization \
    (https://docs.opencv.org/3.4/d5/daf/tutorial_py_histogram_equalization.html)

    :param image: image array (BGR)
    :param gamma: (optional) default 0.7
    :return: enchanted image (BGR)
    """
    image = frame.copy()
    # Apply gamma adjustment
    if gamma == 0:
        return image

    # Build a lookup table mapping the pixel values [0, 255]
    # their adjusted gamma values
    inverted = 1.0 / gamma
    table = np.array([((i / 255.0) ** inverted) *
                     255 for i in np.arange(256)]).astype("uint8")

    # apply gamma correction using the lookup table
    return cv2.LUT(image, table)

def preprocess(frame, contrast=1, brightness=1, gamma=1):
    # Gamma checker
    frame = enlightment(frame, gamma=gamma)

    # Change contrast and brightness
    frame = frame * (1 + contrast / 127) - \
        contrast + brightness
    frame = np.clip(frame, 0, 255)
    frame = np.uint8(frame)

    return frame