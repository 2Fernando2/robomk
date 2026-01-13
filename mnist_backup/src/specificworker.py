#!/usr/bin/python3
# -*- coding: utf-8 -*-
#
#    Copyright (C) 2025 by YOUR NAME HERE
#
#    This file is part of RoboComp
#
#    RoboComp is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    RoboComp is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
#

from PySide6.QtCore import QTimer
from PySide6.QtWidgets import QApplication
from matplotlib.bezier import inside_circle
from rich.console import Console
from genericworker import *
from train_mnist import Net
import interfaces as ifaces
import numpy as np
import time
import traceback
import cv2
import torch
import itertools
import math
import os

sys.path.append('/opt/robocomp/lib')
console = Console(highlight=False)


class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map, configData, startup_check=False):
        super(SpecificWorker, self).__init__(proxy_map, configData)
        
        self.Period = configData["Period"]["Compute"]
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.model = self._load_model()
        self.current_number = -1

        if startup_check:
            self.startup_check()
        else:
            started_camera = False
            while not started_camera:
                try:
                    print("Connecting to Camera360RGB...")
                    self.rgb_original = self.camera360rgb_proxy.getROI(-1, -1, -1, -1, -1, -1)
                    print("Camera specs:")
                    print(" width:", self.rgb_original.width)
                    print(" height:", self.rgb_original.height)
                    print(" focalx", self.rgb_original.focalx)
                    print(" focaly", self.rgb_original.focaly)
                    print(" period", self.rgb_original.period)
                    print(" ratio {:.2f}".format(self.rgb_original.width / self.rgb_original.height))
                    started_camera = True
                    print("Connected to Camera360RGB")
                except Ice.Exception as e:
                    traceback.print_exc()
                    print(e, "Trying again CAMERA...")

            self.timer.timeout.connect(self.compute)
            self.timer.start(self.Period)

    def __del__(self):
        """Destructor"""

    @QtCore.Slot()
    def compute(self):
        image = self.camera360rgb_proxy.getROI(-1, -1, -1, -1, -1, -1)
        color = np.frombuffer(image.image, dtype=np.uint8).reshape(image.height, image.width, 3)
        rect = self.detect_frame(color)
        color_copy = color.copy()
        if rect is not None:
            x1, y1, x2, y2 = rect
            cv2.rectangle(color_copy, (x1, y1), (x2, y2), (0, 255, 0), 2)
            image = color[y1:y2, x1:x2]
            self.current_number = self.process_image(image)
            print(f"Detected number: {self.current_number}")
            #cv2.imshow("Detected ROI", image)
        else:
            self.current_number = -1
        cv2.imshow("Camera360RGB", color_copy)
        cv2.waitKey(1)

    ################################################################

    def detect_frame(self, color):
        color_copy = color.copy()
        gray = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)
        _, edges = cv2.threshold(
            gray, 0, 255,
            cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU
        )

        # Find contours
        contours, _ = cv2.findContours( edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE )

        if not contours:
            print("No contours found")
            return None

        best_cnt = None
        best_score = -1

        h, w = gray.shape

        candidates = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 0.01 * w * h:  # skip tiny contours
                continue

            # Approximate contour to polygon
            peri = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, 0.05 * peri, True)

            # Get bounding box
            x, y, bw, bh = cv2.boundingRect(approx)
            aspect_ratio = bw / float(bh)

            # Check "squareness"
            if 0.5 <= aspect_ratio <= 2.0:
                candidates.append((area,x, y, bw, bh))

        for c in candidates:
            _, x, y, bw, bh = c

            # Score candidates based on the amount of white pixels inside
            image = edges[y:y+bh, x:x+bw]
            white_pixels = cv2.countNonZero(image)
            total_pixels = bw * bh
            white_ratio = white_pixels / total_pixels
            score = white_ratio
            if score > best_score:
                best_score = score
                best_cnt = c

        if best_cnt is not None:
            # Crop inside the frame to get the white area + digit only
            margin = int(min(bw, bh) * 5 / 100)  # 5% margin
            x1 = max(0, x + margin)
            y1 = max(0, y + margin)
            x2 = min(w, x + bw - margin)
            y2 = min(h, y + bh - margin)

            if x2 <= x1 or y2 <= y1:
                return None

            return [x1, y1, x2, y2]
        else:
            return None

    ####################################################################
    def startup_check(self):
        print(f"Testing RoboCompCamera360RGB.TRoi from ifaces.RoboCompCamera360RGB")
        test = ifaces.RoboCompCamera360RGB.TRoi()
        print(f"Testing RoboCompCamera360RGB.TImage from ifaces.RoboCompCamera360RGB")
        test = ifaces.RoboCompCamera360RGB.TImage()
        QTimer.singleShot(200, QApplication.instance().quit)

    # =============== Methods the component implements ==================
    # ===================================================================
    def _load_model(self):
        model = Net().to(self.device)
        model_path = "my_network.pt"
        if os.path.exists(model_path):
            model.load_state_dict(torch.load(model_path, map_location=self.device))
            model.eval()
            print(f"Modelo cargado desde: {model_path}")
        else:
            print(f"Error: No se encontró el modelo en: {model_path}")
            raise FileNotFoundError(f"Model not found: {model_path}")
        return model

    def process_image(self, image):
        try:
            # Convert to gray scale 
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            # Resize to MNIST size
            gray = cv2.resize(gray, (28, 28))
            # Normalize
            gray = gray.astype(np.float32) / 255.0
            gray = (gray - 0.1307) / 0.381
            # Tensor conversion
            tensor = torch.from_numpy(gray).unsqueeze(0).unsqueeze(0).to(self.device)
            # Forward pass
            with torch.no_grad():
                output = self.model(tensor)
                prediction = output.argmax(dim=1).item()
            return prediction
        except Exception as e:
            print(f"Error on process_image: {e}")
            traceback.print_exc()
            return -1

    #
    # IMPLEMENTATION of getNumber method from MNIST interface
    #
    def MNIST_getNumber(self):

        #
        # call DNN and return detection result
        #
        digit = ifaces.MNIST.Digit()
        digit.value = self.current_number
        return digit

    # ===================================================================
    # ===================================================================

    ######################
    # From the RoboCompCamera360RGB you can call this methods:
    # RoboCompCamera360RGB.TImage self.camera360rgb_proxy.getROI(int cx, int cy, int sx, int sy, int roiwidth, int roiheight)

    ######################
    # From the RoboCompCamera360RGB you can use this types:
    # ifaces.RoboCompCamera360RGB.TRoi
    # ifaces.RoboCompCamera360RGB.TImage


