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
        color_full = np.frombuffer(image.image, dtype=np.uint8).reshape(image.height, image.width, 3)
        h, w, _ = color_full.shape
        left_offset = w // 6
        vert_offset = h // 4
        if w <= 2*left_offset or h <= 2*vert_offset:
            print("Error: Small image for ROI")
            return
        color = color_full[vert_offset : h - vert_offset, left_offset : w - left_offset]
        # color = np.frombuffer(image.image, dtype=np.uint8).reshape(image.height, image.width, 3)
        rect = self.detect_frame(color)
        color_copy = color.copy()
        if rect is not None:
            x1, y1, x2, y2 = rect
            cv2.rectangle(color_copy, (x1, y1), (x2, y2), (0, 255, 0), 2)
            image = color[y1:y2, x1:x2]
            self.current_number = self.process_image(image)
            #cv2.putText(color, f"Num: {self.current_number}", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
            #print(f"Detected number: {self.current_number}")
            #cv2.imshow("Detected ROI", image)
        else:
            self.current_number = -1
        cv2.imshow("Camera360RGB", color_copy)
        cv2.waitKey(1)

    ################################################################

    def detect_frame(self, color):
        gray = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)
        binary = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 11, 2)
        contours, hierarchy = cv2.findContours(binary, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            return None
        
        best_rect = None
        max_area = 0
        h, w = gray.shape


        for i, cnt in enumerate(contours):
            area = cv2.contourArea(cnt)
            if 200 < area < (w * h * 0.20):
                peri = cv2.arcLength(cnt, True)
                approx = cv2.approxPolyDP(cnt, 0.04 * peri, True)
                if len(approx) == 4:
                    x, y, bw, bh = cv2.boundingRect(approx)
                    aspect_ratio = float(bw) / bh
                    if 0.6 < aspect_ratio < 1.4:
                        if hierarchy is not None and hierarchy[0][i][2] != -1:
                            if area > max_area:
                                max_area = area
                                margin = int(min(bw, bh) * 0.05)
                                x1, y1 = max(0, x+margin), max(0, y+margin)
                                x2, y2 = min(w, x+bw-margin), min(h, y+bh-margin)
                                if x2 > x1 and y2 > y1:
                                    best_rect = [x1, y1, x2, y2]
        return best_rect

        # color_copy = color.copy()
        # gray = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)
        # # gray = cv2.GaussianBlur(gray, (5, 5), 0)
        
        # # _, edges = cv2.threshold( gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
        # _, edges = cv2.threshold( gray, 80, 255, cv2.THRESH_BINARY_INV)

        # # Find contours
        # contours, _ = cv2.findContours( edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE )

        # if not contours:
        #     print("No contours found")
        #     return None

        # best_cnt = None
        # best_score = -1

        # h, w = gray.shape

        # candidates = []
        # for cnt in contours:
        #     area = cv2.contourArea(cnt)
        #     if area < 0.01 * w * h:  # skip tiny contours
        #         continue

        #     # Approximate contour to polygon
        #     peri = cv2.arcLength(cnt, True)
        #     approx = cv2.approxPolyDP(cnt, 0.05 * peri, True)

        #     # Get bounding box
        #     x, y, bw, bh = cv2.boundingRect(approx)
        #     aspect_ratio = bw / float(bh)

        #     # Check "squareness"
        #     if 0.5 <= aspect_ratio <= 2.0:
        #         candidates.append((area,x, y, bw, bh))

        # for c in candidates:
        #     _, x, y, bw, bh = c

        #     # Score candidates based on the amount of white pixels inside
        #     image = edges[y:y+bh, x:x+bw]
        #     white_pixels = cv2.countNonZero(image)
        #     total_pixels = bw * bh
        #     white_ratio = white_pixels / total_pixels
        #     score = white_ratio
        #     if score > best_score:
        #         best_score = score
        #         best_cnt = c

        # if best_cnt is not None:
        #     # Crop inside the frame to get the white area + digit only
        #     margin = int(min(bw, bh) * 5 / 100)  # 5% margin
        #     x1 = max(0, x + margin)
        #     y1 = max(0, y + margin)
        #     x2 = min(w, x + bw - margin)
        #     y2 = min(h, y + bh - margin)

        #     if x2 <= x1 or y2 <= y1:
        #         return None

        #     return [x1, y1, x2, y2]
        # else:
        #     return None

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
            # Remove raw square from image
            h_raw, w_raw = gray.shape
            margin_h, margin_w = int(h_raw*0.15), int(w_raw*0.15)
            if margin_h * 2 < h_raw and margin_w * 2 < w_raw:
                gray = gray[margin_h : h_raw-margin_h, margin_w : w_raw-margin_w]
            # Binarize and INV
            _, thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
            # Rescale and center image to only get the number 
            coords = cv2.findNonZero(thresh)
            if coords is None: return -1 # full black, not number
            x, y, w, h = cv2.boundingRect(coords)
            digit_crop = thresh[y: y+h, x : x+w]
            # Erode bold numbers
            aspect_ratio = float(w)/h
            if aspect_ratio > 0.5:
                kernel = np.ones((2, 2), np.uint8)
                digit_crop = cv2.erode(digit_crop, kernel, iterations=1)
            # Resize to MNIST size
            h, w = digit_crop.shape
            max_side = max(h, w)
            if max_side == 0: return -1
            scale = 20.0 / max_side
            new_h, new_w = int(h*scale), int(w*scale)
            if new_h <= 0 or new_w <= 0: return -1
            resize = cv2.resize(thresh, (new_w, new_h), interpolation=cv2.INTER_AREA) 
            # Normalize and canvas adapt
            canvas = np.zeros((28, 28), dtype=np.uint8)
            start_h, start_w = (28-new_h) // 2, (28-new_w) // 2
            canvas[start_h : start_h+new_h, start_w : start_w+new_w] = resize 
            # Center by mass
            moments = cv2.moments(canvas)
            if moments['m00'] > 0:
                cx = moments['m10'] / moments['m00']
                cy = moments['m01'] / moments['m00']
                shift_x, shift_y = 14.0-cx, 14.0-cy
                mat = np.float32([[1, 0, shift_x], [0, 1, shift_y]])
                canvas = cv2.warpAffine(canvas, mat, (28, 28))
            # Debug
            cv2.imshow("Input to MNIST (28x28)", cv2.resize(canvas, (280, 280), interpolation=cv2.INTER_NEAREST))
            # PyTorch normalization
            final_img = canvas.astype(np.float32) / 255.0
            final_img = (final_img - 0.1307) / 0.3081
            # Tensor conversion
            tensor = torch.from_numpy(final_img).unsqueeze(0).unsqueeze(0).to(self.device)
            # Forward pass
            with torch.no_grad():
                output = self.model(tensor)
                probs = torch.nn.functional.softmax(output, dim=1)
                # prediction = output.argmax(dim=1).item()
                confidence, prediction = torch.max(probs, 1)
                pred, conf = prediction.item(), confidence.item() 
                print(f"Detected: {pred} (Conf: {conf:.2f})")
            return pred
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
        digit = ifaces.RoboCompMNIST.Digit()
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


