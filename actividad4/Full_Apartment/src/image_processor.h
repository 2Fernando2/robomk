//
// Created by pbustos on 12/11/25.
//

// C++
#pragma once

#include <tuple>
#include <opencv2/opencv.hpp>
#include <QLabel>
#include <QImage>
#include <QPixmap>
#include <cmath>
#include <Camera360RGB.h>

namespace rc
{

    /**
     * @brief Static helper class for computer vision tasks using OpenCV.
     * Handles MNIST digit recognition and color patch detection.
     */
    struct ImageProcessor
    {

        /**
         * @brief Detects a number patch in the camera image using a MNIST neural network.
         * Includes an internal voting system to ensure detection stability over multiple frames.
         * @param mnist_proxy Proxy to the MNIST recognition component.
         * @param camera_proxy Proxy to the RGB camera.
         * @param label_img Optional Qt Label to display the debug image.
         * @return Tuple {
         *              (bool) Success (true if a stable number is identified).
         *              (int)  Detected number.
         *              (int)  Error or Direction hint (0 if centered, -1/1 for steering guidance.) }
         */
        static std::tuple<bool, int, int> check_number_in_image(RoboCompMNIST::MNISTPrxPtr mnist_proxy,
                                                                RoboCompCamera360RGB::Camera360RGBPrxPtr camera_proxy=nullptr,
                                                                QLabel *label_img = nullptr) {
            struct DetectionState {
                std::map<int, int> votes;
                int frames_with_detection = 0;
                int frames_without_detection = 0;
                int frames_centered = 0;
                std::chrono::steady_clock::time_point last_detection_time;

                void reset() {
                    votes.clear();
                    frames_with_detection = 0;
                    frames_without_detection = 0;
                    frames_centered = 0;
                }

                bool has_confidence_result(int &winner, int &confidence) const {
                    if (votes.empty()) return false;
                    auto winner_it = std::max_element(votes.begin(), votes.end(),
                        [](const auto &a, const auto &b){return a.second < b.second;});
                    winner = winner_it->first;
                    confidence = winner_it->second;

                    int total_votes = 0;
                    for (const auto &count: votes | std::views::values) total_votes += count;

                    if (total_votes < 5) return false;

                    float win_ratio = static_cast<float>(confidence) / static_cast<float>(total_votes);
                    return (win_ratio >= 0.6f) || (confidence >= 8);
                }
            };
            static DetectionState state;

            auto now = std::chrono::steady_clock::now();
            if (state.frames_without_detection == 0 && !state.votes.empty()) state.last_detection_time = now;
            if (!state.votes.empty()) {
                auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - state.last_detection_time).count();
                if (elapsed > 2) state.reset();
            }

            // Proxies availability
            if (!label_img || !camera_proxy) {qWarning() << "[IMAGE PROCESSOR] Proxies no inicializados."; return {false, -1, 0};};
            try {
                RoboCompCamera360RGB::TImage img = camera_proxy->getROI(-1, -1, -1, -1, -1, -1);
                cv::Mat cv_img(img.height, img.width, CV_8UC3, img.image.data());

                // Extract ROI
                const int left_offset = cv_img.cols / 6;
                const int vert_offset = cv_img.rows / 4;
                const cv::Rect roi_rect(left_offset, vert_offset, cv_img.cols -2 * left_offset, cv_img.rows -2 * vert_offset);

                if (roi_rect.width <= 0 || roi_rect.height <= 0) return {false, -1, 1};
                cv::Mat roi = cv_img(roi_rect);

                // Gray conversion y binarization
                cv::Mat gray, binary;
                cv::cvtColor(roi, gray, cv::COLOR_BGR2GRAY);

                // Adaptative threshold
                cv::adaptiveThreshold(gray, binary, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY_INV, 11, 2);

                // Contours search
                std::vector<std::vector<cv::Point>> contours;
                std::vector<cv::Vec4i> hierarchy;
                cv::findContours(binary, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

                // Filter best candidates
                cv::Rect best_rect;
                double max_area = 0;
                bool candidate_found = false;
                int w_img = gray.cols;
                int h_img = gray.rows;

                for (size_t i = 0; i < contours.size(); i++) {
                    double area = cv::contourArea(contours[i]);
                    // Noise (small) and wall (big) filter
                    if (area > 200 && area < (w_img * h_img * 0.20)){
                        cv::Rect rect = cv::boundingRect(contours[i]);
                        // Aspect filter
                        float aspect = static_cast<float>(rect.width) / static_cast<float>(rect.height);
                        if (aspect > 0.6 && aspect < 1.4) {
                            // Polygon approximation and filter
                            if (hierarchy[i][2] != -1){
                                std::vector<cv::Point> approx;
                                double peri = cv::arcLength(contours[i], true);
                                cv::approxPolyDP(contours[i], approx, 0.04*peri, true);
                                if (approx.size() == 4){
                                    if (area > max_area) {
                                        max_area = area;
                                        best_rect = rect;
                                        candidate_found = true;
                                    }
                                }
                            }
                        }
                    }
                }

                // Visualization (debug)
                cv::Mat display_img;
                cv::cvtColor(roi, display_img, cv::COLOR_BGR2RGB);
                if (!candidate_found) {
                    state.frames_without_detection++;
                    if (state.frames_without_detection > 20) if (!state.votes.empty()) state.reset();
                    if (label_img) update_label(label_img, display_img);
                    return {false, -1, 1};
                }

                state.frames_without_detection = 0;
                state.frames_with_detection++;

                // Visual Servoing
                cv::Point2f rect_center = (best_rect.tl() + best_rect.br()) * 0.5f;

                // Draw detected candidate
                cv::rectangle(display_img, best_rect, cv::Scalar(255, 0, 0), 2);
                cv::circle(display_img, rect_center, 5, cv::Scalar(0, 255, 0), -1);

                // Tolerance central zone
                const float img_center_x = static_cast<float>(display_img.cols) / 2.0f;
                const float align_tolerance = static_cast<float>(display_img.cols) / 10.0f;
                const float capture_tolerance = static_cast<float>(display_img.cols) / 2.2f;

                float offset_from_center = rect_center.x - img_center_x;
                int direction = 0;
                if (offset_from_center < -align_tolerance) direction = -1; // left
                else if (offset_from_center > align_tolerance) direction = 1; // right
                else { direction = 0; state.frames_centered++;} // centered

                if (std::abs(offset_from_center) < capture_tolerance) {
                    try {
                        RoboCompMNIST::Digit digit = mnist_proxy->getNumber();
                        int detected_val = digit.value;
                        if (detected_val > 0 && detected_val <= 9)
                            state.votes[detected_val]++;
                    } catch (const Ice::Exception &e) {}
                }

                // update GUI
                if (label_img) update_label(label_img, display_img);

                // Not centered, keep turning
                if (direction != 0) { return {false, -1, direction}; }

                // Confidence timeout - quicker turning
                if (state.frames_centered > 50) {
                    state.reset();
                    return {false, -1, 1};
                }

                // Centered, search for confidence result
                int winner = -1;
                int confidence = 0;
                if (state.has_confidence_result(winner, confidence))
                    if (winner > 0 && winner <= 2) {
                        qInfo() << "Number detected with high confidence: " << winner << " - Confidence: " << confidence;
                        state.reset();
                        return {true, winner, 0};
                    }
                qInfo() << "Not enough conficende, keep turning.." ;
                // Not enough confidence, keep turning a bit
                return {false, -1, 0.1};
            } catch (const Ice::Exception &e) {qWarning() << "[IMAGE PROCESSOR] Error reading camera for visualization";}
            return {false, -1, 0.1};
        }

        /**
         * @brief Updates a Qt QLabel with a OpenCV matrix image.
         * Scales the image to fit the label.
         */
        static void update_label(QLabel *label, const cv::Mat &img) {
            if (!label) return;
            QImage qimg(img.data, img.cols, img.rows, static_cast<int>(img.step), QImage::Format_RGB888);
            label->setPixmap(QPixmap::fromImage(qimg).scaled(label->size(), Qt::IgnoreAspectRatio, Qt::SmoothTransformation));
        }


        /**
         * @brief Detects a large color patch in the image.
         * Used for room identification in the absence of numbers.
         * @param proxy Proxy to the 360 RGB camera.
         * @param label_img Optional Qt Label for visualization.
         * @param min_nonzero Minimum pixel count to consider a valid detection.
         * @return Tuple {
         *              (bool) Success (true if a color patch is detected).
         *              (int)  Room index (0 for RED, 1 for GREEN).
         *              (int)  Direction hint (-1/1 for steering guidance.) }
         */
        static std::tuple<bool, int, int> check_colour_patch_in_image(RoboCompCamera360RGB::Camera360RGBPrxPtr proxy,
                                                                      QLabel *label_img = nullptr,
                                                                      int min_nonzero = 1000)
        {
            RoboCompCamera360RGB::TImage img;
            try{ img = proxy->getROI(-1, -1, -1, -1, -1, -1);}
            catch (const Ice::Exception &e){ std::cout << e.what() << " Error reading 360 camera " << std::endl; return {false, -1, 1}; }

            // convert to cv::Mat
            cv::Mat cv_img(img.height, img.width, CV_8UC3, img.image.data());

            // extract a ROI leaving out borders (same as original logic)
            const int left_offset = cv_img.cols / 8;
            const int vert_offset = cv_img.rows / 4;
            const cv::Rect roi(left_offset, vert_offset, cv_img.cols - 2 * left_offset, cv_img.rows - 2 * vert_offset);
            if (roi.width <= 0 || roi.height <= 0) return {false, -1, 1};
            cv_img = cv_img(roi);

            // Convert BGR -> RGB for display
            cv::Mat display_img;
            cv::cvtColor(cv_img, display_img, cv::COLOR_BGR2RGB);

            if (label_img)
            {
                QImage qimg(display_img.data, display_img.cols, display_img.rows, static_cast<int>(display_img.step), QImage::Format_RGB888);
                label_img->setPixmap(QPixmap::fromImage(qimg).scaled(label_img->size(), Qt::IgnoreAspectRatio, Qt::SmoothTransformation));
            }

            // Convert BGR -> HSV for color thresholding
            cv::Mat hsv_img;
            cv::cvtColor(cv_img, hsv_img, cv::COLOR_BGR2HSV);

            // Set ranges depending on color
            cv::Mat mask_green, mask_red_1, mask_red_2 = cv::Mat::zeros(hsv_img.size(), CV_8UC1);
            cv::inRange(hsv_img, cv::Scalar(35, 50, 50), cv::Scalar(85, 255, 255), mask_green);

            cv::inRange(hsv_img, cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255), mask_red_1);
            cv::inRange(hsv_img, cv::Scalar(160, 100, 100), cv::Scalar(179, 255, 255), mask_red_2);
            cv::Mat mask_red = mask_red_1 | mask_red_2;

            // remove small noise
            cv::morphologyEx(mask_green, mask_green, cv::MORPH_OPEN, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3,3)));
            cv::morphologyEx(mask_red, mask_red, cv::MORPH_OPEN, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3,3)));

            const int nonZeroCount_green = cv::countNonZero(mask_green);
            const int nonZeroCount_red = cv::countNonZero(mask_red);
            if (nonZeroCount_green < min_nonzero and nonZeroCount_red < min_nonzero)
                return {false, -1, 1};

            // get larger mask
            cv::Mat mask = (nonZeroCount_red > nonZeroCount_green) ? mask_red : mask_green;
            int room_index = (nonZeroCount_red > nonZeroCount_green) ? 0 : 1;

            // compute moments and center of red patch
            const cv::Moments mu = cv::moments(mask, true);
            if (mu.m00 < 1.0) return {false, -1, 1};

            cv::Point2f bestCenter(static_cast<float>(mu.m10 / mu.m00), static_cast<float>(mu.m01 / mu.m00));

            // decide turning direction: default right (1), left (-1)
            int left_right = 1;
            if (bestCenter.x < (display_img.cols / 2) && bestCenter.x > 0)
                left_right = -1;

            // check center is near middle of image (tolerance)
            const int tolerance = display_img.cols / 10;
            const int left_bound = display_img.cols / 2 - tolerance;
            const int right_bound = display_img.cols / 2 + tolerance;
            if ((bestCenter.x < left_bound) || (bestCenter.x > right_bound))
                return {false, -1, left_right};

            // draw marker on detected center and update label if provided
            cv::circle(display_img, bestCenter, 40, cv::Scalar(0, 255, 0), -1);
            if (label_img)
            {
                QImage qimg(display_img.data, display_img.cols, display_img.rows, static_cast<int>(display_img.step), QImage::Format_RGB888);
                label_img->setPixmap(QPixmap::fromImage(qimg).scaled(label_img->size(), Qt::IgnoreAspectRatio, Qt::SmoothTransformation));
            }

            return {true, room_index, left_right};
        }

        /**
         * @brief Helper to convert room indices to readable names.
         */
        static std::string room_name_from_index(int index)
        {
            switch(index)
            {
                case 0: return "RED";
                case 1: return "GREEN";
                case 2: return "BLUE";
                case 3: return "YELLOW";
                default: return "UNKNOWN";
            }
        }
    };
}