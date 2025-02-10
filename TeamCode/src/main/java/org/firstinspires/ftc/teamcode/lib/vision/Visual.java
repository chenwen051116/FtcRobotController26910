package org.firstinspires.ftc.teamcode.lib.vision;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.lang.reflect.Array;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.Arrays;


public class Visual {
    private Limelight3A limelight;
    private IMU imu;
    private Telemetry telemetry;

    private String snapScript = "#!/usr/bin/env python\n" +
            "import contextlib as __stickytape_contextlib\n" +
            "\n" +
            "@__stickytape_contextlib.contextmanager\n" +
            "def __stickytape_temporary_dir():\n" +
            "    import tempfile\n" +
            "    import shutil\n" +
            "    dir_path = tempfile.mkdtemp()\n" +
            "    try:\n" +
            "        yield dir_path\n" +
            "    finally:\n" +
            "        shutil.rmtree(dir_path)\n" +
            "\n" +
            "with __stickytape_temporary_dir() as __stickytape_working_dir:\n" +
            "    def __stickytape_write_module(path, contents):\n" +
            "        import os, os.path\n" +
            "\n" +
            "        def make_package(path):\n" +
            "            parts = path.split(\"/\")\n" +
            "            partial_path = __stickytape_working_dir\n" +
            "            for part in parts:\n" +
            "                partial_path = os.path.join(partial_path, part)\n" +
            "                if not os.path.exists(partial_path):\n" +
            "                    os.mkdir(partial_path)\n" +
            "                    with open(os.path.join(partial_path, \"__init__.py\"), \"wb\") as f:\n" +
            "                        f.write(b\"\\n\")\n" +
            "\n" +
            "        make_package(os.path.dirname(path))\n" +
            "\n" +
            "        full_path = os.path.join(__stickytape_working_dir, path)\n" +
            "        with open(full_path, \"wb\") as module_file:\n" +
            "            module_file.write(contents)\n" +
            "\n" +
            "    import sys as __stickytape_sys\n" +
            "    __stickytape_sys.path.insert(0, __stickytape_working_dir)\n" +
            "\n" +
            "    __stickytape_write_module('src/detector_contour.py', b'import cv2\\r\\nimport numpy as np\\r\\nfrom src.color_def import *\\r\\nfrom src.block import Block\\r\\nfrom typing import List\\r\\nfrom src.detectors import *\\r\\n\\r\\n\\r\\nclass ContourVizResults(TypedDict, total=False):\\r\\n    \"\"\"Defines the structure of the dictionary returned by visualize().\"\"\"\\r\\n    final_detection: np.ndarray\\r\\n    avg_HSV: np.ndarray\\r\\n    original: np.ndarray\\r\\n    preprocessed: np.ndarray\\r\\n    hsv_space: np.ndarray\\r\\n    combined_mask: np.ndarray\\r\\n\\r\\nclass ColorBlockDetectorContour(Detector):\\r\\n    \"\"\"\\r\\n    Detects color blocks by:\\r\\n      1) Preprocessing (brightness, blur)\\r\\n      2) Creating color masks\\r\\n      3) Finding contours\\r\\n      4) Computing mean & std(H, S, V) inside each contour\\r\\n    \"\"\"\\r\\n\\r\\n    def __init__(self, detecting_colors: List[Color]):\\r\\n        # Basic image processing parameters\\r\\n        self.blur_size = 35\\r\\n        self.brightness = 0\\r\\n        self.erode_iter = 7\\r\\n        self.dilate_iter = 6\\r\\n        self.min_contour_area = 1000\\r\\n        self.kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))\\r\\n\\r\\n        # Thresholds for std(H, S, V)\\r\\n        self.std_threshold_hsv = (3, 50, 50)\\r\\n\\r\\n        # Storage for debug images (intermediate steps)\\r\\n        self._debug_images : ContourVizResults = {}\\r\\n        self.detecting_colors = detecting_colors\\r\\n\\r\\n    def process_frame(self, frame: np.ndarray) -> List[Block]:\\r\\n        \"\"\"Main entry: preprocess and detect blocks, while saving debug images.\"\"\"\\r\\n        self._debug_images = {}\\r\\n\\r\\n        # Save original frame\\r\\n        self._debug_images[\\'original\\'] = frame.copy()\\r\\n\\r\\n        # 1) Preprocessing\\r\\n        preprocessed = self._preprocess(frame)\\r\\n        self._debug_images[\\'preprocessed\\'] = preprocessed.copy()\\r\\n\\r\\n        # 2) Convert to HSV\\r\\n        hsv = cv2.cvtColor(preprocessed, cv2.COLOR_BGR2HSV)\\r\\n        hsv_bgr_like = cv2.cvtColor(\\r\\n            hsv, cv2.COLOR_HSV2BGR)  # just for visualization\\r\\n        self._debug_images[\\'hsv_space\\'] = hsv_bgr_like\\r\\n\\r\\n        # 3) Detect blocks\\r\\n        blocks = self._detect_blocks(hsv)\\r\\n\\r\\n        return blocks\\r\\n\\r\\n    def get_debug_images(self) -> ContourVizResults:\\r\\n        \"\"\"Returns debug images for visualization.\"\"\"\\r\\n        return self._debug_images\\r\\n\\r\\n    def _preprocess(self, frame: np.ndarray) -> np.ndarray:\\r\\n        \"\"\"Adjust brightness and apply Gaussian blur.\"\"\"\\r\\n        frame = cv2.convertScaleAbs(frame, alpha=1, beta=self.brightness)\\r\\n        if self.blur_size > 0:\\r\\n            kernel_size = self.blur_size | 1\\r\\n            frame = cv2.GaussianBlur(frame, (kernel_size, kernel_size), 0)\\r\\n        return frame\\r\\n\\r\\n    def _detect_blocks(self, hsv: np.ndarray) -> List[Block]:\\r\\n        blocks = []\\r\\n        combined_mask = np.zeros(hsv.shape[:2], dtype=np.uint8)\\r\\n\\r\\n        for color_def in self.detecting_colors:\\r\\n            mask = self._create_color_mask(hsv, color_def)\\r\\n            combined_mask = cv2.bitwise_or(combined_mask, mask)\\r\\n\\r\\n            contours = self._find_contours(mask)\\r\\n            color_blocks = self._process_contours(contours, color_def, hsv)\\r\\n            blocks.extend(color_blocks)\\r\\n\\r\\n        combined_bgr = cv2.cvtColor(combined_mask, cv2.COLOR_GRAY2BGR)\\r\\n        self._debug_images[\\'combined_mask\\'] = combined_bgr\\r\\n\\r\\n        return blocks\\r\\n\\r\\n    def _create_color_mask(self, hsv: np.ndarray, color_def: Color) -> np.ndarray:\\r\\n        \"\"\"\\r\\n        Create a mask for each color definition, applying morphological operations.\\r\\n        The recognized areas retain their original colors (converted from HSV to BGR).\\r\\n        \\r\\n        Args:\\r\\n            hsv (np.ndarray): The HSV-converted image.\\r\\n            color_def (Color): The color definition (with HSV ranges and BGR info).\\r\\n        \\r\\n        Returns:\\r\\n            np.ndarray: A binary mask after morphological ops.\\r\\n        \"\"\"\\r\\n        # Step 1: Initialize an empty mask\\r\\n        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)\\r\\n\\r\\n        # Step 2: Apply color thresholds for each HSV range\\r\\n        for (lower, upper) in color_def.hsv_ranges:\\r\\n\\r\\n            # Apply threshold to get binary mask\\r\\n            tmp_mask = cv2.inRange(hsv, np.array(list(lower)), np.array(list(upper)))\\r\\n            mask = cv2.bitwise_or(mask, tmp_mask)\\r\\n\\r\\n        # Step 3: For debug: raw mask in color\\r\\n        hsv_bgr = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)\\r\\n        raw_mask_colored = cv2.bitwise_and(hsv_bgr, hsv_bgr, mask=mask)\\r\\n        debug_raw = f\"{color_def.name.lower()}_mask_raw\"\\r\\n        self._debug_images[debug_raw] = raw_mask_colored\\r\\n\\r\\n        # Step 4: Morphological operations (erode & dilate)\\r\\n        mask_morph = cv2.erode(mask, self.kernel, iterations=self.erode_iter)\\r\\n        mask_morph = cv2.dilate(mask_morph, self.kernel,\\r\\n                                iterations=self.dilate_iter)\\r\\n\\r\\n        # Step 5: For debug: morph mask in color\\r\\n        morph_mask_colored = cv2.bitwise_and(hsv_bgr, hsv_bgr, mask=mask_morph)\\r\\n        debug_morph = f\"{color_def.name.lower()}_mask_morph\"\\r\\n        self._debug_images[debug_morph] = morph_mask_colored\\r\\n\\r\\n        return mask_morph\\r\\n\\r\\n    def _find_contours(self, mask: np.ndarray) -> List[np.ndarray]:\\r\\n        \"\"\"Find external contours with area > min_contour_area.\"\"\"\\r\\n        contours, _ = cv2.findContours(\\r\\n            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)\\r\\n        return [c for c in contours if cv2.contourArea(c) > self.min_contour_area]\\r\\n\\r\\n    def _process_contours(self,\\r\\n                          contours: List[np.ndarray],\\r\\n                          color_def: Color,\\r\\n                          hsv: np.ndarray) -> List[Block]:\\r\\n        \"\"\"Compute mean & std(H, S, V) inside each contour and filter by thresholds.\"\"\"\\r\\n        blocks = []\\r\\n        for cnt in contours:\\r\\n            rect = cv2.minAreaRect(cnt)\\r\\n            (cx, cy), (w, h), angle = rect\\r\\n\\r\\n            # Normalize orientation\\r\\n            if w < h:\\r\\n                w, h = h, w\\r\\n                angle += 90\\r\\n\\r\\n            # Get bounding rect (for ROI)\\r\\n            x_min, y_min, w_int, h_int = cv2.boundingRect(cnt)\\r\\n            if w_int == 0 or h_int == 0:\\r\\n                continue\\r\\n\\r\\n            # Create local mask (contour inside bounding box)\\r\\n            contour_mask = np.zeros((h_int, w_int), dtype=np.uint8)\\r\\n            shifted_cnt = cnt - [x_min, y_min]\\r\\n            cv2.drawContours(contour_mask, [shifted_cnt], 0, (255,), -1)\\r\\n\\r\\n            # Extract HSV ROI\\r\\n            hsv_roi = hsv[y_min:y_min + h_int, x_min:x_min + w_int]\\r\\n            hsv_masked = cv2.bitwise_and(hsv_roi, hsv_roi, mask=contour_mask)\\r\\n\\r\\n            # Split channels and extract valid pixels\\r\\n            h_ch, s_ch, v_ch = cv2.split(hsv_masked)\\r\\n            h_valid = h_ch[contour_mask == 255].astype(np.float32)\\r\\n            s_valid = s_ch[contour_mask == 255].astype(np.float32)\\r\\n            v_valid = v_ch[contour_mask == 255].astype(np.float32)\\r\\n\\r\\n            if len(h_valid) == 0:\\r\\n                continue\\r\\n\\r\\n            # Compute mean & std for H, S, V\\r\\n            mean_h = float(np.mean(h_valid))\\r\\n            mean_s = float(np.mean(s_valid))\\r\\n            mean_v = float(np.mean(v_valid))\\r\\n\\r\\n            std_h = compute_hue_std_flip(h_valid, flip_threshold=90.0)\\r\\n            std_s = float(np.std(s_valid))\\r\\n            std_v = float(np.std(v_valid))\\r\\n\\r\\n            # Create a new Block\\r\\n            if std_h <= self.std_threshold_hsv[0] and \\\\\\r\\n               std_s <= self.std_threshold_hsv[1] and \\\\\\r\\n               std_v <= self.std_threshold_hsv[2]:\\r\\n\\r\\n                block = Block(\\r\\n                    center=(cx, cy),\\r\\n                    size=(w, h),\\r\\n                    angle=angle,\\r\\n                    color=color_def,\\r\\n                    color_std=(std_h, std_s, std_v),\\r\\n                    mean_hsv=(mean_h, mean_s, mean_v),\\r\\n                    # store the original contour (absolute coordinates)\\r\\n                    contour=cnt\\r\\n                )\\r\\n                blocks.append(block)\\r\\n        return blocks\\r\\n')\n" +
            "    __stickytape_write_module('src/color_def.py', b'import cv2\\r\\nimport numpy as np\\r\\nfrom dataclasses import dataclass, field\\r\\nfrom typing import List, Tuple\\r\\n\\r\\n# ---------- Data Classes ----------\\r\\n\\r\\nhsv_t = Tuple[int, int, int]\\r\\nbgr_t = Tuple[int, int, int]\\r\\n\\r\\n\\r\\n@dataclass\\r\\nclass Color:\\r\\n    \"\"\"Stores color name, HSV ranges, and BGR values for drawing.\"\"\"\\r\\n    name: str\\r\\n    hsv_ranges: List[Tuple[hsv_t, hsv_t]]\\r\\n    bgr: bgr_t\\r\\n\\r\\n\\r\\n# ---------- Color Block Detector ----------\\r\\n\\r\\ndef compute_hue_std_flip(h_array: np.ndarray, flip_threshold: float = 90.0) -> float:\\r\\n    # Ensure float\\r\\n    h_float = h_array.astype(np.float32)\\r\\n\\r\\n    # 1) Direct std\\r\\n    std1 = np.std(h_float)\\r\\n\\r\\n    # 2) Flip\\r\\n    shifted = h_float.copy()\\r\\n    mask = (shifted < flip_threshold)\\r\\n    shifted[mask] += 180.0\\r\\n    std2 = np.std(shifted)\\r\\n\\r\\n    return float(min(std1, std2))\\r\\n\\r\\n\\r\\nRED_R9000P = Color(\\r\\n    name=\"RED_R9000P\",\\r\\n    hsv_ranges=[\\r\\n        ((0, 70, 50), (3, 160, 225)),\\r\\n        ((165, 70, 50), (180, 160, 225)),\\r\\n    ],\\r\\n    bgr=(0, 0, 255)\\r\\n)\\r\\n\\r\\nBLUE_R9000P = Color(\\r\\n    name=\"BLUE_R9000P\",\\r\\n    hsv_ranges=[\\r\\n        ((110, 80, 70), (125, 180, 230)),\\r\\n    ],\\r\\n    bgr=(255, 0, 0)\\r\\n)\\r\\n\\r\\nYELLOW_R9000P = Color(\\r\\n    name=\"YELLOW_R9000P\",\\r\\n    hsv_ranges=[\\r\\n        ((17, 60, 140), (32, 125, 255)),\\r\\n    ],\\r\\n    bgr=(0, 255, 255)\\r\\n)\\r\\n\\r\\nCOLOR_DEF_R9000P = [RED_R9000P, BLUE_R9000P, YELLOW_R9000P]\\r\\n\\r\\nRED_LL = Color(\\r\\n    name=\"RED_LL\",\\r\\n    hsv_ranges=[\\r\\n        ((0, 190, 90), (5, 255, 250)),\\r\\n        ((160, 190, 90), (180, 255, 250)),\\r\\n    ],\\r\\n    bgr=(0, 0, 255)\\r\\n)\\r\\n\\r\\nYELLOW_LL = Color(\\r\\n    name=\"YELLOW_LL\",\\r\\n    hsv_ranges=[\\r\\n        ((15, 130, 160), (35, 255, 250)),\\r\\n    ],\\r\\n    bgr=(0, 255, 255)\\r\\n)\\r\\n\\r\\nBLUE_LL = Color(\\r\\n    name=\"BLUE_LL\",\\r\\n    hsv_ranges=[\\r\\n        ((100, 220, 70), (125, 255, 230)),\\r\\n    ],\\r\\n    bgr=(255, 0, 0)\\r\\n)\\r\\n\\r\\nCOLOR_DEF_LL = [RED_LL, YELLOW_LL, BLUE_LL]')\n" +
            "    __stickytape_write_module('src/block.py', b'import numpy as np\\r\\nfrom dataclasses import dataclass, field\\r\\nfrom .color_def import Color\\r\\nfrom typing import List, Tuple\\r\\n\\r\\n@dataclass\\r\\nclass Block:\\r\\n    \"\"\"Represents a detected color block with position, size, angle, color info, and HSV stats.\"\"\"\\r\\n    center: Tuple[float, float]\\r\\n    size: Tuple[float, float]\\r\\n    angle: float\\r\\n    color: Color\\r\\n    color_std: Tuple[float, float, float] = (0.0, 0.0, 0.0)\\r\\n    mean_hsv: Tuple[float, float, float] = (0.0, 0.0, 0.0)\\r\\n    # store the absolute contour for visualization\\r\\n    contour: np.ndarray = field(default_factory=lambda: np.array([]))\\r\\n')\n" +
            "    __stickytape_write_module('src/detectors.py', b'from abc import ABC, abstractmethod\\r\\nfrom src.block import Block\\r\\nfrom typing import List\\r\\nimport numpy as np\\r\\nfrom typing import TypedDict, Dict \\r\\n\\r\\nVizResults = Dict[str, np.ndarray]\\r\\n\\r\\nclass Detector(ABC):\\r\\n    def __init__(self):\\r\\n        pass\\r\\n    @abstractmethod\\r\\n    def process_frame(self, frame: np.ndarray) -> List[Block]:\\r\\n        pass\\r\\n    \\r\\n    @abstractmethod\\r\\n    def get_debug_images(self) -> Dict[str, np.ndarray]: \\r\\n        pass\\r\\n    \\r\\n    # def _preprocess(self, frame: np.ndarray) -> np.ndarray: \\r\\n    #     # Example preprocessing: convert frame to grayscale\\r\\n    #     pass')\n" +
            "    __stickytape_write_module('src/detector_watershed.py', b'import cv2\\r\\nimport numpy as np\\r\\nfrom typing import List, TypedDict\\r\\nfrom src.detectors import Detector\\r\\nfrom src.color_def import Color, compute_hue_std_flip\\r\\nfrom src.block import Block\\r\\n\\r\\n\\r\\nclass WatershedVizResults(TypedDict, total=False):\\r\\n    \"\"\"Defines the structure of the dictionary returned by visualize().\"\"\"\\r\\n    original: np.ndarray\\r\\n    preprocessed: np.ndarray\\r\\n    combined_mask: np.ndarray\\r\\n    sure_bg: np.ndarray\\r\\n    sure_fg: np.ndarray\\r\\n    unknown: np.ndarray\\r\\n    final_detection: np.ndarray\\r\\n\\r\\nclass ColorBlockDetectorWatershed(Detector):\\r\\n    \"\"\"\\r\\n    Detect color blocks by:\\r\\n      1) Preprocessing (brightness, blur)\\r\\n      2) Combining all color ranges to create a single mask\\r\\n      3) Using distance transform + watershed to segment\\r\\n      4) Extract each labeled region, build Block\\r\\n    \"\"\"\\r\\n\\r\\n    def __init__(self, detecting_colors):\\r\\n        # Basic image processing parameters\\r\\n        self.blur_size = 50\\r\\n        self.brightness = 0\\r\\n        self.mask_erode_iter = 2\\r\\n        self.mask_dilate_iter = 2\\r\\n        self.min_area = 1000  # minimum region area\\r\\n        self.kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))\\r\\n        self.sure_fg_min_dis_ratio = 0.7\\r\\n        self.detecting_colors = detecting_colors\\r\\n        # For optional HSV std check:\\r\\n        self.std_threshold_hsv = (3, 50, 50)\\r\\n\\r\\n        # Storage for debug images (intermediate steps)\\r\\n        self._debug_images: WatershedVizResults = {}\\r\\n\\r\\n    def process_frame(self, frame: np.ndarray) -> List[Block]:\\r\\n        \"\"\"\\r\\n        Main entry: preprocess, do watershed, return Blocks\\r\\n        \"\"\"\\r\\n        # Clear old debug images\\r\\n        self._debug_images = {}\\r\\n\\r\\n        # Save original\\r\\n        self._debug_images[\\'original\\'] = frame.copy()\\r\\n\\r\\n        # 1) Preprocess\\r\\n        preprocessed = self._preprocess(frame)\\r\\n        self._debug_images[\\'preprocessed\\'] = preprocessed.copy()\\r\\n\\r\\n        # 2) Convert to HSV\\r\\n        hsv = cv2.cvtColor(preprocessed, cv2.COLOR_BGR2HSV)\\r\\n\\r\\n        # 3) Detect blocks via watershed\\r\\n        blocks = self._detect_blocks(hsv, frame)\\r\\n\\r\\n        return blocks\\r\\n\\r\\n    def get_debug_images(self) -> WatershedVizResults:\\r\\n        return self._debug_images\\r\\n\\r\\n    def _preprocess(self, frame: np.ndarray) -> np.ndarray:\\r\\n        \"\"\"Adjust brightness and apply Gaussian blur.\"\"\"\\r\\n        # Adjust brightness\\r\\n        frame = cv2.convertScaleAbs(frame, alpha=1, beta=self.brightness)\\r\\n        # Apply blur\\r\\n        if self.blur_size > 0:\\r\\n            kernel_size = self.blur_size | 1  # ensure odd\\r\\n            frame = cv2.GaussianBlur(frame, (kernel_size, kernel_size), 0)\\r\\n        return frame\\r\\n\\r\\n    def _detect_blocks(self, hsv: np.ndarray, frame_bgr: np.ndarray) -> List[Block]:\\r\\n        \"\"\"\\r\\n        Perform multiple watershed passes: one per color in self.detecting_colors.\\r\\n\\r\\n        Workflow for each color:\\r\\n        1) Create a color-specific mask.\\r\\n        2) Morphological cleaning.\\r\\n        3) _watershed_segment => obtains markers.\\r\\n        4) For each labeled region:\\r\\n            - Compute shape (minAreaRect)\\r\\n            - (Optionally) compute HSV stats\\r\\n            - Assign the color (since this pass is specific to one color)\\r\\n            - Construct a Block\\r\\n        Accumulate all blocks from each color and return.\\r\\n        \"\"\"\\r\\n\\r\\n        all_blocks: List[Block] = []\\r\\n        # For debugging, we can store intermediate images per color\\r\\n        # e.g. \"blue_mask\", \"blue_sure_bg\", etc.\\r\\n        self._debug_images = {}\\r\\n\\r\\n        # Save original just once\\r\\n        self._debug_images[\\'original\\'] = frame_bgr.copy()\\r\\n\\r\\n        # 1) Preprocess (optional brightness, blur)\\r\\n        preprocessed = self._preprocess(frame_bgr)\\r\\n        self._debug_images[\\'preprocessed\\'] = preprocessed.copy()\\r\\n\\r\\n        # 2) Convert to HSV\\r\\n        hsv_img = cv2.cvtColor(preprocessed, cv2.COLOR_BGR2HSV)\\r\\n\\r\\n        # For each color, run its own Watershed pass\\r\\n        for color_def in self.detecting_colors:\\r\\n            # A) Create color-specific mask\\r\\n            color_mask = self._create_color_mask(hsv_img, color_def)\\r\\n            # Optional morphological ops if not already inside _create_color_mask\\r\\n            color_mask = cv2.morphologyEx(\\r\\n                color_mask, cv2.MORPH_OPEN, self.kernel, iterations=2)\\r\\n            color_mask = cv2.morphologyEx(\\r\\n                color_mask, cv2.MORPH_CLOSE, self.kernel, iterations=2)\\r\\n\\r\\n            # Save debug mask\\r\\n            mask_bgr = cv2.cvtColor(color_mask, cv2.COLOR_GRAY2BGR)\\r\\n            self._debug_images[f\"{color_def.name}_mask\"] = mask_bgr\\r\\n\\r\\n            # B) Watershed on this color mask\\r\\n            markers, num_labels, sure_bg, sure_fg, unknown = self._watershed_segment(\\r\\n                color_mask, frame_bgr)\\r\\n\\r\\n            # Debug images for sure_bg, sure_fg, unknown\\r\\n            self._debug_images[f\"{color_def.name}_sure_bg\"] = cv2.cvtColor(\\r\\n                sure_bg, cv2.COLOR_GRAY2BGR)\\r\\n            self._debug_images[f\"{color_def.name}_sure_fg\"] = cv2.cvtColor(\\r\\n                sure_fg, cv2.COLOR_GRAY2BGR)\\r\\n\\r\\n            unknown_bgr = np.zeros_like(frame_bgr)\\r\\n            unknown_bgr[unknown == 255] = (128, 128, 128)\\r\\n            self._debug_images[f\"{color_def.name}_unknown\"] = unknown_bgr\\r\\n\\r\\n            # C) Build blocks from each label\\r\\n            h, w = color_mask.shape[:2]\\r\\n            for lbl in range(2, num_labels + 1):\\r\\n                region_mask = (markers == lbl)\\r\\n                region_area = np.count_nonzero(region_mask)\\r\\n                if region_area < self.min_area:\\r\\n                    continue\\r\\n\\r\\n                # extract coordinates\\r\\n                region_pts = np.argwhere(region_mask)\\r\\n                if len(region_pts) == 0:\\r\\n                    continue\\r\\n\\r\\n                # minAreaRect\\r\\n                coords_xy = np.fliplr(region_pts).astype(np.float32)  # (x, y)\\r\\n                coords_xy_list = coords_xy[:, np.newaxis, :]\\r\\n                rect = cv2.minAreaRect(coords_xy_list)\\r\\n                (rx, ry), (rw, rh), angle = rect\\r\\n                if rw < rh:\\r\\n                    rw, rh = rh, rw\\r\\n                    angle += 90\\r\\n\\r\\n                # optional: compute HSV stats in that region\\r\\n                label_mask = np.zeros((h, w), dtype=np.uint8)\\r\\n                label_mask[region_mask] = 255\\r\\n\\r\\n                hsv_region = cv2.bitwise_and(hsv_img, hsv_img, mask=label_mask)\\r\\n\\r\\n                # For mean\\r\\n                mean_hsv = cv2.mean(hsv_region, mask=label_mask)[:3]\\r\\n\\r\\n                # For std\\r\\n                h_ch, s_ch, v_ch = cv2.split(hsv_region)\\r\\n                h_valid = h_ch[label_mask == 255].astype(np.float32)\\r\\n                s_valid = s_ch[label_mask == 255].astype(np.float32)\\r\\n                v_valid = v_ch[label_mask == 255].astype(np.float32)\\r\\n\\r\\n                std_h = compute_hue_std_flip(h_valid)\\r\\n                std_s = float(np.std(s_valid))\\r\\n                std_v = float(np.std(v_valid))\\r\\n\\r\\n                # We can retrieve the contour for accurate shape\\r\\n                contours, _ = cv2.findContours(\\r\\n                    label_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)\\r\\n                if not contours:\\r\\n                    continue\\r\\n                largest_contour = max(contours, key=cv2.contourArea)\\r\\n\\r\\n                # create Block\\r\\n                block = Block(\\r\\n                    center=(rx, ry),\\r\\n                    size=(rw, rh),\\r\\n                    angle=angle,\\r\\n                    color=color_def,  # since this pass is specifically for color_def\\r\\n                    mean_hsv=(mean_hsv[0], mean_hsv[1], mean_hsv[2]),\\r\\n                    color_std=(std_h, std_s, std_v),\\r\\n                    contour=largest_contour\\r\\n                )\\r\\n                all_blocks.append(block)\\r\\n\\r\\n        return all_blocks\\r\\n\\r\\n\\r\\n\\r\\n    def _watershed_segment(self, mask: np.ndarray, frame_bgr: np.ndarray):\\r\\n        \"\"\"\\r\\n        Given a binary mask (255=FG, 0=BG), apply morphological + distance transform,\\r\\n        compute sure_fg, sure_bg, unknown, then run cv2.watershed on frame_bgr.\\r\\n        \\r\\n        Returns:\\r\\n          markers, num_labels, sure_bg, sure_fg, unknown\\r\\n        \"\"\"\\r\\n        # Step 1: sure_bg by dilate\\r\\n        sure_bg = cv2.dilate(mask, self.kernel, iterations=7)\\r\\n\\r\\n        # Step 2: distance transform => sure_fg\\r\\n        dist_transform = cv2.distanceTransform(mask, cv2.DIST_L2, 5)\\r\\n        max_val = dist_transform.max()\\r\\n        _, sure_fg = cv2.threshold(dist_transform, self.sure_fg_min_dis_ratio * max_val, 255, 0)\\r\\n        # set to 255 if larger than threshold, else 0\\r\\n        sure_fg = sure_fg.astype(np.uint8)\\r\\n\\r\\n        # Step 3: unknown = sure_bg - sure_fg\\r\\n        unknown = cv2.subtract(sure_bg, sure_fg)\\r\\n\\r\\n        # Step 4: connectedComponents => markers\\r\\n        num_labels, markers = cv2.connectedComponents(sure_fg)\\r\\n        # in connected component, 0 is for background\\r\\n        # but we want background to be 1, so add 1 to all labels\\r\\n        markers += 1 \\r\\n        markers[unknown == 255] = 0\\r\\n\\r\\n        # Step 5: watershed\\r\\n        # 1: background \\r\\n        # 0: unknown\\r\\n        # other: different regions\\r\\n        cv2.watershed(frame_bgr, markers)\\r\\n\\r\\n        return markers, num_labels, sure_bg, sure_fg, unknown\\r\\n\\r\\n    def _create_color_mask(self, hsv: np.ndarray, color_def: Color) -> np.ndarray:\\r\\n        \"\"\"\\r\\n        Create a mask for a single color definition, with morphological ops.\\r\\n        \"\"\"\\r\\n        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)\\r\\n\\r\\n        for (lower, upper) in color_def.hsv_ranges:\\r\\n            lower_np = np.array(lower, dtype=np.uint8)\\r\\n            upper_np = np.array(upper, dtype=np.uint8)\\r\\n            tmp_mask = cv2.inRange(hsv, lower_np, upper_np)\\r\\n            mask = cv2.bitwise_or(mask, tmp_mask)\\r\\n\\r\\n        # Morphology\\r\\n        mask_morph = cv2.erode(\\r\\n            mask, self.kernel, iterations=self.mask_erode_iter)\\r\\n        mask_morph = cv2.dilate(mask_morph, self.kernel,\\r\\n                                iterations=self.mask_dilate_iter)\\r\\n        return mask_morph\\r\\n')\n" +
            "    __stickytape_write_module('src/detector_meanshift.py', b'import cv2\\r\\nimport numpy as np\\r\\nfrom typing import List, TypedDict\\r\\nfrom src.detectors import Detector\\r\\nfrom src.color_def import Color, compute_hue_std_flip\\r\\nfrom src.block import Block\\r\\n\\r\\n\\r\\nclass MeanShiftVizResults(TypedDict, total=False):\\r\\n    \"\"\"Typed dictionary structure for debug images.\"\"\"\\r\\n    original: np.ndarray\\r\\n    preprocessed: np.ndarray\\r\\n    mean_shift_filtered: np.ndarray\\r\\n    mask_after_threshold: np.ndarray\\r\\n    final_detection: np.ndarray\\r\\n\\r\\n\\r\\nclass ColorBlockDetectorMeanShift(Detector):\\r\\n    \"\"\"\\r\\n    Detect color blocks by:\\r\\n      1) Preprocessing (brightness, blur)\\r\\n      2) Mean Shift filtering to smooth colors\\r\\n      3) For each color, threshold => morphological cleaning => findContours\\r\\n      4) Build Block from each contour\\r\\n    \"\"\"\\r\\n\\r\\n    def __init__(self, detecting_colors: List[Color]):\\r\\n        # Basic image processing parameters\\r\\n        self.blur_size = 0\\r\\n        self.brightness = 0\\r\\n        self.mask_erode_iter = 0\\r\\n        self.mask_dilate_iter = 0\\r\\n        self.min_area = 1000  # Minimum region area\\r\\n        self.spatial_radius = 30  # For pyrMeanShiftFiltering\\r\\n        self.color_radius = 50\\r\\n        self.max_level = 1  # for iterative level in meanShift\\r\\n        self.kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))\\r\\n\\r\\n        # Optional HSV std check\\r\\n        self.std_threshold_hsv = (3, 50, 50)\\r\\n        self.detecting_colors = detecting_colors\\r\\n\\r\\n        # Storage for debug images\\r\\n        self._debug_images: MeanShiftVizResults = {}\\r\\n\\r\\n    def process_frame(self, frame: np.ndarray) -> List[Block]:\\r\\n        \"\"\"\\r\\n        Main entry: preprocess, do mean shift filtering, color threshold, build Blocks\\r\\n        \"\"\"\\r\\n        # Clear old debug images\\r\\n        self._debug_images = {}\\r\\n\\r\\n        # Save original\\r\\n        self._debug_images[\\'original\\'] = frame.copy()\\r\\n\\r\\n        # 1) Preprocess (e.g. brightness, blur)\\r\\n        preprocessed = self._preprocess(frame)\\r\\n        self._debug_images[\\'preprocessed\\'] = preprocessed.copy()\\r\\n\\r\\n        st_time = cv2.getTickCount()\\r\\n        # 2) Mean Shift filtering\\r\\n        mean_shift_bgr = cv2.pyrMeanShiftFiltering(\\r\\n            preprocessed,\\r\\n            sp=self.spatial_radius,\\r\\n            sr=self.color_radius,\\r\\n            maxLevel=self.max_level\\r\\n        )\\r\\n        ed_time = cv2.getTickCount()\\r\\n        print(\"Mean Shift Time:\", (ed_time - st_time) / cv2.getTickFrequency())\\r\\n        self._debug_images[\\'mean_shift_filtered\\'] = mean_shift_bgr.copy()\\r\\n\\r\\n        # Convert to HSV for color thresholding\\r\\n        mean_shift_hsv = cv2.cvtColor(mean_shift_bgr, cv2.COLOR_BGR2HSV)\\r\\n\\r\\n        # 3) For each color, threshold => morphological => findContours => create Blocks\\r\\n        blocks: List[Block] = []\\r\\n        for color_def in self.detecting_colors:\\r\\n            color_blocks = self._detect_blocks_for_color(\\r\\n                mean_shift_hsv, mean_shift_bgr, color_def)\\r\\n            blocks.extend(color_blocks)\\r\\n\\r\\n        return blocks\\r\\n\\r\\n    def get_debug_images(self) -> MeanShiftVizResults:\\r\\n        return self._debug_images\\r\\n\\r\\n    def _preprocess(self, frame: np.ndarray) -> np.ndarray:\\r\\n        \"\"\"Adjust brightness and optionally blur.\"\"\"\\r\\n        # Adjust brightness\\r\\n        frame = cv2.convertScaleAbs(frame, alpha=1, beta=self.brightness)\\r\\n        # Gaussian blur if needed\\r\\n        if self.blur_size > 0:\\r\\n            kernel_size = self.blur_size | 1  # ensure odd\\r\\n            frame = cv2.GaussianBlur(frame, (kernel_size, kernel_size), 0)\\r\\n        return frame\\r\\n\\r\\n    def _detect_blocks_for_color(self, hsv_img: np.ndarray, bgr_img: np.ndarray, color_def: Color) -> List[Block]:\\r\\n        \"\"\"\\r\\n        For a single color definition, threshold => morphological cleaning => findContours => build Blocks\\r\\n        \"\"\"\\r\\n        blocks: List[Block] = []\\r\\n\\r\\n        # A) Combine all HSV ranges for this color\\r\\n        mask = np.zeros(hsv_img.shape[:2], dtype=np.uint8)\\r\\n        for (lower, upper) in color_def.hsv_ranges:\\r\\n            lower_np = np.array(lower, dtype=np.uint8)\\r\\n            upper_np = np.array(upper, dtype=np.uint8)\\r\\n            tmp_mask = cv2.inRange(hsv_img, lower_np, upper_np)\\r\\n            mask = cv2.bitwise_or(mask, tmp_mask)\\r\\n\\r\\n        # B) Morphological cleaning\\r\\n        if self.mask_erode_iter > 0:\\r\\n            mask = cv2.erode(mask, self.kernel, iterations=self.mask_erode_iter)\\r\\n        if self.mask_dilate_iter > 0:\\r\\n            mask = cv2.dilate(mask, self.kernel, iterations=self.mask_dilate_iter)\\r\\n\\r\\n        # Optional: store debug for each color\\r\\n        mask_debug = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)\\r\\n        self._debug_images[f\"{color_def.name}_mask_after_threshold\"] = mask_debug\\r\\n\\r\\n        # C) Find contours in the mask\\r\\n        contours, _ = cv2.findContours(\\r\\n            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)\\r\\n\\r\\n        h, w = mask.shape[:2]\\r\\n        for cnt in contours:\\r\\n            region_area = cv2.contourArea(cnt)\\r\\n            if region_area < self.min_area:\\r\\n                continue\\r\\n\\r\\n            # minAreaRect\\r\\n            rect = cv2.minAreaRect(cnt)\\r\\n            (rx, ry), (rw, rh), angle = rect\\r\\n            if rw < rh:\\r\\n                rw, rh = rh, rw\\r\\n                angle += 90\\r\\n\\r\\n            # Build a mask for this contour to compute stats\\r\\n            contour_mask = np.zeros((h, w), dtype=np.uint8)\\r\\n            cv2.drawContours(contour_mask, [cnt], 0, 255, -1)\\r\\n\\r\\n            # Extract HSV region\\r\\n            hsv_region = cv2.bitwise_and(hsv_img, hsv_img, mask=contour_mask)\\r\\n\\r\\n            # Mean HSV\\r\\n            mean_hsv = cv2.mean(hsv_region, mask=contour_mask)[:3]\\r\\n\\r\\n            # Std HSV\\r\\n            h_ch, s_ch, v_ch = cv2.split(hsv_region)\\r\\n            h_valid = h_ch[contour_mask == 255].astype(np.float32)\\r\\n            s_valid = s_ch[contour_mask == 255].astype(np.float32)\\r\\n            v_valid = v_ch[contour_mask == 255].astype(np.float32)\\r\\n\\r\\n            std_h = compute_hue_std_flip(h_valid)\\r\\n            std_s = float(np.std(s_valid))\\r\\n            std_v = float(np.std(v_valid))\\r\\n\\r\\n            # Create Block\\r\\n            block = Block(\\r\\n                center=(rx, ry),\\r\\n                size=(rw, rh),\\r\\n                angle=angle,\\r\\n                color=color_def,\\r\\n                mean_hsv=(mean_hsv[0], mean_hsv[1], mean_hsv[2]),\\r\\n                color_std=(std_h, std_s, std_v),\\r\\n                contour=cnt\\r\\n            )\\r\\n            blocks.append(block)\\r\\n\\r\\n        return blocks\\r\\n')\n" +
            "    __stickytape_write_module('src/visualizer.py', b'import cv2\\r\\nimport numpy as np\\r\\nfrom typing import List, Dict, Optional, TypedDict\\r\\nfrom src.color_def import *\\r\\nfrom src.block import Block\\r\\nfrom src.detectors import VizResults\\r\\n\\r\\nclass BlockVisualizer:\\r\\n    \"\"\"\\r\\n    Manages two modes:\\r\\n      - Mode 0: Final detection only (single window)\\r\\n      - Mode 1: Debug images (multiple windows, intermediate steps) + one additional window\\r\\n                showing each block region filled with its average HSV color.\\r\\n\\r\\n    Parameters:\\r\\n        show (bool): Whether to display images using OpenCV\\'s cv2.imshow(). If False, functions return images.\\r\\n    \"\"\"\\r\\n\\r\\n    def __init__(self, show: bool = True):\\r\\n        self.mode = 0\\r\\n        self.prev_mode = 0  # Force initialization\\r\\n        self.main_window = \"Block Detection\"\\r\\n        self.show = show  # Whether to display images or return them\\r\\n\\r\\n    def toggle_mode(self):\\r\\n        \"\"\"Switch between mode 0 and mode 1.\"\"\"\\r\\n        self.mode = (self.mode + 1) % 2\\r\\n\\r\\n    def visualize(self, frame: np.ndarray, blocks: List[Block], debug_images: VizResults) -> Optional[VizResults]:\\r\\n        \"\"\"\\r\\n        Decide which visualization to show based on mode.\\r\\n        Only destroy/recreate windows if the mode changed.\\r\\n\\r\\n        Parameters:\\r\\n            frame (np.ndarray): The original frame.\\r\\n            blocks (List[Block]): List of detected blocks.\\r\\n            debug_images (Dict[str, np.ndarray]): Dictionary of debug images.\\r\\n\\r\\n        Returns:\\r\\n            Optional[Dict[str, np.ndarray]]: If `self.show=False`, returns a dictionary of images.\\r\\n        \"\"\"\\r\\n        if self.mode != self.prev_mode:\\r\\n            cv2.destroyAllWindows()\\r\\n            self.prev_mode = self.mode\\r\\n\\r\\n        results : VizResults= {}\\r\\n\\r\\n        if self.mode == 0:\\r\\n            final_result = self.show_final_result(frame, blocks)\\r\\n            if not self.show:\\r\\n                results[\\'final_detection\\'] = final_result\\r\\n        else:\\r\\n            debug_outputs = self.show_debug_images(debug_images)\\r\\n            avg_hsv_image = self.show_avg_hsv_fill(frame, blocks)\\r\\n\\r\\n            if not self.show:\\r\\n                for name, img in debug_outputs.items(): results[name] = img\\r\\n                results[\\'avg_HSV\\'] = avg_hsv_image\\r\\n\\r\\n        return results if not self.show else None\\r\\n\\r\\n    def show_final_result(self, frame: np.ndarray, blocks: List[Block]) -> np.ndarray:\\r\\n        \"\"\"Draw bounding boxes and put text for each block.\"\"\"\\r\\n        output = frame.copy()\\r\\n        for block in blocks:\\r\\n            box = cv2.boxPoints(\\r\\n                (block.center, block.size, block.angle))  # type: ignore\\r\\n            box = np.intp(box)\\r\\n            cv2.drawContours(output, [box], 0, block.color.bgr, 2)  # type: ignore\\r\\n\\r\\n            # Text lines with extra info: avgH, avgS, avgV\\r\\n            lines = [\\r\\n                f\"{block.color.name}: {block.angle:.1f} deg\",\\r\\n                f\"stdHSV=({block.color_std[0]:.1f}, {block.color_std[1]:.1f}, {block.color_std[2]:.1f})\",\\r\\n                f\"avgHSV=({block.mean_hsv[0]:.1f}, {block.mean_hsv[1]:.1f},{block.mean_hsv[2]:.1f})\"\\r\\n            ]\\r\\n            x0, y0 = int(block.center[0]), int(block.center[1])\\r\\n            for i, line in enumerate(lines):\\r\\n                offset_y = i * 15\\r\\n                cv2.putText(\\r\\n                    output,\\r\\n                    line,\\r\\n                    (x0, y0 + offset_y),\\r\\n                    cv2.FONT_HERSHEY_SIMPLEX,\\r\\n                    0.5,\\r\\n                    (255, 255, 255),\\r\\n                    1\\r\\n                )\\r\\n\\r\\n        cv2.putText(\\r\\n            output,\\r\\n            \"Final Detection\",\\r\\n            (10, 30),\\r\\n            cv2.FONT_HERSHEY_SIMPLEX,\\r\\n            1.0,\\r\\n            (0, 255, 0),\\r\\n            2\\r\\n        )\\r\\n\\r\\n        if self.show:\\r\\n            cv2.imshow(self.main_window, output)\\r\\n        return output\\r\\n\\r\\n    def show_debug_images(self, debug_images: VizResults) -> Dict[str, np.ndarray]:\\r\\n        \"\"\"Display intermediate debug images, or return them if `self.show` is False.\"\"\"\\r\\n        results = {}\\r\\n        for name, img in debug_images.items():\\r\\n            if not isinstance(img, np.ndarray):\\r\\n                continue\\r\\n            display = img.copy()\\r\\n            cv2.putText(display, name, (10, 30), cv2.FONT_HERSHEY_SIMPLEX,\\r\\n                        1.0, (0, 255, 0), 2)\\r\\n\\r\\n            if self.show:\\r\\n                cv2.imshow(name, display)\\r\\n            else:\\r\\n                results[name] = display\\r\\n\\r\\n        return results\\r\\n\\r\\n    def show_avg_hsv_fill(self, frame: np.ndarray, blocks: List[Block]) -> np.ndarray:\\r\\n        \"\"\"\\r\\n        Create a black canvas the same size as \\'frame\\', then fill each block\\'s contour\\r\\n        with the block\\'s average HSV color (converted to BGR). Show this in a new window\\r\\n        or return the processed image.\\r\\n        \"\"\"\\r\\n        canvas = np.zeros_like(frame)  # black canvas\\r\\n        for block in blocks:\\r\\n            # Convert mean_hsv -> BGR\\r\\n            hsv_pixel = np.uint8(\\r\\n                [[[block.mean_hsv[0], block.mean_hsv[1], block.mean_hsv[2]]]])  # type: ignore\\r\\n            bgr_pixel = cv2.cvtColor(\\r\\n                hsv_pixel, cv2.COLOR_HSV2BGR)  # type: ignore\\r\\n            avg_color = (int(bgr_pixel[0, 0, 0]), int(\\r\\n                bgr_pixel[0, 0, 1]), int(bgr_pixel[0, 0, 2]))\\r\\n\\r\\n            # Fill the contour with this color\\r\\n            cv2.drawContours(canvas, [block.contour], 0, avg_color, -1)\\r\\n\\r\\n        cv2.putText(\\r\\n            canvas,\\r\\n            \"Blocks filled w/ average HSV\",\\r\\n            (10, 30),\\r\\n            cv2.FONT_HERSHEY_SIMPLEX,\\r\\n            1.0,\\r\\n            (255, 255, 255),\\r\\n            2\\r\\n        )\\r\\n\\r\\n        if self.show:\\r\\n            cv2.imshow(\"Avg HSV Debug\", canvas)\\r\\n        return canvas\\r\\n')\n" +
            "    __stickytape_write_module('src/utils/serializer.py', b'from src.block import Block\\r\\nfrom ctypes import Structure, c_int16, c_int32, c_float\\r\\nimport struct\\r\\nfrom typing import Tuple, Optional, List\\r\\nfrom enum import Enum\\r\\n\\r\\nclass SerializedColor(Enum):\\r\\n    YELLOW = 1\\r\\n    RED = 2\\r\\n    BLUE = 4\\r\\n\\r\\ndef get_serialized_color(color_name: str) -> Optional[SerializedColor]:\\r\\n    \"\"\"Maps detected color names to their corresponding SerializedColor enum based on keyword matching.\"\"\"\\r\\n    color_name_upper = color_name.upper()\\r\\n\\r\\n    for enum_member in SerializedColor:\\r\\n        if enum_member.name in color_name_upper:\\r\\n            return enum_member\\r\\n\\r\\n    return None  # Return None if no match is found\\r\\n\\r\\nclass SerializedBlock(Structure):\\r\\n    _pack_ = 1  # No padding\\r\\n    _fields_ = [\\r\\n        (\"center_x\", c_int16),\\r\\n        (\"center_y\", c_int16),\\r\\n        (\"width\", c_int16),\\r\\n        (\"height\", c_int16),\\r\\n        (\"angle\", c_float),\\r\\n        (\"color\", c_int32)\\r\\n    ]\\r\\n\\r\\n    def __str__(self) -> str:\\r\\n        return f\"SerializedBlock(center=({self.center_x}, {self.center_y}), size=({self.width}, {self.height}), angle={self.angle}, color={self.color})\"\\r\\n\\r\\n    def pack_to_floats(self) -> Tuple[float, float, float, float]:\\r\\n        \"\"\"Packs the structure into four floats.\"\"\"\\r\\n        raw_bytes = bytes(self)\\r\\n        assert len(raw_bytes) == 16, f\"Expected 16 bytes, got {len(raw_bytes)}\"\\r\\n        f1, f2, f3, f4 = struct.unpack(\\'<4f\\', raw_bytes)\\r\\n        return f1, f2, f3, f4\\r\\n    \\r\\n    def serialize_to_float(self) -> List[float]:\\r\\n        \"\"\"Directly serialize the 6 quantities to floats\"\"\"\\r\\n        return [float(num) for num in [self.center_x, self.center_y, self.width, self.height, self.angle, self.color]]\\r\\n\\r\\n    @staticmethod\\r\\n    def from_block(block: \\'Block\\') -> \\'SerializedBlock\\':\\r\\n        \"\"\"Creates a SerializedBlock from a Block object.\"\"\"\\r\\n        color_enum = get_serialized_color(block.color.name)\\r\\n        assert color_enum is not None, f\"Unknown color: {block.color}\"\\r\\n        return SerializedBlock(\\r\\n            center_x=int(block.center[0]),  # Ensure int conversion\\r\\n            center_y=int(block.center[1]),\\r\\n            width=int(block.size[0]),\\r\\n            height=int(block.size[1]),\\r\\n            angle=float(block.angle),  # Ensure float conversion\\r\\n            color=color_enum.value  # Store enum as int\\r\\n        )\\r\\n\\r\\n    @classmethod\\r\\n    def from_bytes(cls, raw_bytes: bytes) -> \\'SerializedBlock\\':\\r\\n        \"\"\"Creates a SerializedBlock from raw bytes.\"\"\"\\r\\n        assert len(raw_bytes) == 16, f\"Expected 16 bytes, got {len(raw_bytes)}\"\\r\\n        return cls.from_buffer_copy(raw_bytes)\\r\\n\\r\\n    @classmethod\\r\\n    def from_floats(cls, f1: float, f2: float, f3: float, f4: float) -> \\'SerializedBlock\\':\\r\\n        \"\"\"Creates a SerializedBlock from four floats.\"\"\"\\r\\n        raw_bytes = struct.pack(\\'<4f\\', f1, f2, f3, f4)\\r\\n        return cls.from_bytes(raw_bytes)\\r\\n    \\r\\n    @classmethod\\r\\n    def from_raw_floats(cls, fs: List[float]) -> \\'SerializedBlock\\':\\r\\n        return SerializedBlock(\\r\\n            center_x=int(fs[0]),\\r\\n            center_y=int(fs[1]),\\r\\n            width=int(fs[2]),\\r\\n            height=int(fs[3]),\\r\\n            angle=fs[4],\\r\\n            color=int(fs[5]),\\r\\n        )\\r\\n\\r\\ndef serialize_to_floats(blocks: List[\\'Block\\']) -> List[float]:\\r\\n    \"\"\"Serializes a list of Blocks into a list of floats.\"\"\"\\r\\n    serialized_blocks = [SerializedBlock.from_block(block) for block in blocks]\\r\\n    return [0, 0] + [f for serialized_block in serialized_blocks for f in serialized_block.serialize_to_float()]\\r\\n\\r\\ndef deserialize_from_floats(floats: List[float]) -> List[SerializedBlock]:\\r\\n    \"\"\"Deserializes a list of floats into a list of SerializedBlocks.\"\"\"\\r\\n    assert len(floats) % 4 == 0, f\"Expected a multiple of 4 floats, got {len(floats)}\"\\r\\n    serialized_blocks = [SerializedBlock.from_raw_floats(floats[i:i+6]) for i in range(2, len(floats), 6)]\\r\\n    return serialized_blocks\\r\\n')\n" +
            "    from src.detector_contour import ColorBlockDetectorContour\n" +
            "    from src.detector_watershed import ColorBlockDetectorWatershed\n" +
            "    from src.detector_meanshift import ColorBlockDetectorMeanShift\n" +
            "    from src.visualizer import BlockVisualizer\n" +
            "    from src.utils.serializer import *\n" +
            "    from src.color_def import COLOR_DEF_LL\n" +
            "    from src.utils.serializer import *\n" +
            "    from ctypes import sizeof\n" +
            "    import cv2\n" +
            "    \n" +
            "    MAX_RET_BLK_CNT = 5\n" +
            "    \n" +
            "    def runPipeline(image, llrobot):\n" +
            "        # can change the algorithm with 3 options: contour, watershed, meanshift\n" +
            "        detector = ColorBlockDetectorContour(COLOR_DEF_LL)\n" +
            "        visualizer = BlockVisualizer(show=False)\n" +
            "    \n" +
            "        blocks = detector.process_frame(image)\n" +
            "        image = visualizer.show_final_result(image, blocks)\n" +
            "    \n" +
            "        # sort the block by area, which can be calculated by contour\n" +
            "        blocks = sorted(blocks, key=lambda x: cv2.contourArea(x.contour), reverse=True)\n" +
            "        \n" +
            "        #select the first MAX_RET_BLK_CNT blocks if exceed\n" +
            "    \n" +
            "        if len(blocks) > MAX_RET_BLK_CNT:\n" +
            "            blocks = blocks[:MAX_RET_BLK_CNT]\n" +
            "    \n" +
            "        serialized_blocks = serialize_to_floats(blocks)\n" +
            "    \n" +
            "        return [], image, serialized_blocks ";
    public void teleInit(HardwareMap hwm, Telemetry telemetry) {
        this.telemetry = telemetry;
        HL = hwm.get(HuskyLens.class, "HL");
        HuskyLens.Block[] myHuskyLensBlocks = HL.blocks();
        HL.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);

        imu = hwm.get(IMU.class, "imu");
        limelight = hwm.get(Limelight3A.class, "lm");
        limelight.uploadPython(snapScript, 0);
        limelight.pipelineSwitch(0);
        limelight.start();
        limelight.setPollRateHz(20);
        imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
                        )
                )
        );
    }


    public void apt() {
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    public Pose2d getfieldposeMT1() {
        double x = 0;
        double y = 0;
        double d = 0;
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            if (result != null && result.isValid()) {
                Pose3D botpose = result.getBotpose();
                if (botpose != null) {
                    x = botpose.getPosition().x * 39.3701;
                    y = botpose.getPosition().y * 39.3701;
                    d = 1;
                }
            }
        }
        return new Pose2d(x, y, d);
    }

    public void updateyaw(double yaw) {
        limelight.updateRobotOrientation(yaw);
    }

    public void selfyaw(double yaw) {
        limelight.updateRobotOrientation(imu.getRobotYawPitchRollAngles().getYaw());
    }

    public Pose2d getfieldposeMT2() {
        double x = 0;
        double y = 0;
        double d = 0;
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            if (result != null && result.isValid()) {
                Pose3D botpose = result.getBotpose_MT2();
                if (botpose != null) {
                    x = botpose.getPosition().x * 39.3701;
                    y = botpose.getPosition().y * 39.3701;
                    d = 1;
                }
            }
        }
        return new Pose2d(x, y, d);
    }

    public void llstop() {
        limelight.stop();
    }


    private HuskyLens HL;
   // private HuskyLens.Arrow[] myHuskyLensAs;
    private HuskyLens.Block[] myHuskyLensBlocks = null;
    public double hlGetAngle(HuskyLens.Block block) {
        if(block != null) {
            return VisionUtils.getStatus(block.width, block.height, block.x, block.y).getHeading();
        }
        else{
            return Math.PI;
        }
    }

    public BlockData getBlock(int color){
//        // HuskyLens implementation
//        myHuskyLensBlocks = HL.blocks();
//        telemetry.addData("Block count", JavaUtil.listLength(myHuskyLensBlocks));
//        for (HuskyLens.Block block : myHuskyLensBlocks) {
//            myHuskyLensBlock = block;
//            double ang = 114;
//            ang = VisionUtils.getStatus(block.width, block.height, block.x, block.y).getHeading();
//            if (Double.isNaN(ang)){
//                ang = 514;
//            }
//            telemetry.addData("Block", "id=" + myHuskyLensBlock.id + " size: " + myHuskyLensBlock.width + "x" + myHuskyLensBlock.height + " position: " + myHuskyLensBlock.x + "," + myHuskyLensBlock.y + " , angle: " + ang);
//            telemetry.update();
//            if(myHuskyLensBlock.id == id){
//                return myHuskyLensBlock;
//            }
//        }
//        telemetry.update();
        // LimeLight implementation
        limelight.pipelineSwitch(0);
        limelight.uploadPython(snapScript, 0);
        LLResult resultTmp = limelight.getLatestResult();
        //telemetry.addData("the result is valid or not ", resultTmp.isValid());
        LLStatus status = limelight.getStatus();
        //telemetry.addData("ll status", status.toString());
        double[] result = resultTmp.getPythonOutput();
        //telemetry.addData("raw python output", "");
        //for (double dbl : result){
            //telemetry.addData("", dbl);
        //}

        BlockData[] blocks = new BlockData[5];
        for(int i = 0; i < blocks.length; i++) {
            blocks[i] = new BlockData(result, 2 + i * 6);
            //telemetry.addData("Block", "x=" + blocks[i].centerX + "; y=" + blocks[i].centerY + "; w=" + blocks[i].width + "; h=" + blocks[i].height + "; angle=" + blocks[i].angle + "; color=" + blocks[i].color);
            if((blocks[i].color & color) != 0) {
                //telemetry.addData("Block", "Block detected at x=" + blocks[i].centerX + ", y=" + blocks[i].centerY);
                //telemetry.update();
                return blocks[i];
            }
        }
        //telemetry.update();
        return null;
    }

//    public HuskyLens.Arrow getA(){
//        myHuskyLensAs = HL.arrows();
//
//        return myHuskyLensAs[0];
//    }


    public double blueGetangle(){
        BlockData block = getBlock(BlockData.COLOR_BLUE);
        if(block != null){
            return block.getAngle();
        }
        block = getBlock(BlockData.COLOR_YELLOW);
        if(block != null){
            return block.getAngle();
        }
        return 0;

    }

    public double autoFocus(double angle){
        return (angle-90)/180.0;
    }

    public double redGetangle(){
        BlockData block = getBlock(BlockData.COLOR_RED);
        if(block != null){
            return block.getAngle();
        }
        block = getBlock(BlockData.COLOR_YELLOW);
        if(block != null){
            return block.getAngle();
        }
        return 90;

    }

//    public double autoFocus(){
//        return Vu.getServoValFromArrow(getA().x_origin,getA().y_origin,getA().x_target,getA().y_target);
//    }




}
