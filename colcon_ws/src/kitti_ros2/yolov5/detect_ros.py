from pathlib import Path
import torch.nn as nn
import math
import torch
import cv2
import time
import os
from yolov5.models.common import DetectMultiBackend
from yolov5.utils.torch_utils import select_device, time_sync
from yolov5.utils.general import (
    check_img_size, non_max_suppression, scale_coords, LOGGER)
from yolov5.utils.plots import Annotator, colors, save_one_box
import sys
sys.path.append(
    "/home/jun/ros2_guyue/colcon_ws/install/yolov5_detect/lib/python3.8/site-packages/yolov5_detect")
# import torch.backends.cudnn as cudnn

FILE = Path(__file__).resolve()
print("FILE: ", FILE)  # /detection_yolov5/scripts/yolov5/detect.py
ROOT = FILE.parents[0]  # YOLOv5 root directory
print("ROOT: ", ROOT)
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative
print("ROOT 2: ", ROOT)  # .


class Yolov5Detector:
    def __init__(self,
                 classesYaml=ROOT / 'data/coco128.yaml',  # all classes
                 device='cuda:0',  # cuda device, i.e. 0 or 0,1,2,3 or cpu
                 weights=ROOT / 'yolov5s.pt',  # model.pt path(s)
                 imgsz=(375, 1242),  # inference size (height, width)
                 conf_thres=0.25,  # confidence threshold
                 iou_thres=0.45,  # NMS IOU threshold
                 filter_classes=None,  # filter by class: --class 0, or --class 0 2 3
                 agnostic_nms=False,  # class-agnostic NMS
                 max_det=1000,  # maximum detections per image
                 half=False,  # use FP16 half-precision inference
                 ):
        print("Yolov5Detector: detectorInit()")

        imgsz *= 2 if len(imgsz) == 1 else 1  # expand

        # Load model
        self.classesYaml = classesYaml
        self.device = select_device(device)
        # self.device = select_device('0')
        self.model = DetectMultiBackend(
            weights, device=self.device, dnn=False, data=self.classesYaml)
        # self.model_stride = self.model.stride
        self.model_stride = 32
        self.model_names = self.model.names
        self.model_pt = self.model.pt
        self.model_jit = self.model.jit
        self.model_onnx = self.model.onnx
        self.model_engine = self.model.engine
        self.imgsz = check_img_size(
            imgsz, s=self.model_stride)  # check image size
        self.conf_thres = conf_thres
        self.iou_thres = iou_thres
        self.filter_classes = filter_classes
        self.agnostic_nms = agnostic_nms
        self.max_det = max_det

        self.line_thickness = 2
        self.hide_labels = False
        self.hide_conf = False

        # Half.  # half precision only supported by PyTorch on CUDA
        half &= (
            self.model_pt or self.model_jit or self.model_engine) and self.device.type != 'cpu'
        if self.model_pt or self.model_jit:
            self.model.model.half() if half else self.model.model.float()
        self.half = half

    def detectorRelease(self):
        print("Yolov5Detector: detectorRelease()")
        None

    def detectorWarmUp(self):
        print("Yolov5Detector: detectorWarmUp()")
        # Run inference
        # self.model.warmup(imgsz=(1, 3, *self.imgsz), half=self.half)  # warmup
        self.model.warmup(imgsz=(1, 3, *self.imgsz))

    """
    return:
    [tensor([[3.75988e+02, 3.47674e+01, 5.74189e+02, 3.70005e+02, 8.75967e-01, 0.00000e+00],
        [5.00493e+01, 1.12659e+02, 5.00566e+02, 3.71366e+02, 5.72355e-01, 0.00000e+00],
        [2.19189e+02, 2.23126e+02, 2.54996e+02, 3.73880e+02, 5.25457e-01, 2.70000e+01]], device='cuda:0')]
    """

    def detectImage(self, imageNumpy, imageOrigin, needProcess=False):
        #print("Yolov5Detector: detectImage()")

        # torch.cuda.empty_cache() #清空CUDA

        if needProcess:
            image = torch.from_numpy(imageNumpy).to(self.device)
            image = image.half() if self.half else image.float()  # uint8 to fp16/32
            image /= 255  # 0 - 255 to 0.0 - 1.0  # 归一化

            if len(image.shape) == 3:
                image = image[None]  # expand for batch dim .  功能等同于: image = torch.unsqueeze(im, dim=0)
                #image = torch.unsqueeze(im, dim=0)
        else:
            image = imageNumpy

        pred = self.model(image, augment=False, visualize=False)
        # NMS
        pred = non_max_suppression(pred, self.conf_thres, self.iou_thres,
                                   self.filter_classes, self.agnostic_nms, max_det=self.max_det)
        #print("Yolov5Detector: detectImage() pred: ",pred)

        if needProcess:
            imgRet, detect, imageLabelList = self._postProcessPredictions(pred, image, imageOrigin)
            imageLabelList = imageLabelList[0] if len(imageLabelList) > 0 else []
            #detect = detect[0] if len(detect)>0 else []
            detect = detect.cpu().numpy()
            return imgRet, detect, imageLabelList
        else:
            return pred

    def postProcessPredictions(self, pred, imageProsessed, imageOrigin):
        imgRet, detect, imageLabelList = self._postProcessPredictions(pred, imageProsessed, imageOrigin)
        return imgRet, detect

    def _postProcessPredictions(self, pred, imageProsessed, imageOrigin):
        #print("Yolov5Detector: postProcessPredictions()")
        im = imageProsessed
        im0s = imageOrigin

        #dt, seen = [0.0, 0.0, 0.0], 0

        imageLabelList = []

        # Post Process predictions
        for i, det in enumerate(pred):  # per image ,   len(pred) = 1
            imageLabels = []
            imageLabelList.append(imageLabels)
            #seen += 1
            im0 = im0s.copy()

            s = '%gx%g ' % im.shape[2:]  # print string
            # gn = torch.tensor(im0.shape)[[1, 0, 1, 0]]  # normalization gain whwh
            # imc = im0.copy() if True else im0  # for save_crop
            annotator = Annotator(im0, line_width=self.line_thickness, example=str(self.model_names))
            if len(det):
                # Rescale boxes from img_size to im0 size
                det[:, :4] = scale_coords(im.shape[2:], det[:, :4], im0.shape).round()

                # Print results
                for c in det[:, -1].unique():
                    n = (det[:, -1] == c).sum()  # detections per class
                    s += f"{n} {self.model_names[int(c)]}{'s' * (n > 1)}, "  # add to string
                    imageLabels.append(self.model_names[int(c)])

                # Write results
                for *xyxy, conf, cls in reversed(det):
                    c = int(cls)  # integer class
                    label = None if self.hide_labels else (
                        self.model_names[c] if self.hide_conf else f'{self.model_names[c]} {conf:.2f}')
                    annotator.box_label(xyxy, label, color=colors(c, True))
                    # if True:
                    #    save_one_box(xyxy, imc, file=save_dir / 'crops' / self.model_names[c] / f'{p.stem}.jpg', BGR=True)

            # Print time (inference-only)
            #LOGGER.info(f'{s}Done. ({t3 - t2:.3f}s)')

            # Stream results
            im0 = annotator.result()
        return im0, det, imageLabelList


if __name__ == "__main__":
    None
