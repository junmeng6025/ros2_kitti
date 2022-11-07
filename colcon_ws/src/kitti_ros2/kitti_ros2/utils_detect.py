import torch.nn as nn
import torch
import cv2


def paddingImage(img, divider=32):
    """
    padding image size dividable by 32
    e.g.
    KITTI raw image CHW: [3, 375, 1242] -> [3, 384, 1248]
    """
    if img.shape[1] % divider != 0 or img.shape[2] % divider != 0:
        # padding1_mult = int(img.shape[1] / divider) + 1
        # padding2_mult = int(img.shape[2] / divider) + 1
        # pad1 = (divider * padding1_mult) - img.shape[1]
        # pad2 = (divider * padding2_mult) - img.shape[2]

        pad1 = divider - (img.shape[1] % divider)
        pad2 = divider - (img.shape[2] % divider)

        # pad1 = 384 - 375    # 9
        # pad2 = 1248 - 1242  # 6

        # 4-tuple: (W_left, W_right, H_top, H_bottom)
        padding = nn.ReplicationPad2d((0, pad2, 0, pad1))
        # https://pytorch.org/docs/stable/generated/torch.nn.ReplicationPad2d.html
        # padding (int, tuple) – the size of the padding.
        # If is int, uses the same padding in all boundaries.
        # If a 4-tuple, uses (padding_left, padding_right, padding_top, padding_bottom)

        # return padding(img)  # torch of the latest version has fixed the problem
        # current torch version: 1.7.0. NEEDS UPDATE
        return torch.squeeze(padding(torch.unsqueeze(img, 0)), 0)
    else:
        return img


class Colors:
    # Ultralytics color palette https://ultralytics.com/
    def __init__(self):
        # hex = matplotlib.colors.TABLEAU_COLORS.values()
        hexs = ('FF3838', 'FF9D97', 'FF701F', 'FFB21D', 'CFD231', '48F90A', '92CC17', '3DDB86', '1A9334', '00D4BB',
                '2C99A8', '00C2FF', '344593', '6473FF', '0018EC', '8438FF', '520085', 'CB38FF', 'FF95C8', 'FF37C7')
        self.palette = [self.hex2rgb(f'#{c}') for c in hexs]
        self.n = len(self.palette)

    def __call__(self, i, bgr=True):
        c = self.palette[int(i) % self.n]
        return (c[2], c[1], c[0]) if bgr else c

    @staticmethod
    def hex2rgb(h):  # rgb order (PIL)
        return tuple(int(h[1 + i:1 + i + 2], 16) for i in (0, 2, 4))


colors = Colors()  # create instance for 'from utils.plots import colors'

TYP_MAP = ['person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train', 'truck', 'boat', 'traffic light',
           'fire hydrant', 'stop sign', 'parking meter', 'bench', 'bird', 'cat', 'dog', 'horse', 'sheep', 'cow',
           'elephant', 'bear', 'zebra', 'giraffe', 'backpack', 'umbrella', 'handbag', 'tie', 'suitcase', 'frisbee',
           'skis', 'snowboard', 'sports ball', 'kite', 'baseball bat', 'baseball glove', 'skateboard', 'surfboard',
           'tennis racket', 'bottle', 'wine glass', 'cup', 'fork', 'knife', 'spoon', 'bowl', 'banana', 'apple',
           'sandwich', 'orange', 'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake', 'chair', 'couch',
           'potted plant', 'bed', 'dining table', 'toilet', 'tv', 'laptop', 'mouse', 'remote', 'keyboard', 'cell phone',
           'microwave', 'oven', 'toaster', 'sink', 'refrigerator', 'book', 'clock', 'vase', 'scissors', 'teddy bear',
           'hair drier', 'toothbrush']


def drawLabel_detect_and_distance(image, labels, threeD, color_dict=colors, model_names=TYP_MAP):
    img_cp = image.copy()
    for label in labels:
        (x1, y1, x2, y2, conf, cls) = label  # get a list of 'str'
        xc = int((int(x1) + int(x2))/2)
        yc = int((int(y1) + int(y2))/2)
        depth = (threeD[yc][xc])[-1] - 4.5
        typ = model_names[int(cls)]
        label = '%s, %.2fm' % (typ, depth)

        c1, c2 = (int(x1), int(y1)), (int(x2), int(y2))
        cv2.rectangle(img_cp, c1, c2, color_dict(cls), 2)
        tl = 3
        tf = max(tl - 1, 1)  # font thickness
        t_size = cv2.getTextSize(label, 0, fontScale=tl / 3, thickness=tf)[0]
        c2 = c1[0] + t_size[0], c1[1] - t_size[1] - 3
        cv2.rectangle(img_cp, c1, c2,
                      color_dict(cls), -1, cv2.LINE_AA)  # filled
        cv2.putText(img_cp, label, (c1[0], c1[1] - 2), 0, tl/3,
                    [255, 255, 255], thickness=tf, lineType=cv2.LINE_AA)

    return img_cp
