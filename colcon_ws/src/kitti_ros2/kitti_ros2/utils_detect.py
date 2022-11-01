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


def drawLabel_detect_and_distance(image, labels, threeD):
    img_cp = image.copy()
    for label in labels:
        (x1, y1, x2, y2, conf, typ) = label  # get a list of 'str'
        xc = int((int(x1) + int(x2))/2)
        yc = int((int(y1) + int(y2))/2)
        depth = (threeD[yc][xc])[-1] - 4.5
        label = '%s, %.2fm' % (typ, depth)

        c1, c2 = (int(x1), int(y1)), (int(x2), int(y2))
        cv2.rectangle(img_cp, c1, c2, (128, 128, 128), 2)
        tl = 3
        tf = max(tl - 1, 1)  # font thickness
        t_size = cv2.getTextSize(label, 0, fontScale=tl / 3, thickness=tf)[0]
        c2 = c1[0] + t_size[0], c1[1] - t_size[1] - 3
        cv2.rectangle(img_cp, c1, c2, (128, 128, 128), -
                      1, cv2.LINE_AA)  # filled
        cv2.putText(img_cp, label, (c1[0], c1[1] - 2), 0, tl/3,
                    [255, 255, 255], thickness=tf, lineType=cv2.LINE_AA)

    return img_cp
