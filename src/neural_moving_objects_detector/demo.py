# Conclude setting / general reprocessing / plots / metrices / datasets
# from utils.utils import \
#     time_synchronized,select_device, increment_path,\
#     scale_coords,xyxy2xywh,non_max_suppression,split_for_trace_model,\
#     driving_area_mask,lane_line_mask,plot_one_box,show_seg_result,\
#     AverageMeter,\
#     LoadImages
import os
from pathlib import Path
import random
import cv2
import numpy as np
import torch
import torchvision
import ast


def main(filestring):
    def select_device(device='', batch_size=None):
        # device = 'cpu' or '0' or '0,1,2,3'
        cpu = device.lower() == 'cpu'
        if cpu:
            os.environ['CUDA_VISIBLE_DEVICES'] = '-1'  # force torch.cuda.is_available() = False
        elif device:  # non-cpu device requested
            os.environ['CUDA_VISIBLE_DEVICES'] = device  # set environment variable
            assert torch.cuda.is_available(), f'CUDA unavailable, invalid device {device} requested'  # check availability

        cuda = not cpu and torch.cuda.is_available()
        if cuda:
            n = torch.cuda.device_count()
            if n > 1 and batch_size:  # check that batch_size is compatible with device_count
                assert batch_size % n == 0, f'batch-size {batch_size} not multiple of GPU count {n}'
            for i, d in enumerate(device.split(',') if device else range(n)):
                p = torch.cuda.get_device_properties(i)
                

        return torch.device('cuda:0' if cuda else 'cpu')


    def plot_one_box(x, img, color=None, label=None, line_thickness=3):
        # Plots one bounding box on image img
        tl = line_thickness or round(0.002 * (img.shape[0] + img.shape[1]) / 2) + 1  # line/font thickness
        color = color or [random.randint(0, 255) for _ in range(3)]
        c1, c2 = (int(x[0]), int(x[1])), (int(x[2]), int(x[3]))
        cv2.rectangle(img, c1, c2, [0,255,255], thickness=2, lineType=cv2.LINE_AA)
        if label:
            tf = max(tl - 1, 1)  # font thickness
            t_size = cv2.getTextSize(label, 0, fontScale=tl / 3, thickness=tf)[0]
            c2 = c1[0] + t_size[0], c1[1] - t_size[1] - 3


    def _make_grid(nx=20, ny=20):
            yv, xv = torch.meshgrid([torch.arange(ny), torch.arange(nx)])
            return torch.stack((xv, yv), 2).view((1, 1, ny, nx, 2)).float()
        
    def split_for_trace_model(pred = None, anchor_grid = None):
        z = []
        st = [8,16,32]
        for i in range(3):
            bs, _, ny, nx = pred[i].shape  
            pred[i] = pred[i].view(bs, 3, 85, ny, nx).permute(0, 1, 3, 4, 2).contiguous()
            y = pred[i].sigmoid()
            gr = _make_grid(nx, ny).to(pred[i].device)
            y[..., 0:2] = (y[..., 0:2] * 2. - 0.5 + gr) * st[i]  # xy
            y[..., 2:4] = (y[..., 2:4] * 2) ** 2 * anchor_grid[i]  # wh
            z.append(y.view(bs, -1, 85))
        pred = torch.cat(z, 1)
        return pred

    def scale_coords(img1_shape, coords, img0_shape, ratio_pad=None):
        # Rescale coords (xyxy) from img1_shape to img0_shape
        if ratio_pad is None:  # calculate from img0_shape
            gain = min(img1_shape[0] / img0_shape[0], img1_shape[1] / img0_shape[1])  # gain  = old / new
            pad = (img1_shape[1] - img0_shape[1] * gain) / 2, (img1_shape[0] - img0_shape[0] * gain) / 2  # wh padding
        else:
            gain = ratio_pad[0][0]
            pad = ratio_pad[1]

        coords[:, [0, 2]] -= pad[0]  # x padding
        coords[:, [1, 3]] -= pad[1]  # y padding
        coords[:, :4] /= gain
        clip_coords(coords, img0_shape)
        return coords


    def clip_coords(boxes, img_shape):
        # Clip bounding xyxy bounding boxes to image shape (height, width)
        boxes[:, 0].clamp_(0, img_shape[1])  # x1
        boxes[:, 1].clamp_(0, img_shape[0])  # y1
        boxes[:, 2].clamp_(0, img_shape[1])  # x2
        boxes[:, 3].clamp_(0, img_shape[0])  # y2

    def xywh2xyxy(x):
        # Convert nx4 boxes from [x, y, w, h] to [x1, y1, x2, y2] where xy1=top-left, xy2=bottom-right
        y = x.clone() if isinstance(x, torch.Tensor) else np.copy(x)
        y[:, 0] = x[:, 0] - x[:, 2] / 2  # top left x
        y[:, 1] = x[:, 1] - x[:, 3] / 2  # top left y
        y[:, 2] = x[:, 0] + x[:, 2] / 2  # bottom right x
        y[:, 3] = x[:, 1] + x[:, 3] / 2  # bottom right y
        return y

    def non_max_suppression(prediction, conf_thres=0.25, iou_thres=0.45, classes=None, agnostic=False, multi_label=False,
                            labels=()):
        """Runs Non-Maximum Suppression (NMS) on inference results

        Returns:
            list of detections, on (n,6) tensor per image [xyxy, conf, cls]
        """

        nc = prediction.shape[2] - 5  # number of classes
        xc = prediction[..., 4] > conf_thres  # candidates

        # Settings
        min_wh, max_wh = 2, 4096  # (pixels) minimum and maximum box width and height
        max_det = 300  # maximum number of detections per image
        max_nms = 30000  # maximum number of boxes into torchvision.ops.nms()
        time_limit = 10.0  # seconds to quit after
        redundant = True  # require redundant detections
        multi_label &= nc > 1  # multiple labels per box (adds 0.5ms/img)
        merge = False  # use merge-NMS

        output = [torch.zeros((0, 6), device=prediction.device)] * prediction.shape[0]
        for xi, x in enumerate(prediction):  # image index, image inference
            # Apply constraints
            # x[((x[..., 2:4] < min_wh) | (x[..., 2:4] > max_wh)).any(1), 4] = 0  # width-height
            x = x[xc[xi]]  # confidence

            # Cat apriori labels if autolabelling
            if labels and len(labels[xi]):
                l = labels[xi]
                v = torch.zeros((len(l), nc + 5), device=x.device)
                v[:, :4] = l[:, 1:5]  # box
                v[:, 4] = 1.0  # conf
                v[range(len(l)), l[:, 0].long() + 5] = 1.0  # cls
                x = torch.cat((x, v), 0)

            # If none remain process next image
            if not x.shape[0]:
                continue

            # Compute conf
            x[:, 5:] *= x[:, 4:5]  # conf = obj_conf * cls_conf

            # Box (center x, center y, width, height) to (x1, y1, x2, y2)
            box = xywh2xyxy(x[:, :4])

            # Detections matrix nx6 (xyxy, conf, cls)
            if multi_label:
                i, j = (x[:, 5:] > conf_thres).nonzero(as_tuple=False).T
                x = torch.cat((box[i], x[i, j + 5, None], j[:, None].float()), 1)
            else:  # best class only
                conf, j = x[:, 5:].max(1, keepdim=True)
                x = torch.cat((box, conf, j.float()), 1)[conf.view(-1) > conf_thres]

            # Filter by class
            if classes is not None:
                x = x[(x[:, 5:6] == torch.tensor(classes, device=x.device)).any(1)]

            # Apply finite constraint
            # if not torch.isfinite(x).all():
            #     x = x[torch.isfinite(x).all(1)]

            # Check shape
            n = x.shape[0]  # number of boxes
            if not n:  # no boxes
                continue
            elif n > max_nms:  # excess boxes
                x = x[x[:, 4].argsort(descending=True)[:max_nms]]  # sort by confidence

            # Batched NMS
            c = x[:, 5:6] * (0 if agnostic else max_wh)  # classes
            boxes, scores = x[:, :4] + c, x[:, 4]  # boxes (offset by class), scores
            i = torchvision.ops.nms(boxes, scores, iou_thres)  # NMS
            if i.shape[0] > max_det:  # limit detections
                i = i[:max_det]
            if merge and (1 < n < 3E3):  # Merge NMS (boxes merged using weighted mean)
                # update boxes as boxes(i,4) = weights(i,n) * boxes(n,4)
                iou = box_iou(boxes[i], boxes) > iou_thres  # iou matrix
                weights = iou * scores[None]  # box weights
                x[i, :4] = torch.mm(weights, x[:, :4]).float() / weights.sum(1, keepdim=True)  # merged boxes
                if redundant:
                    i = i[iou.sum(1) > 1]  # require redundancy

            output[xi] = x[i]

        return output

    def box_iou(box1, box2):
        # https://github.com/pytorch/vision/blob/master/torchvision/ops/boxes.py
        """
        Return intersection-over-union (Jaccard index) of boxes.
        Both sets of boxes are expected to be in (x1, y1, x2, y2) format.
        Arguments:
            box1 (Tensor[N, 4])
            box2 (Tensor[M, 4])
        Returns:
            iou (Tensor[N, M]): the NxM matrix containing the pairwise
                IoU values for every element in boxes1 and boxes2
        """

        def box_area(box):
            # box = 4xn
            return (box[2] - box[0]) * (box[3] - box[1])

        area1 = box_area(box1.T)
        area2 = box_area(box2.T)

        # inter(N,M) = (rb(N,M,2) - lt(N,M,2)).clamp(0).prod(2)
        inter = (torch.min(box1[:, None, 2:], box2[:, 2:]) - torch.max(box1[:, None, :2], box2[:, :2])).clamp(0).prod(2)
        return inter / (area1[:, None] + area2 - inter)  # iou = inter / (area1 + area2 - inter)

    class LoadImages:  # for inference
        def __init__(self, filestring, img_size=640, stride=32):

            self.filestring = filestring
            self.img_size = img_size
            self.stride = stride
            self.nf = 1  # number of files
            self.mode = 'image'
            
            self.cap = None
            ## af.write("__init__ foi\n")

        def __iter__(self):
            self.count = 0
            return self

        def __next__(self):
            #af = open("aaaaaaaaaaaaaaaaaa.txt", "a")
            if self.count == self.nf:
                raise StopIteration

            # Read image
            self.count += 1
            #img0 = cv2.imread(path)  # BGR
            img0 = ast.literal_eval(self.filestring)
            img0 = np.array(img0)
            img0 = np.float32(img0)

            #print(f'image {self.count}/{self.nf} {path}: ', end='')

            # Padded resize
            img0 = cv2.resize(img0, (640,480), interpolation=cv2.INTER_LINEAR)
            img = letterbox(img0, self.img_size, stride=self.stride)[0]

            # Convert
            img = img[:, :, ::-1].transpose(2, 0, 1)  # BGR to RGB, to 3x416x416
            img = np.ascontiguousarray(img)
            ## af.write("__next__ foi\n")
            return img, img0

    def letterbox(img, new_shape=(640, 480), color=(114, 114, 114), auto=True, scaleFill=False, scaleup=True, stride=32):
        #af = open("aaaaaaaaaaaaaaaaaa.txt", "a")
        # Resize and pad image while meeting stride-multiple constraints
        ## af.write("shape\n")
        shape = img.shape[:2]  # current shape [height, width]
        if isinstance(new_shape, int):
            new_shape = (new_shape, new_shape)
        #print(sem_img.shape)
        # Scale ratio (new / old)
        r = min(new_shape[0] / shape[0], new_shape[1] / shape[1])

        if not scaleup:  # only scale down, do not scale up (for better test mAP)
            r = min(r, 1.0)

        # Compute padding
        ratio = r, r  # width, height ratios
        new_unpad = int(round(shape[1] * r)), int(round(shape[0] * r))
        dw, dh = new_shape[1] - new_unpad[0], new_shape[0] - new_unpad[1]  # wh padding
        if auto:  # minimum rectangle
            dw, dh = np.mod(dw, stride), np.mod(dh, stride)  # wh padding
        elif scaleFill:  # stretch
            dw, dh = 0.0, 0.0
            new_unpad = (new_shape[1], new_shape[0])
            ratio = new_shape[1] / shape[1], new_shape[0] / shape[0]  # width, height ratios

        dw /= 2  # divide padding into 2 sides
        dh /= 2

        if shape[::-1] != new_unpad:  # resize
            img = cv2.resize(img, new_unpad, interpolation=cv2.INTER_LINEAR)
        
        top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
        left, right = int(round(dw - 0.1)), int(round(dw + 0.1))

        img = cv2.copyMakeBorder(img, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color)  # add border
        
        return img, ratio, (dw, dh)

    def detect(filestring):
        # af = open("aaaaaaaaaaaaaaaaaa.txt", "a")
        # af.write("file_path\n")
        file_path = os.path.realpath(__file__)[:-7]
        # setting and directories
        weights, imgsz = 'YOLOPv2/data/weights/yolopv2.pt', 480
        save_img = True  # save inference images
        # Load model
        stride =32
        # af.write("torch\n")
        model  = torch.jit.load(Path.joinpath(Path(file_path), weights))
        # af.write("select_device\n")
        device = select_device('0')
        half = device.type != 'cpu'  # half precision only supported on CUDA
        # af.write("model\n")
        model = model.to(device)

        # af.write("eval\n")
        if half:
            model.half()  # to FP16  
        model.eval()
        
        # Set Dataloader
        vid_path, vid_writer = None, None
        # af.write("LoadImages\n")
        dataset = LoadImages( filestring=filestring, img_size=imgsz, stride=stride)
        # Run inference
        # af.write("run once\n")
        if device.type != 'cpu':
            model(torch.zeros(1, 3, imgsz, imgsz).to(device).type_as(next(model.parameters())))  # run once
        for  img, im0s in dataset:
            img = torch.from_numpy(img).to(device)
            img = img.half() if half else img.float()  # uint8 to fp16/32
            img /= 255.0  # 0 - 255 to 0.0 - 1.0

            if img.ndimension() == 3:
                img = img.unsqueeze(0)

            [pred,anchor_grid],seg,ll= model(img)

            pred = split_for_trace_model(pred,anchor_grid)

            pred = non_max_suppression(pred, 0.3, 0.45, classes=None, agnostic=False)

            # Process detections
            for i, det in enumerate(pred):  # detections per image
                im0 =  im0s
                if len(det):
                    # Rescale boxes from img_size to im0 size
                    det[:, :4] = scale_coords(img.shape[2:], det[:, :4], im0.shape).round()

                    # Write results
                    for *xyxy, conf, cls in reversed(det):
                        if save_img :  # Add bbox to image
                            plot_one_box(xyxy, im0, line_thickness=3)
        return im0



    import sys
    sys.argv=['']

    with torch.no_grad():
        im0 = detect(filestring)
        lis = ""
        i = 0
        for n1 in im0:
            for n2 in n1:
                for n3 in n2:
                    lis += str(int(n3)) + ("," if i < len(n2)*len(n1)-1 else "\n")
                    i+=1
            i = 0
        lis = lis[:-1]
        return lis