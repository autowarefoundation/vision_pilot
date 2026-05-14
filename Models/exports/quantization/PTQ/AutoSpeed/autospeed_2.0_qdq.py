import os
import cv2
import glob
import onnx
import yaml
import torch
import numpy as np
import onnxruntime as ort

from tqdm import tqdm
from time import time
from pathlib import Path
from torch.utils import data
from argparse import ArgumentParser
from Models.data_utils.load_data_auto_speed import LoadDataAutoSpeed

INPUT_WIDTH = 1024
INPUT_HEIGHT = 512

NUM_CALIBRATION_IMAGES = 130

from onnxruntime.quantization import (
    CalibrationDataReader,
    quantize_static,
    QuantType,
    QuantFormat,
    CalibrationMethod,
)

# NODES_TO_EXCLUDE = ["node_softmax",
#                     "node_softmax_1",
#                     "node_sigmoid",
#                     "node_cat_18", ]

NODES_TO_EXCLUDE = [
    # DFL / distribution decode
    "node_softmax_1",
    "node_Conv_1228",

    # Final bbox decode math
    "node_unsqueeze",
    "node_sub_286",
    "node_add_1469",
    "node_add_1474",
    "node_div",
    "node_sub_291",
    "node_cat_17",
    "node_mul_924",

    # Final class activation
    "node_sigmoid",

    # Final output concat
    "node_cat_18",
]


class ImageCalibrationDataReader(CalibrationDataReader):

    def __init__(self, image_dir, input_name, input_width, input_height, num_images=500, ):
        self.batch_size = 1
        self.input_name = input_name
        self.input_width = input_width
        self.input_height = input_height

        image_paths = []
        image_paths.extend(glob.glob(os.path.join(image_dir, "*.jpg")))
        image_paths.extend(glob.glob(os.path.join(image_dir, "*.png")))
        image_paths.extend(glob.glob(os.path.join(image_dir, "*.jpeg")))

        image_paths = sorted(image_paths)

        if num_images is not None:
            image_paths = image_paths[:num_images]

        self.image_paths = image_paths

        # Iterator index
        self.current_index = 0

        self.pbar = tqdm(total=len(self.image_paths), desc="Calibration", unit="img")

    def preprocess_image(self, image_path):
        image = cv2.imread(image_path)
        if image is None:
            raise RuntimeError(f"Failed to load image: {image_path}")

        # IMPORTANT: use SAME preprocessing as inference
        image, _, _, _ = resize_letterbox(
            image,
            target_size=(self.input_width, self.input_height)
        )

        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        image = image.astype(np.float32) / 255.0
        image = np.transpose(image, (2, 0, 1))

        return image[None, :, :, :]

    def get_next(self):
        # End of dataset
        if self.current_index >= len(self.image_paths):
            self.pbar.close()
            return None

        image_path = self.image_paths[self.current_index]

        self.current_index += 1

        self.pbar.update(1)

        return {
            self.input_name: self.preprocess_image(image_path)
        }

    def rewind(self):
        self.current_index = 0
        self.pbar.reset()


def quantize(dataset, fp32_onnx_model_path, int8_onnx_model_path):
    session = ort.InferenceSession(
        fp32_onnx_model_path,
        providers=["CPUExecutionProvider"]
    )

    input_name = session.get_inputs()[0].name

    # Create calibration reader
    calibration_data_reader = ImageCalibrationDataReader(
        image_dir=dataset + "/images/val/",
        input_name=input_name,
        input_width=INPUT_WIDTH,
        input_height=INPUT_HEIGHT,
        num_images=NUM_CALIBRATION_IMAGES,
    )

    # Quantize
    quantize_static(
        model_input=fp32_onnx_model_path,
        model_output=int8_onnx_model_path,

        calibration_data_reader=calibration_data_reader,

        # BEST FOR ACCURACY
        quant_format=QuantFormat.QDQ,

        # INT8
        # activation_type=QuantType.QUInt8,
        activation_type=QuantType.QInt8,
        weight_type=QuantType.QInt8,

        # VERY IMPORTANT
        per_channel=True,

        # MUCH BETTER THAN MINMAX
        # calibrate_method=CalibrationMethod.Entropy,
        # calibrate_method=CalibrationMethod.MinMax,
        calibrate_method=CalibrationMethod.Percentile,

        # Keep outputs/classifier FP32
        nodes_to_exclude=NODES_TO_EXCLUDE,

        # Recommended
        reduce_range=False,
    )

    print()
    print("INT8 model saved:")
    print(int8_onnx_model_path)


# Evaluate functions
def run_onnx_model(session, image_tensor):
    # Get input name
    input_name = session.get_inputs()[0].name

    input_data = image_tensor.detach().cpu().numpy()
    # input_data = np.expand_dims(input_data, axis=0)
    outputs = session.run(None, {input_name: input_data})
    outputs = outputs[0]

    return outputs


def wh2xy(x):
    y = x.clone() if isinstance(x, torch.Tensor) else np.copy(x)
    y[:, 0] = x[:, 0] - x[:, 2] / 2  # top left x
    y[:, 1] = x[:, 1] - x[:, 3] / 2  # top left y
    y[:, 2] = x[:, 0] + x[:, 2] / 2  # bottom right x
    y[:, 3] = x[:, 1] + x[:, 3] / 2  # bottom right y
    return y


def nms(boxes, scores, iou_threshold=0.45):
    x1 = boxes[:, 0]
    y1 = boxes[:, 1]
    x2 = boxes[:, 2]
    y2 = boxes[:, 3]

    areas = (x2 - x1) * (y2 - y1)
    order = scores.argsort()[::-1]

    keep = []

    while order.size > 0:
        i = order[0]
        keep.append(i)

        xx1 = np.maximum(x1[i], x1[order[1:]])
        yy1 = np.maximum(y1[i], y1[order[1:]])
        xx2 = np.minimum(x2[i], x2[order[1:]])
        yy2 = np.minimum(y2[i], y2[order[1:]])

        w = np.maximum(0.0, xx2 - xx1)
        h = np.maximum(0.0, yy2 - yy1)

        inter = w * h
        iou = inter / (areas[i] + areas[order[1:]] - inter)

        inds = np.where(iou <= iou_threshold)[0]
        order = order[inds + 1]

    return np.array(keep, dtype=np.int64)


def non_max_suppression(outputs, confidence_threshold=0.001, iou_threshold=0.65):
    max_wh = 7680
    max_det = 300
    max_nms = 30000

    bs = outputs.shape[0]  # batch size
    nc = outputs.shape[1] - 4  # number of classes
    xc = outputs[:, 4:4 + nc].max(axis=1) > confidence_threshold  # candidates

    # Settings
    start = time()
    limit = 0.5 + 0.05 * bs  # seconds to quit after
    output = [torch.zeros((0, 6), device=outputs.device)] * bs
    for index, x in enumerate(outputs):  # image index, image inference
        x = x.transpose(1, 0)[xc[index]]  # confidence

        # If none remain process next image
        if not x.shape[0]:
            continue

        # matrix nx6 (box, confidence, cls)
        box, cls = np.split(x, [4], axis=1)
        box = wh2xy(box)  # (cx, cy, w, h) to (x1, y1, x2, y2)
        if nc > 1:
            i, j = np.nonzero(cls > confidence_threshold)
            x = np.concatenate((box[i], x[i, 4 + j, None], j[:, None].astype(np.float32)), axis=1)
        else:  # best class only
            conf, j = cls.max(1, keepdim=True)
            x = np.concatenate((box, conf, j.astype(np.float32)), axis=1)[conf.reshape(-1) > confidence_threshold]

        # Check shape
        n = x.shape[0]  # number of boxes
        if not n:  # no boxes
            continue
        x = x[x[:, 4].argsort()[::-1][:max_nms]]  # sort by confidence and remove excess boxes

        # Batched NMS
        c = x[:, 5:6] * max_wh  # classes
        boxes, scores = x[:, :4] + c, x[:, 4]  # boxes, scores
        indices = nms(boxes, scores, iou_threshold)  # NMS
        indices = indices[:max_det]  # limit detections

        output[index] = x[indices]
        if (time() - start) > limit:
            break  # time limit exceeded

    return output


def compute_metric(output, target, iou_v):
    # intersection(N,M) = (rb(N,M,2) - lt(N,M,2)).clamp(0).prod(2)
    a = np.expand_dims(target[:, 1:], axis=1)  # (N, 1, D)
    a1, a2 = np.split(a, 2, axis=2)  # split last dim into 2

    b = np.expand_dims(output[:, :4], axis=0)  # (1, M, 4)
    b1, b2 = np.split(b, 2, axis=2)  # split last dim into 2

    intersection = np.clip(np.minimum(a2, b2) - np.maximum(a1, b1), 0, None).prod(axis=2)
    # IoU = intersection / (area1 + area2 - intersection)
    iou = intersection / ((a2 - a1).prod(2) + (b2 - b1).prod(2) - intersection + 1e-7)

    correct = np.zeros((output.shape[0], iou_v.shape[0]))
    correct = correct.astype(bool)
    for i in range(len(iou_v)):
        # IoU > threshold and classes match
        x = np.where((iou >= iou_v[i]) & (target[:, 0:1] == output[:, 5]))
        if x[0].shape[0]:
            matches = np.concatenate((np.stack(x, axis=1), iou[x[0], x[1]][:, None]), axis=1)
            if x[0].shape[0] > 1:
                matches = matches[matches[:, 2].argsort()[::-1]]
                matches = matches[np.unique(matches[:, 1], return_index=True)[1]]
                matches = matches[np.unique(matches[:, 0], return_index=True)[1]]
            correct[matches[:, 1].astype(int), i] = True
    return torch.tensor(correct, dtype=torch.bool, device=output.device)


def smooth(y, f=0.1):
    # Box filter of fraction f
    nf = round(len(y) * f * 2) // 2 + 1  # number of filter elements (must be odd)
    p = np.ones(nf // 2)  # ones padding
    yp = np.concatenate((p * y[0], y, p * y[-1]), 0)  # y padded
    return np.convolve(yp, np.ones(nf) / nf, mode='valid')  # y-smoothed


def compute_ap(tp, conf, output, target, plot=False, names=(), eps=1E-16):
    """
    Compute the average precision, given the recall and precision curves.
    Source: https://github.com/rafaelpadilla/Object-Detection-Metrics.
    # Arguments
        tp:  True positives (nparray, nx1 or nx10).
        conf:  Object-ness value from 0-1 (nparray).
        output:  Predicted object classes (nparray).
        target:  True object classes (nparray).
    # Returns
        The average precision
    """
    # Sort by object-ness
    i = np.argsort(-conf)
    tp, conf, output = tp[i], conf[i], output[i]

    # Find unique classes
    unique_classes, nt = np.unique(target, return_counts=True)
    nc = unique_classes.shape[0]  # number of classes, number of detections

    # Create Precision-Recall curve and compute AP for each class
    p = np.zeros((nc, 1000))
    r = np.zeros((nc, 1000))
    ap = np.zeros((nc, tp.shape[1]))
    px, py = np.linspace(start=0, stop=1, num=1000), []  # for plotting
    for ci, c in enumerate(unique_classes):
        i = output == c
        nl = nt[ci]  # number of labels
        no = i.sum()  # number of outputs
        if no == 0 or nl == 0:
            continue

        # Accumulate FPs and TPs
        fpc = (1 - tp[i]).cumsum(0)
        tpc = tp[i].cumsum(0)

        # Recall
        recall = tpc / (nl + eps)  # recall curve
        # negative x, xp because xp decreases
        r[ci] = np.interp(-px, -conf[i], recall[:, 0], left=0)

        # Precision
        precision = tpc / (tpc + fpc)  # precision curve
        p[ci] = np.interp(-px, -conf[i], precision[:, 0], left=1)  # p at pr_score

        # AP from recall-precision curve
        for j in range(tp.shape[1]):
            m_rec = np.concatenate(([0.0], recall[:, j], [1.0]))
            m_pre = np.concatenate(([1.0], precision[:, j], [0.0]))

            # Compute the precision envelope
            m_pre = np.flip(np.maximum.accumulate(np.flip(m_pre)))

            # Integrate area under curve
            x = np.linspace(start=0, stop=1, num=101)  # 101-point interp (COCO)
            ap[ci, j] = np.trapezoid(np.interp(x, m_rec, m_pre), x)  # integrate
            if plot and j == 0:
                py.append(np.interp(px, m_rec, m_pre))  # precision at mAP@0.5

    # Compute F1 (harmonic mean of precision and recall)
    f1 = 2 * p * r / (p + r + eps)
    # if plot:
    #     names = dict(enumerate(names))  # to dict
    #     names = [v for k, v in names.items() if k in unique_classes]  # list: only classes that have data
    #     plot_pr_curve(px, py, ap, names, save_dir="./weights/PR_curve.png")
    #     plot_curve(px, f1, names, save_dir="./weights/F1_curve.png", y_label="F1")
    #     plot_curve(px, p, names, save_dir="./weights/P_curve.png", y_label="Precision")
    #     plot_curve(px, r, names, save_dir="./weights/R_curve.png", y_label="Recall")
    i = smooth(f1.mean(0), 0.1).argmax()  # max F1 index
    p, r, f1 = p[:, i], r[:, i], f1[:, i]
    tp = (r * nt).round()  # true positives
    fp = (tp / (p + eps) - tp).round()  # false positives
    ap50, ap = ap[:, 0], ap.mean(1)  # AP@0.5, AP@0.5:0.95
    m_pre, m_rec = p.mean(), r.mean()
    map50, mean_ap = ap50.mean(), ap.mean()
    return tp, fp, m_pre, m_rec, map50, mean_ap


def evaluate(int8_model_path, dataset, params):
    session = ort.InferenceSession(
        int8_model_path,
        providers=["CPUExecutionProvider"]
    )

    current_dir = Path(dataset + "/images/val/")
    filenames = [f.as_posix() for f in current_dir.rglob("*") if f.is_file()]

    dataset = LoadDataAutoSpeed(filenames, 1024, 512, params, augment=False)
    loader = data.DataLoader(dataset, batch_size=4, shuffle=False, num_workers=4,
                             pin_memory=True, collate_fn=LoadDataAutoSpeed.collate_fn)

    # Configure
    iou_v = np.linspace(start=0.5, stop=0.95, num=10, dtype=np.float32)
    n_iou = iou_v.size

    m_pre = 0
    m_rec = 0
    map50 = 0
    mean_ap = 0
    metrics = []
    p_bar = tqdm(loader, desc=('%10s' * 5) % ('', 'precision', 'recall', 'mAP50', 'mAP'))
    for samples, targets in p_bar:
        samples = samples / 255.  # 0 - 255 to 0.0 - 1.0
        _, _, h, w = samples.shape  # batch-size, channels, height, width
        scale = torch.tensor((w, h, w, h))
        # Inference
        outputs = run_onnx_model(session, samples)
        # NMS
        outputs = non_max_suppression(outputs)
        # Metrics
        for i, output in enumerate(outputs):
            idx = targets['idx'] == i
            cls = targets['cls'][idx]
            box = targets['box'][idx]

            metric = np.zeros((output.shape[0], n_iou), dtype=bool)

            if output.shape[0] == 0:
                if cls.shape[0]:
                    metrics.append((metric, *torch.zeros((2, 0)), cls.squeeze(-1)))
                continue
            # Evaluate
            if cls.shape[0]:
                target = np.concatenate((cls, wh2xy(box) * scale), axis=1)
                metric = compute_metric(output[:, :6], target, iou_v)
            # Append
            metrics.append((metric, output[:, 4], output[:, 5], cls.squeeze(-1)))

    # Compute metrics
    metrics = [np.concatenate(x, axis=0) for x in zip(*metrics)]
    if len(metrics) and metrics[0].any():
        tp, fp, m_pre, m_rec, map50, mean_ap = compute_ap(*metrics, names=params["names"])
    # Print results
    print(('%10s' + '%10.3g' * 4) % ('', m_pre, m_rec, map50, mean_ap))


# Visualization functions
def xywh2xyxy(x):
    """Convert [cx, cy, w, h] to [x1, y1, x2, y2]"""
    y = x.copy()
    y[:, 0] = x[:, 0] - x[:, 2] / 2  # x1
    y[:, 1] = x[:, 1] - x[:, 3] / 2  # y1
    y[:, 2] = x[:, 0] + x[:, 2] / 2  # x2
    y[:, 3] = x[:, 1] + x[:, 3] / 2  # y2
    return y


def resize_letterbox(img, target_size=(1024, 512)):
    target_w, target_h = target_size

    if img is None:
        raise ValueError("Input image is None")

    orig_h, orig_w = img.shape[:2]

    scale = min(target_w / orig_w, target_h / orig_h)

    new_w = int(orig_w * scale)
    new_h = int(orig_h * scale)

    # resize (OpenCV expects (W, H))
    img_resized = cv2.resize(
        img,
        (new_w, new_h),
        interpolation=cv2.INTER_LINEAR
    )

    # padded canvas
    padded_img = np.full(
        (target_h, target_w, 3),
        114,
        dtype=img.dtype
    )

    pad_x = (target_w - new_w) // 2
    pad_y = (target_h - new_h) // 2

    padded_img[
        pad_y:pad_y + new_h,
        pad_x:pad_x + new_w
    ] = img_resized

    return padded_img, scale, pad_x, pad_y


def make_visualization(prediction, img_cv):
    color_map = {
        1: (0, 0, 255),  # red
        2: (0, 255, 255),  # yellow
        3: (255, 255, 0)  # cyan
    }

    # img_cv = cv2.imread(input_image_filepath)
    for pred in prediction:
        x1, y1, x2, y2, conf, cls = pred

        # Pick color, fallback to white if unknown class
        color = color_map.get(int(cls), (255, 255, 255))

        x1, y1, x2, y2 = map(int, [x1, y1, x2, y2])
        # cv2.rectangle(img_cv, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.rectangle(img_cv, (x1, y1), (x2, y2), color, 2)
        # label = f"Class: {int(cls)} | Score: {conf:.2f}"
        # cv2.putText(img_cv, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    cv2.imshow('Prediction Objects', img_cv)
    cv2.waitKey(0)


def test_onnx_model(model_path, image_path):
    # Load model
    session = ort.InferenceSession(
        model_path,
        # providers=["CUDAExecutionProvider"]
        providers=["CPUExecutionProvider"]
    )

    # Get input name
    input_name = session.get_inputs()[0].name

    img = cv2.imread(image_path)
    image = cv2.imread(image_path)
    # Preprocess
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    image = image.astype(np.float32) / 255.0

    # HWC → CHW
    image = np.transpose(image, (2, 0, 1))

    # Add batch dim
    image = np.expand_dims(image, axis=0)
    # Run inference
    predictions = session.run(None, {input_name: image})[0]

    predictions = non_max_suppression(
        predictions,
        confidence_threshold=0.6,
        iou_threshold=0.45
    )[0]

    raw_predictions = predictions[0]

    # print(raw_predictions.min(), raw_predictions.max())
    # print(raw_predictions[:, 4:, :].min(), raw_predictions[:, 4:, :].max())

    # predictions = post_process_predictions(predictions[0])

    if predictions.size == 0:
        return []

    # --- adjust from letterboxed to original coords ---
    image, scale, pad_x, pad_y = resize_letterbox(img)
    predictions[:, [0, 2]] = (predictions[:, [0, 2]] - pad_x) / scale
    predictions[:, [1, 3]] = (predictions[:, [1, 3]] - pad_y) / scale

    # clamp to image bounds
    predictions[:, [0, 2]] = predictions[:, [0, 2]].clip(0, 1024)
    predictions[:, [1, 3]] = predictions[:, [1, 3]].clip(0, 512)

    make_visualization(predictions.tolist(), image)


def test_onnx_video(model_path, video_path):
    session = ort.InferenceSession(
        model_path,
        providers=["CPUExecutionProvider"]
    )

    input_name = session.get_inputs()[0].name

    cap = cv2.VideoCapture(video_path)

    while cap.isOpened():

        ret, frame = cap.read()
        if not ret:
            break

        orig_h, orig_w = frame.shape[:2]

        # LETTERBOX (IMPORTANT)
        img, scale, pad_x, pad_y = resize_letterbox(
            frame,
            target_size=(INPUT_WIDTH, INPUT_HEIGHT)
        )

        # PREPROCESS
        image = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        image = image.astype(np.float32) / 255.0
        image = np.transpose(image, (2, 0, 1))
        image = np.expand_dims(image, axis=0)

        # INFERENCE
        outputs = session.run(None, {input_name: image})[0]

        # NMS (same as evaluation!)
        predictions = non_max_suppression(
            outputs,
            confidence_threshold=0.6,
            iou_threshold=0.45
        )[0]

        if predictions.shape[0] > 0:
            # UNDO LETTERBOX
            predictions[:, [0, 2]] = (predictions[:, [0, 2]] - pad_x) / scale
            predictions[:, [1, 3]] = (predictions[:, [1, 3]] - pad_y) / scale

            # clamp
            predictions[:, [0, 2]] = predictions[:, [0, 2]].clip(0, orig_w)
            predictions[:, [1, 3]] = predictions[:, [1, 3]].clip(0, orig_h)

            # VISUALIZE
            draw_frame = frame.copy()
            make_visualization(predictions.tolist(), draw_frame)

        else:
            cv2.imshow("Prediction Objects", frame)

        # exit on 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


# QuantizeLinear -> DequantizeLinear
if __name__ == "__main__":
    parser = ArgumentParser(description="QTQ quantization for AutoSpeed 2.0")

    parser.add_argument('-q', '--quantize', action='store_true', required=False, help="Quantize model")
    parser.add_argument('-e', '--evaluate', action='store_true', required=False, help="Evaluate model")
    parser.add_argument('-t', '--test', action='store_true', required=False,
                        help="Test model inference and visualize results")
    parser.add_argument('-d', '--dataset', help="dataset directory path")
    parser.add_argument("-fp32onnx", "--fp32_onnx_model_path", dest="fp32_onnx_model_path",
                        default="autosteer_2.0_fp32.onnx",
                        help="FP32 ONNX model path")
    parser.add_argument("-int8onnx", "--int8_onnx_model_path", dest="int8_onnx_model_path",
                        default="autosteer_2.0_int8.onnx",
                        help="INT8 ONNX model path")
    parser.add_argument('-i', '--input', help="Input test image")

    args = parser.parse_args()
    dataset = args.dataset
    fp32_onnx_model_path = args.fp32_onnx_model_path
    int8_onnx_model_path = args.int8_onnx_model_path

    with open('../config/auto_speed.yaml', errors='ignore') as f:
        params = yaml.safe_load(f)

    if args.quantize:
        quantize(dataset, fp32_onnx_model_path, int8_onnx_model_path)
    elif args.evaluate:
        # evaluate(fp32_onnx_model_path, dataset, params)
        evaluate(int8_onnx_model_path, dataset, params)
    elif args.test:
        # test_onnx_model(args.int8_onnx_model_path, args.input)
        test_onnx_video(int8_onnx_model_path, '<VIDEO_PATH>')
    else:
        print("No quantization, evaluation or test parameters provided!")

    # model = onnx.load(fp32_onnx_model_path)
    # for i, node in enumerate(model.graph.node):
    #     print(f"\n[{i}]")
    #     print("name   :", node.name)
    #     print("op_type:", node.op_type)
    #     print("inputs :", list(node.input))
    #     print("outputs:", list(node.output))
