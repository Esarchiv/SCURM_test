from openvino.runtime import Core
import numpy as np
import cv2, time


CLASSES = {0: 'blue_hit', 1: 'blue_R', 2: 'blue_miss', 3: 'red_miss', 4: 'red_hit', 5: 'red_R'}
colors = [(255, 0, 0), (0, 0, 255), (255, 255, 0), (0, 255, 255), (255, 0, 255), (0, 255, 0)]


object_points = np.array([
    [0, 0.1700, 0.1750],
    [0, -0.1700, 0.1750],
    [0, -0.1850, -0.1650],
   # [0, 0, -0.7150],
    [0, 0.1850, -0.1650]
], dtype=np.float32)


camera_matrix = np.array([
    [2075.15666, 0.0, 646.02307],
    [0.0, 2073.92438, 479.8963],
    [0.0, 0.0, 1.0]
], dtype=np.float32)

dist_coeffs = np.array([-0.051148, 0.231678, 0.000775, 0.002697, 0.0], dtype=np.float32)

def draw_bounding_box(img, class_id, confidence, x, y, x_plus_w, y_plus_h):
    label = f'{CLASSES[class_id]} ({confidence:.2f})'
    color = colors[class_id]
    cv2.rectangle(img, (x, y), (x_plus_w, y_plus_h), color, 2)
    cv2.putText(img, label, (x - 10, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

def draw_key_points(img, key_points):
    key_points = np.array(key_points).reshape((-1, 2))  # Adjusted to reshape into (-1, 2)
    for i, key_point in enumerate(key_points):
        points = key_point[0:2]
        cv2.circle(img, tuple(points.astype(int)), 5, colors[i % len(colors)], -1)

def prepare_input(image):
    input_img = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    input_img = cv2.resize(input_img, (640, 640))
    input_img = input_img / 255.0
    input_img = input_img.transpose(2, 0, 1)
    input_tensor = input_img[np.newaxis, :, :, :].astype(np.float32)
    return input_tensor, image.shape[0], image.shape[1]

def process_output(outputs, img_height, img_width):
    outputs = np.array([cv2.transpose(outputs[0])])
    rows = outputs.shape[1]
    boxes = []
    scores = []
    preds_kpts = []
    for i in range(rows):
        x, y, w, h = outputs[0][i][:4]
        class_scores = outputs[0][i][4:10]
        key_points = outputs[0][i][10:] 

        # 获取最高的分类置信度及其索引
        class_id = np.argmax(class_scores)
        confidence = class_scores[class_id]

        if confidence >= 0.1:  # 阈值
            x = int(x * img_width / 640)
            y = int(y * img_height / 640)
            w = int(w * img_width / 640)
            h = int(h * img_height / 640)
            key_points = key_points.reshape(-1, 2) * [img_width / 640, img_height / 640]

            box = [x - (0.5 * w), y - (0.5 * h), x + (0.5 * w), y + (0.5 * h)]
            boxes.append(box)
            scores.append(confidence)
            preds_kpts.append((class_id, key_points))  # 将class_id和key_points一起存储

    return boxes, scores, preds_kpts

def apply_nms(boxes, scores, iou_threshold=0.6):
    indices = cv2.dnn.NMSBoxes(boxes, scores, score_threshold=0.5, nms_threshold=iou_threshold)
    return indices

# Load ONNX model
core = Core()
model = core.read_model('/home/esarchiv/Desktop/1/best_csf.onnx')
compiled_model = core.compile_model(model, 'CPU')
output_node = compiled_model.outputs[0]
ir = compiled_model.create_infer_request()

# Open video file
cap = cv2.VideoCapture('/home/esarchiv/Desktop/testfile/test_video.mp4')

# Get video properties
frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
fps = cap.get(cv2.CAP_PROP_FPS)

out = cv2.VideoWriter('/home/esarchiv/Desktop/output_video.mp4', cv2.VideoWriter_fourcc(*'mp4v'), fps, (frame_width, frame_height))

while cap.isOpened():
    start = time.time()
    ret, frame = cap.read()
    if not ret:
        break

    input_tensor, img_height, img_width = prepare_input(frame)
    outputs = ir.infer(input_tensor)[output_node]
    boxes, scores, preds_kpts = process_output(outputs, img_height, img_width)

    # Apply NMS
    nms_indices = apply_nms(boxes, scores)
    if len(nms_indices) > 0:
        nms_indices = nms_indices.flatten()

    for i in nms_indices:
        box = boxes[i]
        class_id, key_points = preds_kpts[i]  # 获取class_id和key_points
        x1, y1, x2, y2 = map(int, box)
        draw_bounding_box(frame, class_id, scores[i], x1, y1, x2, y2)
        draw_key_points(frame, key_points)

        # 使用PnP算法求解位姿
        if len(key_points) == 5:
            success, rotation_vector, translation_vector = cv2.solvePnP(object_points, key_points, camera_matrix, dist_coeffs)
            if success:
                # 将旋转向量转换为旋转矩阵
                rotation_matrix, _ = cv2.Rodrigues(rotation_vector)
                
                # 输出位姿信息
                print("旋转向量：\n", rotation_vector)
                print("旋转矩阵：\n", rotation_matrix)
                print("平移向量：\n", translation_vector)
 

    end = time.time()
    fps = (1 / (end - start))
    fps_label = "fps: %.2f " % fps
    cv2.putText(frame, fps_label, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

    out.write(frame)

    cv2.imshow('buff_info', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break


cap.release()
out.release()
cv2.destroyAllWindows()