# ===================================================================

from ultralytics import YOLO
import torch
import cv2

# 1) 모델 로드
model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)

# 2) 이미지 읽기 (절대 경로로)
#img_path = '/home/a/catkin_ws/src/provin_ground/script/IMG_5784.jpg'
# 신호등 데이터
#img_path = '/home/a/catkin_ws/src/ctrack_img/001294.jpg'
# 사람 + 신호등 데이터
img_path = '/home/a/catkin_ws/src/ctrack_img/000654.jpg'
img = cv2.imread(img_path)
if img is None:
    raise FileNotFoundError(f"이미지 없음: {img_path}")

# 2.1) 화면 표시용으로 리사이즈 (가로 최대 800px)
max_w = 800
h, w = img.shape[:2]
if w > max_w:
    scale = max_w / w
    img = cv2.resize(img, (int(w * scale), int(h * scale)))

# 3) 추론
results = model(img)

# 4) DataFrame 추출
df = results.pandas().xyxy[0]
tl_df = df[df['name']=='traffic light']
print(tl_df)

# 5) 바운딩박스 그리기
for _, row in tl_df.iterrows():
    x1, y1 = int(row.xmin), int(row.ymin)
    x2, y2 = int(row.xmax), int(row.ymax)
    conf = row.confidence
    cv2.rectangle(img, (x1,y1), (x2,y2), (0,0,255), 2)
    cv2.putText(img, f"TL {conf:.2f}", (x1, y1-10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,255), 2)

# 6) 결과 보여주기
cv2.imshow('Traffic Light Detection', img)
cv2.waitKey(0)
cv2.destroyAllWindows()

# 7) 플래그 출력
flag = not tl_df.empty
print("Red light detected:", flag)
