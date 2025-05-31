import cv2

cap = cv2.VideoCapture(0, cv2.CAP_V4L2)

# 必须设置的参数
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M','J','P','G'))  # MJPG编码
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

if not cap.isOpened():
    print("摄像头打开失败")
else:
    print("尝试读取帧...")
    ret, frame = cap.read()
    if ret:
        cv2.imwrite("test.jpg", frame)
        print("截图保存成功")
    else:
        print(f"读取失败，错误代码：{cap.get(cv2.CAP_PROP_POS_MSEC)}")

cap.release()