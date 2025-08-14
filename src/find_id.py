import cv2, glob, os
# 寻找多个可能的路径
search_paths = [
    '/home/wenhao/uf_custom_ws/board.jpg',
    '/home/wenhao/uf_custom_ws/*.jpg',
    '/home/wenhao/uf_custom_ws/*.png', 
    '/home/wenhao/.ros/*.jpg',
    '/home/wenhao/.ros/*.png'
]
paths = []
for pattern in search_paths:
    paths.extend(glob.glob(pattern))

if not paths:
    print("未找到图片文件！请先保存一张 ChArUco 板的图片")
    print("方法1: ros2 run image_view image_saver --ros-args -r image:=/camera/camera/color/image_raw")
    print("方法2: 拍照保存为 board.jpg 放到工作区")
    raise SystemExit

img_path = sorted(paths)[-1]
print(f"使用图片: {img_path}")
img = cv2.imread(img_path); gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
cands = {"DICT_4X4_50":cv2.aruco.DICT_4X4_50,"DICT_4X4_100":cv2.aruco.DICT_4X4_100,
         "DICT_4X4_250":cv2.aruco.DICT_4X4_250,"DICT_4X4_1000":cv2.aruco.DICT_4X4_1000}
best=None
for name,val in cands.items():
    d=cv2.aruco.getPredefinedDictionary(val)
    params = cv2.aruco.DetectorParameters_create()
    corners, ids, _ = cv2.aruco.detectMarkers(gray, d, parameters=params)
    n = 0 if ids is None else len(ids)
    print(f"{name}: {n} markers")
    if best is None or n > best[1]:
        best=(name,n,corners,ids,d)
if best[1]==0: print("未检测到任何 marker"); raise SystemExit
print(f"Best: {best[0]}, IDs={best[3].flatten().tolist()}")
cv2.aruco.drawDetectedMarkers(img, best[2], best[3])
out=os.path.expanduser('/home/wenhao/uf_custom_ws/aruco_annotated.jpg'); cv2.imwrite(out,img); print("Saved:", out)