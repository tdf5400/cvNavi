import cv2
import numpy as np


def floodFill(image, loDiff, upDiff, mask_wide=0):
    """
    floodFill 计算地面方法
    :param image:Any
    :param loDiff:最低阈值（三变量turple，如：(20, 100, 255)）
    :param upDiff:最大阈值（三变量turple，如：(40, 150, 255)）
    :param mask_wide:   道路感兴趣区域宽度（用于排除扫描到路面外的干扰），
                        0 - 不启用
    :return: copyImg, threImg
    """
    # __FILLCOLOR - floodfill时填充的颜色
    __FILLCOLOR = (255, 255, 255)  # 绿色
    __FURTHERMORPH = 0  # 更精确的形态学运算开关

    # 预处理
    # copyImg = image
    copyImg = cv2.medianBlur(image, 3)
    # copyImg = cv2.GaussianBlur(image, (29, 29), sigmaX=0)

    # BGR转换为HSV进行处理
    copyImg = cv2.cvtColor(copyImg, cv2.COLOR_BGR2HSV)

    h, w = image.shape[:2]
    mask = np.zeros([h + 2, w + 2], dtype=np.uint8)  # mask必须行和列都加2，且必须为uint8单通道阵列
    # 规划道路区域为感兴趣区域（排除扫描到路面外的干扰）
    global Out_thre_cache
    if not mask_wide == 0:
        mask[:, 320 - mask_wide:320 + mask_wide] = 255
        mask = cv2.bitwise_not(mask)

        Out_thre_cache = w / 2 - mask_wide  # 更新mask值
    else:
        Out_thre_cache = 0  # 更新mask值

    # 计算种子点
    if mask_wide == 0:
        seedThreshold = int(h * w / 5)  # 20000   # 最少像素值（只取感兴趣区域）
    else:
        seedThreshold = int(h * (2 * mask_wide) / 6)  # 20000   # 最少像素值（只取感兴趣区域）
    timesLimit = 5  # 计算次数限制
    seed = [int(w / 2) - 1, h - 1]  # 以画面中间最下面的点为起始点 （x, y）
    times = 0  # 循环次数，若超过阈值则返回(None,None)
    seedMoveDistance = int((h-1) * 0.8 / timesLimit)  # 失败后上升的距离（限高0.5）

    while True:
        # floodFill
        # --------------------------------19.12.21 阈值数据备份--------------------------------
        # cv2.floodFill(copyImg, mask, tuple(seed), (255, 255, 255), (20, 100, 255), (40, 150, 255),
        #               flags=cv2.FLOODFILL_FIXED_RANGE)
        # ------------------------------------------------------------------------------------
        cv2.floodFill(copyImg, mask, tuple(seed), __FILLCOLOR, loDiff, upDiff,
                      flags=cv2.FLOODFILL_FIXED_RANGE)

        # 二值化并统计渲染数量
        threImg = cv2.inRange(copyImg, copyImg[seed[1], seed[0]], copyImg[seed[1], seed[0]])  # 将与种子点一样被染色的点划出来
        threCounter = np.sum(threImg == 255)  # 统计出现的数量

        # 退出的判定（大于阈值且不等于ROI面积）
        if threCounter >= seedThreshold and \
                ((mask_wide == 0 and threCounter < (h * w))
                 or (mask_wide and threCounter < (h * (2 * (mask_wide*0.95))))):
            break

        else:
            times += 1
            if times < timesLimit:
                seed[1] -= seedMoveDistance  # seed上移
            else:
                return None, None

    # 形态学运算
    if __FURTHERMORPH:  # 精确的形态学运算
        # morph_size - morphImg外扩距离(单边)
        # morphImg - 用于避免边缘导致的形态学运算错误
        morph_size = 20
        morphImg = np.zeros((h + morph_size * 2, w + morph_size * 2), dtype=np.uint8)
        for i in range(h):
            for j in range(w):
                morphImg.itemset(morph_size + i, morph_size + j, threImg.item(i, j))
        threImg = morphImg
    kernel = np.ones((65, 75), dtype=np.uint8)
    threImg = cv2.morphologyEx(threImg, cv2.MORPH_CLOSE, kernel)
    # kernel = np.ones((35, 35), dtype=np.uint8)
    # threImg = cv2.morphologyEx(threImg, cv2.MORPH_OPEN, kernel)
    kernel = np.ones((7, 7), dtype=np.uint8)
    threImg = cv2.morphologyEx(threImg, cv2.MORPH_ERODE, kernel)
    if __FURTHERMORPH:  # 精确的形态学运算
        # 取原来图像的框
        temp = threImg.copy()
        threImg = np.zeros((h, w), dtype=np.uint8)
        for i in range(h):
            for j in range(w):
                threImg.itemset(i, j, temp.item(morph_size + i, morph_size + j))

    # 色彩空间转换BGR
    copyImg = cv2.cvtColor(copyImg, cv2.COLOR_HSV2BGR)

    return copyImg, threImg



