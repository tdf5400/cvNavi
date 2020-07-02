import cv2
import numpy as np
import math
import navigation.mathPack as mathPack

"""
路径计算方法整合
20.02.16 使用B、C点的直线斜率结合曲率计算负反馈进行斜率输出
         更改斜率判断规则，k>0-左边，k<0-右边
20.02.17 出田垄时返回的数据为“图像中田地占画面高的比例”
"""

## 状态标志
FIT_CROSS_STRAIGHT = 0  # 直道
FIT_CROSS_TRUN = 1  # 弯道
FIT_CROSS_OUT = 2  # 出林道
FIT_CROSS_ERR = -1  # 错误

## 设置参数
OUT_THRESHOLD = 0.65  # 出田垄进度计算-阈值

## 全局变量
Out_thre_cache = 0  # 使用mask法进行floodfill后，

Gaussian_distributionList = []  # 创建高斯数组，节省计算量
for i in range(480):
    Gaussian_distributionList.append(mathPack.Gaussian_distribution(i, 0, 100))
np.float32(Gaussian_distributionList)


# 用此变量向fitRoad_cross传递宽度


def fitRoad_Out_Conf(imgThre, threshold):
    """
    计算出田垄后，图像中田地占画面高的比例
    :param imgThre: 二值化图像
    :param threshold: 判定条件（百分数，0~1）
    （一行中有threshold%的值<127-->>道路）
    :return: 数值正确 - 道路/画面高度
             数值错误 - 0
    """
    # 合法性判断
    if type(imgThre).__name__ != 'ndarray' or (threshold < 0 or threshold > 1):
        return 0

    threshold *= 100
    copyImg = imgThre.copy()
    height, width = copyImg.shape[0:2]

    step = height / 100  # 步长
    times = 0  # 道路次数
    errTimes = 0  # 3次判定失败则退出
    # 行遍历
    data = copyImg[(height - 1)::-1, ]  # 切片
    for i in range(100):
        temp = data[int(i * step)]

        if np.percentile(temp, threshold) < 127:
            times += 1
            errTimes = 0  # 清空错误次数
        else:
            errTimes += 1
            if errTimes >= 3:
                break

    if times == 0:
        return 0
    else:
        return times / 100


def fitRoad_cross(imgThre, threshold, scanPercent=0.7, outroadThre=0.8):
    """
    计算路径（十字法）
    :param imgThre: 二值化图像
    :param threshold:  阈值（检测到顶部后，下移距离）
    :param scanPercent: 从扫描范围(0~1)从底部开始
    :param outroadThre: 图形高度*outroadTre 为判断出田垄的位置（从底部开始算），
                        从此位置检查左右两边的边缘
    :return: 状态, 数据
                    FIT_CROSS_STRAIGHT, 斜率（左-负 右-正）
                    FIT_CROSS_TRUN,     曲率（左-负 右-正）
                    FIT_CROSS_OUT,      0
                    FIT_CROSS_ERR,      错误号
    """
    assert 0 <= scanPercent <= 1, 'scanPercent应处于0到1之间！'
    assert 0 <= outroadThre <= 1, 'outroadThre应处于0到1之间！'

    copyImg = imgThre.copy()
    height, width = copyImg.shape[0:2]

    # 参数设定
    # W_cir - K值计算中圆曲率权值，K=K_BC-W_cir*K_cir
    W_cir = 0#.2
    # W_height - K值计算中距离权值，将与弯道的距离加入参考值
    W_height = 3 
    # Out_thre - 出田垄判断阈值，顶部有跳变时，两边跳变点
    #            与边缘距离均小于Out_thre判断为出田垄
    #            Out_thre_cache 用于配合floodfill的mask用法
    global Out_thre_cache
    Out_thre = Out_thre_cache + 3
    DISPLAY_PROCESS = 1  # 显示十字法计算轨迹（调试用）

    # 检测边缘
    edgeImg = cv2.adaptiveThreshold(copyImg, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 7, 0)
    # cv2.imshow('demo', edgeImg)

    # 从中点向上前进
    middlePoint = int(width / 2)
    edge = [0, width]  # [左边缘， 右边缘]
    for i in range(height - 1, int(height * (1 - scanPercent)) - 1, -1):
        if edgeImg.item((i, middlePoint)) == 255:  # 向上前进遇到边缘
            roadHeight = i
            startPoint = i + threshold if (i + threshold) < (height - 1) else (height - 2)
            # 向左检测（因为二值化图像经过高斯模糊，所以127也允许）
            for j in range(middlePoint, -1, -1):
                if edgeImg.item((startPoint, j)) >= 240:
                    edge[0] = j
                    break
            # 向右检测
            for j in range(middlePoint, width):
                if edgeImg.item((startPoint, j)) >= 240:
                    edge[1] = j
                    break

            ## 判断左右开口,取离中点近的值
            judge = 0  # 0-左，1-右
            distance_l, distance_r = abs(edge[0] - middlePoint), abs(edge[1] - middlePoint)
            # 跳变点均靠近边缘则判断为出田垄
            if (((width / 2) - distance_l) < Out_thre) and (((width / 2) - distance_r) < Out_thre):
                road_rate = fitRoad_Out_Conf(edgeImg, OUT_THRESHOLD)
                return FIT_CROSS_OUT, road_rate
            # 朝向判断
            if distance_l < distance_r:
                judge = 0
            else:
                judge = 1
            # 计算曲率
            # K = 1/r = (4 * Sabc) / (a*b*c)
            # Sabc = [(a+b+c)(a+b)(a+c)(b+c)]**0.5
            A = [height - 1, middlePoint]  # 曲线起点
            B = [startPoint, edge[judge]]  # 左/右交叉点
            C = (i, middlePoint)  # 顶点

            # 获取曲线起点【A点】（从两边开始）
            for j in range(0 if judge == 0 else (width - 1), middlePoint, 1 if judge == 0 else -1):
                if edgeImg.item(height - 1, j) == 255:
                    A[1] = j
                    break
            else:  # 底层找不到则上浮
                for j in range((height - 1), height - threshold, -1):
                    for k in range(middlePoint, width if judge == 0 else -1, 1 if judge == 0 else -1):
                        if edgeImg.item(j, k) == 255:
                            A = (j, k)
                            j = -1  # 退出双重循环
                            break
                    if j == -1:
                        break

            ## 计算BC斜率K_BC（竖直方向为x轴方向,左方向为y轴方向，底层中点为原点O）
            # 坐标系转换
            A = (height - A[0], width / 2 - A[1])
            B = (height - B[0], width / 2 - B[1])
            C = (height - C[0], width / 2 - C[1])
            if A == B == C:  # 数值正确性判断
                return FIT_CROSS_ERR, 0
            # 斜率计算
            K_BC = (C[1] - B[1]) / (C[0] - B[0])

            ## 计算K值
            K_AC = (C[1] - A[1]) / (C[0] - A[0])  # 计算AC斜率，判断是否在同一直线上
            if K_BC == K_AC:
                K = K_BC
            else:
                # 计算曲线上边界长度
                a = ((abs(B[0] - C[0])) ** 2 + (abs(B[1] - C[1])) ** 2) ** 0.5
                b = ((abs(A[0] - C[0])) ** 2 + (abs(A[1] - C[1])) ** 2) ** 0.5
                c = ((abs(B[0] - A[0])) ** 2 + (abs(B[1] - A[1])) ** 2) ** 0.5

                if a == 0 or b == 0 or c == 0:  # 发生错误
                    return FIT_CROSS_ERR, 0

                Sabc = ((a + b + c) * (a + b) * (a + c) * (b + c)) ** 0.5  # 三角形abc面积
                K_cir = (4 * Sabc) / (a * b * c)  # 曲率
                K_cir = abs(K_cir)  # 曲率取绝对值

                K = K_BC + W_cir * K_cir
				
            if K > 0:
                K = abs(K) + W_height*50/(height-roadHeight)
            else:
                K = -(abs(K) +W_height*50/(height-roadHeight))
            K = (distance_l - distance_r) / (height-roadHeight)

            # 轨迹显示功能（DEBUG）
            if DISPLAY_PROCESS:
                for i in range(height - 1, int(height * (1 - scanPercent)) - 1, -1):
                    if edgeImg.item((i, middlePoint)) == 255:  # 到达顶点
                        startPoint = i + threshold if (i + threshold) < (height - 1) else (height - 2)
                        cv2.line(edgeImg, (edge[0], startPoint), (edge[1], startPoint), 200)  # 画横线

                        cv2.imshow('edgeImg', edgeImg)
                        break
                    edgeImg[i, middlePoint] = 200

            return FIT_CROSS_TRUN, K

    # 遇不到边缘,区分“直道”与“出路口”
    else:
        edge = [0, width]  # [左边缘， 右边缘]
        # 画面顶部1/4出向左右两边巡线，若有线则为直道
        # 向左寻线
        for i in range(middlePoint, -1, -1):
            if edgeImg.item(int(height * (1 - outroadThre)), i):
                edge[0] = i
        # 向右巡线
        for i in range(middlePoint, width):
            if edgeImg.item(int(height * (1 - outroadThre)), i):
                edge[1] = i

        # 两边均无线则判断为出路口
        if edge == [0, width]:
            road_rate = fitRoad_Out_Conf(edgeImg, OUT_THRESHOLD)
            return FIT_CROSS_OUT, road_rate

        # 直道修正
        try:
            lines_theta = fitRoad_middle(copyImg, activation=False)  # 使用激活函数
            return FIT_CROSS_STRAIGHT, lines_theta
        except IOError:  # 错误报告
            return FIT_CROSS_ERR, -3
    return FIT_CROSS_ERR, -1


def backup_fitRoad_cross(imgThre, threshold, scanPercent=0.7, outroadThre=0.8):
    """
    计算路径（十字法）
    :param imgThre: 二值化图像
    :param threshold:  阈值（检测到顶部后，下移距离）
    :param scanPercent: 从扫描范围(0~1)从底部开始
    :param outroadThre: 图形高度*outroadTre 为判断出田垄的位置（从底部开始算），
                        从此位置检查左右两边的边缘
    :return: 状态, 数据
                    FIT_CROSS_STRAIGHT, 斜率（左-负 右-正）
                    FIT_CROSS_TRUN,     曲率（左-负 右-正）
                    FIT_CROSS_OUT,      0
                    FIT_CROSS_ERR,      错误号
    """
    assert 0 <= scanPercent <= 1, 'scanPercent应处于0到1之间！'
    assert 0 <= outroadThre <= 1, 'outroadThre应处于0到1之间！'

    copyImg = imgThre.copy()
    height, width = copyImg.shape[0:2]

    # 参数设定
    # W_cir - K值计算中圆曲率权值，K=K_BC-W_cir*K_cir
    W_cir = 0  # .2
    # W_height - K值计算中距离权值，将与弯道的距离加入参考值
    W_height = 3
    # Out_thre - 出田垄判断阈值，顶部有跳变时，两边跳变点
    #            与边缘距离均小于Out_thre判断为出田垄
    #            Out_thre_cache 用于配合floodfill的mask用法
    global Out_thre_cache
    Out_thre = Out_thre_cache + 3
    DISPLAY_PROCESS = 1  # 显示十字法计算轨迹（调试用）

    # 检测边缘
    edgeImg = cv2.adaptiveThreshold(copyImg, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 7, 0)
    # cv2.imshow('demo', edgeImg)

    # 从中点向上前进
    middlePoint = int(width / 2)
    edge = [0, width]  # [左边缘， 右边缘]
    for i in range(height - 1, int(height * (1 - scanPercent)) - 1, -1):
        if edgeImg.item((i, middlePoint)) == 255:  # 向上前进遇到边缘
            roadHeight = i
            startPoint = i + threshold if (i + threshold) < (height - 1) else (height - 2)
            # 向左检测（因为二值化图像经过高斯模糊，所以127也允许）
            for j in range(middlePoint, -1, -1):
                if edgeImg.item((startPoint, j)) >= 240:
                    edge[0] = j
                    break
            # 向右检测
            for j in range(middlePoint, width):
                if edgeImg.item((startPoint, j)) >= 240:
                    edge[1] = j
                    break

            ## 判断左右开口,取离中点近的值
            judge = 0  # 0-左，1-右
            distance_l, distance_r = abs(edge[0] - middlePoint), abs(edge[1] - middlePoint)
            # 跳变点均靠近边缘则判断为出田垄
            if (((width / 2) - distance_l) < Out_thre) and (((width / 2) - distance_r) < Out_thre):
                road_rate = fitRoad_Out_Conf(edgeImg, OUT_THRESHOLD)
                return FIT_CROSS_OUT, road_rate
            # 朝向判断
            if distance_l < distance_r:
                judge = 0
            else:
                judge = 1
            # 计算曲率
            # K = 1/r = (4 * Sabc) / (a*b*c)
            # Sabc = [(a+b+c)(a+b)(a+c)(b+c)]**0.5
            A = [height - 1, middlePoint]  # 曲线起点
            B = [startPoint, edge[judge]]  # 左/右交叉点
            C = (i, middlePoint)  # 顶点

            # 获取曲线起点【A点】（从两边开始）
            for j in range(0 if judge == 0 else (width - 1), middlePoint, 1 if judge == 0 else -1):
                if edgeImg.item(height - 1, j) == 255:
                    A[1] = j
                    break
            else:  # 底层找不到则上浮
                for j in range((height - 1), height - threshold, -1):
                    for k in range(middlePoint, width if judge == 0 else -1, 1 if judge == 0 else -1):
                        if edgeImg.item(j, k) == 255:
                            A = (j, k)
                            j = -1  # 退出双重循环
                            break
                    if j == -1:
                        break

            ## 计算BC斜率K_BC（竖直方向为x轴方向,左方向为y轴方向，底层中点为原点O）
            # 坐标系转换
            A = (height - A[0], width / 2 - A[1])
            B = (height - B[0], width / 2 - B[1])
            C = (height - C[0], width / 2 - C[1])
            if A == B == C:  # 数值正确性判断
                return FIT_CROSS_ERR, 0
            # 斜率计算
            K_BC = (C[1] - B[1]) / (C[0] - B[0])

            ## 计算K值
            K_AC = (C[1] - A[1]) / (C[0] - A[0])  # 计算AC斜率，判断是否在同一直线上
            if K_BC == K_AC:
                K = K_BC
            else:
                # 计算曲线上边界长度
                a = ((abs(B[0] - C[0])) ** 2 + (abs(B[1] - C[1])) ** 2) ** 0.5
                b = ((abs(A[0] - C[0])) ** 2 + (abs(A[1] - C[1])) ** 2) ** 0.5
                c = ((abs(B[0] - A[0])) ** 2 + (abs(B[1] - A[1])) ** 2) ** 0.5

                if a == 0 or b == 0 or c == 0:  # 发生错误
                    return FIT_CROSS_ERR, 0

                Sabc = ((a + b + c) * (a + b) * (a + c) * (b + c)) ** 0.5  # 三角形abc面积
                K_cir = (4 * Sabc) / (a * b * c)  # 曲率
                K_cir = abs(K_cir)  # 曲率取绝对值

                K = K_BC + W_cir * K_cir

            if K > 0:
                K = abs(K) + W_height * 50 / (height - roadHeight)
            else:
                K = -(abs(K) + W_height * 50 / (height - roadHeight))
            print('---', W_height / (height - roadHeight))

            # 轨迹显示功能（DEBUG）
            if DISPLAY_PROCESS:
                for i in range(height - 1, int(height * (1 - scanPercent)) - 1, -1):
                    if edgeImg.item((i, middlePoint)) == 255:  # 到达顶点
                        startPoint = i + threshold if (i + threshold) < (height - 1) else (height - 2)
                        cv2.line(edgeImg, (edge[0], startPoint), (edge[1], startPoint), 200)  # 画横线

                        cv2.imshow('edgeImg', edgeImg)
                        break
                    edgeImg[i, middlePoint] = 200

            return FIT_CROSS_TRUN, K

    # 遇不到边缘,区分“直道”与“出路口”
    else:
        edge = [0, width]  # [左边缘， 右边缘]
        # 画面顶部1/4出向左右两边巡线，若有线则为直道
        # 向左寻线
        for i in range(middlePoint, -1, -1):
            if edgeImg.item(int(height * (1 - outroadThre)), i):
                edge[0] = i
        # 向右巡线
        for i in range(middlePoint, width):
            if edgeImg.item(int(height * (1 - outroadThre)), i):
                edge[1] = i

        # 两边均无线则判断为出路口
        if edge == [0, width]:
            road_rate = fitRoad_Out_Conf(edgeImg, OUT_THRESHOLD)
            return FIT_CROSS_OUT, road_rate

        # 直道修正
        try:
            lines_theta = fitRoad_middle(copyImg, activation=False)  # 使用激活函数
            return FIT_CROSS_STRAIGHT, lines_theta
        except IOError:  # 错误报告
            return FIT_CROSS_ERR, -3
    return FIT_CROSS_ERR, -1

def fitRoad_middle(imgThre, scanPercent=0.7, activation=False):
    """
    计算路径（以每行）（适用于直道）
    :param imgThre: 路径的二值化图像
    :param roadHeight: 道路信息的高度（用于ROI）
    :param activation: 是否使用激活函数（高斯）
    :return: 路线斜率
    """
    # 参数设定
    DISPLAY_PROCESS = 1  # 显示计算路径（调试用）

    copyImg = imgThre.copy()
    height, width = copyImg.shape[0:2]
    width_middle = int(width / 2)  # 水平中心点
    road = np.zeros(copyImg.shape, dtype=np.uint8)  # 存储路径图像
    roadHeight = int(height * (1 - scanPercent))  # 道路信息的高度


    # 扫描
    points = np.zeros((height-roadHeight-1), dtype=np.int16)
    points[0] = width_middle

    for i in range(height - 2, roadHeight, -1):
        edge = [0, width - 1]

        # 从上个点的位置查找左边界
        for j in range(points[height - 2 - i], -1, -1):
            if copyImg.item((i, j)) == 0:
                edge[0] = j  # 记录边界
                break
        # 从上个点的位置查找右边界
        for j in range(points[height - 2 - i], width, 1):
            if copyImg.item((i, j)) == 0:
                edge[1] = j  # 记录边界
                break

        # 计算中心
        middle = int((edge[0] + edge[1]) / 2)
        points[height - i - 1] = middle  # 记录中心点

    # 坐标转换，与中心的差作为y值（直角坐标系逆时针旋转90°，底部中心点为原点）
    for i in range(len(points)):
        points[i] = width_middle - points[i]

    # 使用激活函数
    if activation:
        newList = [points[0], ]
        # 为节省计算量，事先计算好高斯数组
        P = 0.2 / Gaussian_distributionList[0]  # 比例系数
        for i in range(1, len(points)):
            # newList.append(newList[i - 1] + int(P * (points[i] - newList[i - 1]) *
            #                                     mathPack.Gaussian_distribution(i - 1, 0, 100)))
            newList.append(newList[i - 1] + int(P * (points[i] - newList[i - 1]) *
                                                Gaussian_distributionList[i - 1]))
        points = newList

    # 路径显示功能（DEBUG）
    if DISPLAY_PROCESS:
        for i in range(0, len(points)):
            road.itemset((height - 1 - i, width_middle - points[i]), 255)
        cv2.imshow('Road', road)

    # 一元线性回归,拟合直线 (yi = a + b * xi)
    lenth = len(points)
    x_average = lenth * (lenth + 1) / (2 * lenth)  # x平均值
    y_average = np.mean(points)  # y平均值
    b_numerator = 0     # 斜率b的分子
    b_denominator = 0   # 斜率b的分母

    # 计算b的分子
    for i in range(0, len(points)):
        b_numerator += (i - x_average) * (points[i] - y_average)
        b_denominator += (i - x_average)**2

    # 直接计算b
    b = b_numerator / b_denominator  # K>0-向左,K<0-向右
    a = y_average - b*x_average

    print('y = {}x + {}'.format(b, a))
    direct_K = b + 0.005*a
    return direct_K


def hough(imgEdge, src=None):
    """
        Hough处理
        :param imgEdge: 边界图像
        :param src: 画布
        :return:画线的图像，拟合角度
        """
    # 若无输入画布则为黑布
    if src is None:
        src = np.zeros(imgEdge.shape, dtype=np.uint8)

    lines = cv2.HoughLines(imgEdge, 1, np.pi / 180 * 1, 19)
    if lines is not None:
        __ground = np.zeros(imgEdge.shape, dtype=np.uint8)
        line_direct_theta = 0  # 记录角度
        line_direct_times = 0  # 取平均用
        for i in range(len(lines)):
            rho = lines[i][0][0]
            theta = lines[i][0][1]
            a = math.cos(theta)
            b = math.sin(theta)
            x0 = a * rho
            y0 = b * rho
            # y0=kx0+b    rho=
            pt1 = (int(x0 + 600 * (-b)), int(y0 + 600 * a))
            pt2 = (int(x0 - 600 * (-b)), int(y0 - 600 * a))

            if not theta == 0:
                __lineTheta = math.atan(-1 / math.tan(theta)) / np.pi * 180  # 直线角度
            else:
                __lineTheta = 0
            # 显示特定角度的直线
            if not -15 < __lineTheta < 15 or 75 < line_direct_theta < 105:
                cv2.line(src, pt1, pt2, 255, 3, cv2.LINE_AA)
                line_direct_theta += __lineTheta % 180
                line_direct_times += 1
                # 打印每条线的信息
                # print(rho, __lineTheta)

        # 计算指引线
        if line_direct_times is not 0:
            line_direct_theta = line_direct_theta / line_direct_times
            line_direct_output = line_direct_theta
            # print(f'direct:{line_direct_output}')
            return src, line_direct_output
    return None, None



