"""
PID算法
"""


class pidParam:
    P = 0
    I = 0
    D = 0
    delta_t = 1         # Δt
    __integration = 0     # 积分
    __val_now = None      # 现在的值
    __val_last = None     # 旧值

    def __init__(self, P, I, D, delta_t=1):
        self.P = P
        self.I = I
        self.D = D
        self.delta_t = delta_t

    def __call__(self, now):
        """
        计算结果
        :param now: 新值
        :return: None-未初始化 其他-PID计算结果
        """
        if self.fresh(now):
            return None
        error = self.__val_now - self.__val_last
        self.__integration += error
        output = self.P * error + self.I * self.__integration + self.D * (error / self.delta_t)
        return output

    def fresh(self, now):
        """
        更新新值、旧值
        :param now: 新值
        :return: True-还需再次写入新值 False-成功
        """
        self.__val_last = self.__val_now
        self.__val_now = now
        if self.__val_last is None:
            return True
        return False




