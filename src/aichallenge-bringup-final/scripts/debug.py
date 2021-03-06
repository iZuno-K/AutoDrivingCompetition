import numpy as np


def line_calculator(p):
    # x, y
    p = np.array(p)
    return p


def line(p):
    diff = p[1] - p[0]
    a = diff[1] / diff[0] 
    b = - a * p[0, 0] + p[0, 1]
    print("y = {} x + {}".format(a, b))
    return a, b

if __name__=='__main__':
    # 対向車線の奥の線
    print("y0")
    # p = line_calculator([[36.7160644531, -370.767303467], [-92.1157989502, -337.493499756]])
    p = line_calculator([[-35.3672790527, -352.694793701], [-61.1725769043, -346.60546875]])  # あえてちょいとだけ奥側に設定
    # p = line_calculator([[-40.4277877808, -351.821044922], [-66.4262390137, -345.056396484]])  # あえて少し奥側に設定
    a0, b0 = line(p)
    ans = a0 * -48.9190292358 + b0
    print(-348.149719238 - ans)

    # 対向車線の手前の線
    print("y1")
    p = line_calculator([[37.9616317749, -365.977478027], [ -90.7057418823, -331.969696045]])
    a1, b1 = line(p)
    ans = a1 * -61.4508514404 + b1
    print(-339.482116699 - ans)

    # 最初の交差点の領域
    print("y2 Intersection1")
    p = line_calculator([[-103.473014832, -321.610961914], [-106.013801575, -320.982116699]])
    a2, b2 = line(p)
    x =  -105.431488037
    y = -324.500427246
    ans = a2 * x + b2
    print("y1:{} < y: {} < y2: {}".format(a1 * x + b1, y, ans))
    x = -105.569129944
    y =  -326.137634277
    ans = a2 * x + b2
    print("y1:{} < y: {} < y2: {}".format(a1 * x + b1, y, ans))


    # 奥の交差点の領域
    print("y3 Intersection2")
    # p = line_calculator([[-47.9738807678, -348.397003174], [-49.2040596008, -353.227081299]])
    p = line_calculator([[-52.7878379822, -347.158966064], [-53.9083900452, -352.149200439]])  # larger area than the above line
    a3, b3 = line(p)
    x = -45.7707633972
    y = -351.089508057
    ans = a3 * x + b3
    print("y:{} < y0: {} and y:{} < y3: {}".format(y, a0 * x + b0, y, ans))
    x = -38.4141120911
    y = -353.909057617
    ans = a3 * x + b3
    print("y:{} < y0: {} and y:{} < y3: {}".format(y, a0 * x + b0, y, ans))

    # 奥の対向車の領域判定
    print("y4")
    p = line_calculator([[-35.2807617188, -346.170806885], [-36.5775184631, -351.088165283]])
    # p = line_calculator([[-29.5161151886, -347.947814941], [-30.8802165985, -352.442504883]])  # 一車線分上側に寄せる
    a4, b4 = line(p)
    x = -33.8878211975
    y = -349.160888672
    ans = a4 * x + b4
    print("y: {} < y4: {}".format(y, ans))
    x = -5.38962364197
    y = -356.442047119
    ans = a4 * x + b4
    print("y: {} < y4: {}".format(y, ans))

    # 交差点1の車の領域
    print("y5")
    # p = line_calculator([[-107.047317505, -327.142974854], [-105.749717712, -321.952575684]])
    p = line_calculator([[-103.367607117, -321.516937256], [-105.986793518, -320.537536621]])  # もうちょい広げる
    a5, b5 = line(p)
    x = -102.349273682
    y = -330.886810303
    ans = a5 * x + b5
    print("y: {} < y5: {}".format(y, ans))
    x = -90.7893295288
    y = -334.763946533
    ans = a5 * x + b5
    print("y: {} < y5: {}".format(y, ans))

# y1
# y = -0.2582730920132578 x + -361.2845319741398
# y2
# y = -0.26430773408498043 x + -355.9439251504078
# y3
# y = -0.24750019525741684 x + -347.22055328879355
# y4
# y = 3.9263219260739675 x + -160.03610323652867
# y5
# y = 3.7920438197948 x + -212.38461245157148