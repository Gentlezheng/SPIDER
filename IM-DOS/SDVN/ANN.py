from sklearn.preprocessing import StandardScaler #去量纲
import numpy as np
# 现在导入神经网络中的一个多分类模型，用于训练多分类数据
from sklearn.neural_network import MLPClassifier
# 现在导入sklearn中的用于评测预测结果指标的库，如混淆矩阵和分类报告
from sklearn.metrics import confusion_matrix, classification_report
from sklearn.model_selection import train_test_split
import EXP_ABS as Ea
import Global_Par as Gp
import SDVN_Controller as sc

class ANN:
    def __init__(self):
        self.X_data_array = []
        self.y_data_array = []
        self.X_tobePredict = []
        self.mlp = None

    def DataGet(self, controller, node_list, time):
        RG = controller.junction_matrix
        for u in RG.nodes():

            for u, v in RG.edges(u):  # u 对v 的评价, v 相对于u的距离变化率，速度变化率，连接时长
                # 确保距离小于250
                distance_uv = Ea.distance(node_list[u].position, node_list[v].position)
                if 0 < distance_uv < Gp.com_dis and u != v:

                    # 计算距离的变化率
                    trans_dist = (distance_uv - controller.feature_junction_matrix[u][v]['lat_dist_uv']) / distance_uv

                    # print('距离{}，上一距离{}'.format(distance_uv, controller.feature_junction_matrix[u][v]['lat_dist_uv']))
                    controller.feature_junction_matrix[u][v]['lat_dist_uv'] = distance_uv
                    # 方向是否变动
                    if node_list[u].direction == node_list[v].direction:
                        controller.feature_junction_matrix[u][v]['direction_is_same'] = 1  # 期间方向是否相同
                    else:
                        controller.feature_junction_matrix[u][v]['direction_is_same'] = 0

                    # Gvalue的变化率
                    pre_gvalue = Ea.gvlaue(v, RG)
                    Gvalue = (pre_gvalue - controller.feature_junction_matrix[u][v]['lat_gvalue']) / pre_gvalue
                    controller.feature_junction_matrix[u][v]['lat_gvalue'] = pre_gvalue
                    # print('---------函数内{}---------:'.format(RG[u][v]['lat_gvalue']))

                    # 加速度的变化率
                    pre_accel = Ea.distance(node_list[v].velocity, node_list[u].velocity)
                    if pre_accel != 0:
                        accel_rate = abs(pre_accel - controller.feature_junction_matrix[u][v]['lat_accel'] / pre_accel)
                    else:
                        accel_rate = 0
                    controller.feature_junction_matrix[u][v]['lat_accel'] = pre_accel

                    self.DataProcess(time,
                                node_list[u].node_id,
                                node_list[v].node_id,
                                0,
                                trans_dist,
                                controller.feature_junction_matrix[u][v]['direction_is_same'],
                                Gvalue,
                                accel_rate)
                    # with open('test/每一次的计算结果','a',encoding='utf-8') as f:
                    #     f.write('时间{}，节点{}向节点{}，距离变化率{}，方向是否变动{}，图值{}，加速度变化率{}\n'.format(time,node_list[u].node_id,
                    #             node_list[v].node_id,
                    #             trans_dist,
                    #             RG[u][v]['direction_is_same'],
                    #             Gvalue,
                    #             accel_rate))
                    #     f.close()

        # datasave(X_data_array, 0)
        scaler = StandardScaler()
        # global X_data_array
        print('-------------------')
        print(self.X_data_array)
        print('-------------------')
        X_data_array = np.array(self.X_data_array)
        scaler.fit(X_data_array[:, 3:7])
        X_tobePredict = scaler.transform(X_data_array[:, 3:7])
        # print('-------', X_tobePredict)
        X_predicted = self.mlp.predict(X_tobePredict)
        for i in range(len(X_tobePredict)):
            u = X_data_array[i, 1]
            v = X_data_array[i, 2]
            # if X_predicted[i] > 1:
            RG[u][v]['score'] = X_predicted[i]
            # else:
            #     G.remove_edge(u, v)
            # print(RG[u][v]['score'])
        # print(RG[X_data_array[1, 1]][X_data_array[1, 2]]['score'])
        self.X_data_array = []


    def Predict_mlp(self, X_file, y_file):
        X = np.loadtxt(X_file, delimiter=',')
        y = np.loadtxt(y_file, delimiter=',')
        X_train, X_test, y_train, y_test = self.preprocess(X[:, 3:7], y)
        self.mlp = self.GetPredict(X_train, y_train)
        print('预测模型完成')

    def GetPredict(self, X_train, y_train):
        mlp = MLPClassifier(hidden_layer_sizes=(400, 100), alpha=0.01, max_iter=3000)
        # 调用fit函数就可以进行模型训练，一般的调用模型函数的训练方法都是fit()
        mlp.fit(X_train, y_train)  # 这里y值需要注意，还原成一维数组
        return mlp

    def DataProcess(self, time, node_id_u, node_id_v, link_time, trans_distance_rate, direction_is_same, Gvalue_rate, accel_rate):
        # print('--x数据存储--')
        self.X_data_array.append([time,
                            node_id_u,
                            node_id_v,
                            trans_distance_rate,
                            direction_is_same,
                            Gvalue_rate,
                            accel_rate])
        self.y_data_array.append([link_time])

    def preprocess(self, X, y):
        X_train, X_test, y_train, y_test = train_test_split(X, y)
        # print(X_train)
        # 数据处理基本完毕，返回处理好的数据
        scaler = StandardScaler()
        scaler.fit(X_train)
        X_train = scaler.transform(X_train)
        X_test = scaler.transform(X_test)
        return X_train, X_test, y_train, y_test

    def DataGet2(self, u, v, distance_uv, time):

        # 计算距离的变化率
        v.trans_dist += (distance_uv - v.lat_dist_uv) / distance_uv
        v.lat_dist_uv = distance_uv

        # 方向是否变动
        if u.direction == v.direction:
            v.direction_is_same = 1  # 期间方向是否相同
        else:
            v.direction_is_same = 0

        # Gvalue的变化率,这的权值需要重写weight
        pre_gvalue = Ea.gvlaue2(u)
        v.Gvalue += (pre_gvalue - v.lat_gvalue) / pre_gvalue
        v.lat_gvalue = pre_gvalue

        # 加速度的变化率
        pre_accel = Ea.distance(v.velocity, u.velocity)
        if pre_accel != 0:
            v.accel_rate += abs(pre_accel - v.lat_accel / pre_accel)
        v.lat_accel = pre_accel

        self.DataProcess(time,
                    u.node_id,
                    v.node_id,
                    0,
                    v.trans_dist,
                    v.direction_is_same,
                    v.Gvalue,
                    v.accel_rate)

    def IMN_LIST_predict(self, node):
        scaler = StandardScaler()
        print('X数据：', self.X_data_array)
        X_data_array = np.array(self.X_data_array)
        scaler.fit(X_data_array[:, 3:7])
        X_tobePredict = scaler.transform(X_data_array[:, 3:7])
        X_predicted = self.mlp.predict(X_tobePredict)
        for i in range(len(X_tobePredict)):
            v = X_data_array[i, 2]
            node.IMN_LIST[v] = X_predicted[i]
        self.X_data_array = []


def NeuralNetwork(X_train, y_train, X_test, y_test):
    # 首先，创建一个多分类模型对象 类似于Java的类调用
    # 括号中填写多个参数，如果不写，则使用默认值，我们一般要构建隐层结构，调试正则化参数，设置最大迭代次数
    mlp = MLPClassifier(hidden_layer_sizes=(400, 100), alpha=0.01, max_iter=3000)
    # 调用fit函数就可以进行模型训练，一般的调用模型函数的训练方法都是fit()
    mlp.fit(X_train, y_train)  # 这里y值需要注意，还原成一维数组
    # 模型就这样训练好了，而后我们可以调用多种函数来获取训练好的参数
    # 比如获取准确率
    print('训练集的准确率是：', mlp.score(X_train, y_train))
    # 比如输出当前的代价值
    print('训练集的损失值是：', mlp.loss_)
    # 比如输出每个theta的权重
    # print('训练集的权重值是：', mlp.coefs_)
    predictions = mlp.predict(X_test)
    print('测试集预测：', predictions)
    # 混淆矩阵可以直观的看出分类中正确的个数和分错的个数，以及将正确的样本错误地分到了哪个类别
    matrix_train = confusion_matrix(y_train, mlp.predict(X_train))
    print('训练集的混淆矩阵是：', matrix_train)
    # 分类报告中有多个指标用于评价预测的好坏。
    '''
    TP: 预测为1(Positive)，实际也为1(Truth-预测对了)
    TN: 预测为0(Negative)，实际也为0(Truth-预测对了)
    FP: 预测为1(Positive)，实际为0(False-预测错了)
    FN: 预测为0(Negative)，实际为1(False-预测错了)
    '''
    report_train = classification_report(y_train, mlp.predict(X_train))
    print('训练集的分类报告是：', report_train)


def datasave(X_data_array, y_data_array, x_filename, y_filename):
    # print("-----数据存储-------")
    np.savetxt(x_filename, X_data_array, fmt='%.18e', delimiter=',')
    np.savetxt(y_filename, y_data_array, fmt='%.18e', delimiter=',')
    # print("-------存储完成-------")


