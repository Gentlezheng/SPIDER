from sklearn.preprocessing import StandardScaler #Dimensionality
import numpy as np
# Now import a multi-class model in the neural network for training multi-class data
from sklearn.neural_network import MLPClassifier
# Now import libraries in sklearn for evaluating prediction result indicators, such as confusion matrix and classification report
from sklearn.metrics import confusion_matrix, classification_report
from sklearn.model_selection import train_test_split
import EXP_ABS as Ea
import Global_Par as Gp
import math
import SDVN_Controller as sc

class ANN:
    def __init__(self):
        self.X_data_array = []
        self.y_data_array = []
        self.X_tobePredict = []
        self.mlp = None

    def def_angle(x0, y0, x1, y1, x2, y2):
        if x0 == x1 and y0 == y1:  # Did not move
            return 0
        cos = ((x1 - x0) * (x2 - x0) + (y1 - y0) * (y2 - y0)) / (math.sqrt(pow(x1 - x0, 2) + pow(y1 - y0, 2)) * math.sqrt(pow(x2 - x0, 2) + pow(y2 - y0, 2)))

        return cos
    def DataGet(self, controller, node_list, time):
        RG = controller.junction_matrix
        for u in RG.nodes():

            for u, v in RG.edges(u):  # u's evaluation of v, v relative to u's distance change rate, speed change rate, connection time
                # Make sure the distance is less than 250
                distance_uv = Ea.distance(node_list[u].position, node_list[v].position)
                if 0 < distance_uv < Gp.com_dis and u != v:

                    # Calculate the rate of change of distance
                    trans_dist = (distance_uv - controller.feature_junction_matrix[u][v]['lat_dist_uv']) / distance_uv

                    controller.feature_junction_matrix[u][v]['lat_dist_uv'] = distance_uv
                    # 方向是否变动
                    if node_list[u].direction == node_list[v].direction:
                        # Is the direction the same during the period
                        controller.feature_junction_matrix[u][v]['direction_is_same'] = 1  
                    else:
                        controller.feature_junction_matrix[u][v]['direction_is_same'] = 0

                    # Gvalue change rate
                    pre_gvalue = Ea.gvlaue(v, RG)
                    Gvalue = (pre_gvalue - controller.feature_junction_matrix[u][v]['lat_gvalue']) / pre_gvalue
                    controller.feature_junction_matrix[u][v]['lat_gvalue'] = pre_gvalue
                    # print('---------函数内{}---------:'.format(RG[u][v]['lat_gvalue']))
                    # Angle cosine change rate
                    cos_angle_u_sds = ANN.def_angle(node_list[u].lat_position[0], node_list[u].lat_position[1],
                                                node_list[u].position[0], node_list[u].position[1],
                                                node_list[v].lat_position[0], node_list[v].lat_position[1])
                    cos_angle_u_sdd = ANN.def_angle(node_list[u].position[0], node_list[u].position[1],
                                                node_list[u].lat_position[0], node_list[u].lat_position[1],
                                                node_list[v].position[0], node_list[v].position[1],)

                    cos_angle_v_sds = ANN.def_angle(node_list[v].lat_position[0], node_list[v].lat_position[1],
                                                node_list[v].position[0], node_list[v].position[1],
                                                node_list[u].lat_position[0], node_list[u].lat_position[1])
                    cos_angle_v_sdd = ANN.def_angle(node_list[v].position[0], node_list[v].position[1],
                                                node_list[v].lat_position[0], node_list[v].lat_position[1],
                                                node_list[u].position[0], node_list[u].position[1],)
                    controller.feature_junction_matrix[u][v]['lat_angle'] = np.linalg.det(
                        np.array([[cos_angle_u_sds, cos_angle_u_sdd], [cos_angle_v_sds, cos_angle_v_sdd]]))


                    # Rate of change of acceleration
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
                                accel_rate,
                                controller.feature_junction_matrix[u][v]['lat_angle'])

        scaler = StandardScaler()
        print('-------------------')
        print(self.X_data_array)
        print('-------------------')
        X_data_array = np.array(self.X_data_array)
        scaler.fit(X_data_array[:, 3:7])
        X_tobePredict = scaler.transform(X_data_array[:, 3:7])
        X_predicted = self.mlp.predict(X_tobePredict)
        for i in range(len(X_tobePredict)):
            u = X_data_array[i, 1]
            v = X_data_array[i, 2]
            RG[u][v]['score'] = X_predicted[i]
        self.X_data_array = []


    def Predict_mlp(self, X_file, y_file):
        X = np.loadtxt(X_file, delimiter=',')
        y = np.loadtxt(y_file, delimiter=',')
        X_train, X_test, y_train, y_test = self.preprocess(X[:, 3:7], y)
        self.mlp = self.GetPredict(X_train, y_train)
        print('Predictive model completed')

    def GetPredict(self, X_train, y_train):
        mlp = MLPClassifier(hidden_layer_sizes=(400, 100), alpha=0.01, max_iter=3000)
        # Model training can be performed by calling the fit function. The general training method for calling the model function is fit()
        mlp.fit(X_train, y_train)  # Here the y value needs to be paid attention to, restored to a one-dimensional array
        return mlp

    def DataProcess(self, time, node_id_u, node_id_v, link_time, trans_distance_rate, direction_is_same, Gvalue_rate, accel_rate, angle):
        # print('--x数据存储--')
        self.X_data_array.append([time,
                            node_id_u,
                            node_id_v,
                            trans_distance_rate,
                            direction_is_same,
                            Gvalue_rate,
                            accel_rate,
                            angle])
        self.y_data_array.append([link_time])

    def preprocess(self, X, y):
        X_train, X_test, y_train, y_test = train_test_split(X, y)
        # print(X_train)
        # Data processing is basically completed, return the processed data
        scaler = StandardScaler()
        scaler.fit(X_train)
        X_train = scaler.transform(X_train)
        X_test = scaler.transform(X_test)
        return X_train, X_test, y_train, y_test

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
    # First, create a multi-class model object, similar to the Java class call
    # Fill in multiple parameters in brackets. If you don’t write them, use the default values. We generally need to build a hidden layer structure, debug regularization parameters, and set the maximum number of iterations.
    mlp = MLPClassifier(hidden_layer_sizes=(400, 100), alpha=0.01, max_iter=3000)
    # 调用fit函数就可以进行模型训练，一般的调用模型函数的训练方法都是fit()
    mlp.fit(X_train, y_train)  # Here the y value needs to be paid attention to, restored to a one-dimensional array
    # The model is trained like this, and then we can call a variety of functions to get the trained parameters
    # Such as obtaining accuracy
    print('训练集的准确率是：', mlp.score(X_train, y_train))
    # For example, output the current cost value
    print('训练集的损失值是：', mlp.loss_)
    # 比如输出每个theta的权重
    # print('训练集的权重值是：', mlp.coefs_)
    predictions = mlp.predict(X_test)
    print('测试集预测：', predictions)
    # The confusion matrix can intuitively see the correct number and the number of errors in the classification, as well as which category the correct sample is incorrectly classified into
    matrix_train = confusion_matrix(y_train, mlp.predict(X_train))
    print('训练集的混淆矩阵是：', matrix_train)
    # There are multiple indicators in the classification report to evaluate the quality of the prediction.
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


