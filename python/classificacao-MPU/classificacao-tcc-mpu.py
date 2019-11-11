import numpy as np
import sklearn.preprocessing as pre
from sklearn.neural_network import MLPClassifier
from sklearn.model_selection import train_test_split
from sklearn.metrics import accuracy_score
from sklearn.metrics import confusion_matrix
from sklearn.metrics import classification_report


def GenerateCodeToC(MLP):
    coefs_z = np.array(MLP.coefs_[0])
    coefs_u = np.array(MLP.coefs_[1])
    numberOfInputs = len(coefs_z)
    numberOfHidden = len(coefs_z[0])
    numberofOutputs = len(coefs_u[0])

    print("#define NUMBER_OF_INPUTS_A               ", end="")
    print(numberOfInputs)
    print("#define NUMBER_OF_HIDDEN_A               ", end="")
    print(numberOfHidden)
    print("#define NUMBER_OF_OUTPUTS_A              ", end="")
    print(numberofOutputs)
    print("")

    print("float weightInputHiddenA[NUMBER_OF_INPUTS_A][NUMBER_OF_HIDDEN_A] = ")
    print("{")
    for i in coefs_z:
        print("{", end="")
        for ii in i:
           print("%2.6f" % (ii), end=", ")
           #print(ii, end=", ")
        print("},")
    print("};")

    print("float weightHiddenOutputA[NUMBER_OF_HIDDEN_A][NUMBER_OF_OUTPUTS_A] =")
    print("{")
    for i in coefs_u:
        print("{", end="")
        for ii in i:
            print("%2.6f" % (ii), end=", ")
            #print(ii, end=", ")
        print("},")
    print("};")

    bias_z = MLP.intercepts_[0]
    bias_u = MLP.intercepts_[1]

    print("float biasHiddenA[NUMBER_OF_HIDDEN_A] = ", end="")
    print('{', end="")
    for i in bias_z:
        print('%2.6f'%(i), end=", ")
        #print(i, end=", ")

    print('};')
    print("float biasOutputA[NUMBER_OF_OUTPUTS_A] = ", end="")
    print('{', end="")
    for i in bias_u:
        print('%2.6f'%(i), end=", ")
        #print(i, end=", ")

    print('};')



def LoadDataFromCsv():
    vec = ['cima-k.csv', 'roda-j.csv', 'roda-h.csv', 'puxa-x.csv']
    ret = np.loadtxt("stoped.csv", delimiter=",")
    for i in vec:
        aux = np.loadtxt(i, delimiter=",")
        ret = np.append(ret, aux, axis=0)

    x = np.array(())
    for i in range(5):
        for _ in range(25):
            x = np.append(x, i)
    return ret, x


def ScaleData(arrayToScale):
    scaller = pre.MinMaxScaler(feature_range=(-1, 1), copy=True)
    ret = scaller.fit_transform(arrayToScale)
    return ret, scaller.data_max_, scaller.data_min_

def TrainNeuralNet(data, classes, numberOfHidden, activationFunction, solver, verbose=False):
    xTrain, xTest, yTrain, yTest = train_test_split(data, classes, test_size=0.2)
    if(verbose):
        x = 10
    else:
        x = False

    xData, yData = LoadDataFromCsv()
    scalledData, dataMin, dataMax = ScaleData(xData)
    MLP = MLPClassifier(hidden_layer_sizes=(numberOfHidden,), activation=activationFunction, solver=solver, max_iter=10000, verbose=x, shuffle=True, n_iter_no_change=1000)
    MLP.fit(xTrain, yTrain)

    yPredict = MLP.predict(xTest)
    if(verbose):
        print("Estatisticas: ")
        print("Acuracy Score")
        print(accuracy_score(yTest, yPredict))
        print("Report")
        print(classification_report(yTest, yPredict))

    return MLP

print("Iniciando")
xData, yData = LoadDataFromCsv()
scalledData, dataMin, dataMax = ScaleData(xData)

print("DataMax")
print("[", end="")
for i in dataMax:
    print("%d" % (i), end=",")
print("]")

print("DataMin")
print("[", end="")
for i in dataMin:
    print("%d" % (i), end=",")
print("]")


MLP = TrainNeuralNet(scalledData, yData, 40, 'tanh', 'adam', True)
GenerateCodeToC(MLP)