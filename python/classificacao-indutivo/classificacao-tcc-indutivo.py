import numpy as np
import sklearn.preprocessing as pre
from sklearn.neural_network import MLPClassifier
from sklearn.model_selection import train_test_split
from sklearn.metrics import accuracy_score
from sklearn.metrics import confusion_matrix
from sklearn.metrics import classification_report
def GenerateCodeToCProgram(MLP):
    coefs_z = np.array(MLP.coefs_[0])
    coefs_u = np.array(MLP.coefs_[1])
    numberOfInputs = len(coefs_z)
    numberOfHidden = len(coefs_z[0])
    numberofOutputs = len(coefs_u[0])

    print("#define NUMBER_OF_INPUTS               ", end="")
    print(numberOfInputs)
    print("#define NUMBER_OF_HIDDEN               ", end="")
    print(numberOfHidden)
    print("#define NUMBER_OF_OUTPUTS              ", end="")
    print(numberofOutputs)
    print("")

    print("float weightInputHidden[NUMBER_OF_INPUTS][NUMBER_OF_HIDDEN] = ")
    print("{")
    for i in coefs_z:
        print("{", end="")
        for ii in i:
           print("%2.6f" % (ii), end=", ")
           #print(ii, end=", ")
        print("},")
    print("};")

    print("float weightHiddenOutput[NUMBER_OF_HIDDEN][NUMBER_OF_OUTPUTS] =")
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

    print("float biasHidden[NUMBER_OF_HIDDEN] = ", end="")
    print('{', end="")
    for i in bias_z:
        print('%2.6f'%(i), end=", ")
        #print(i, end=", ")

    print('};')
    print("float biasOutput[NUMBER_OF_OUTPUTS] = ", end="")
    print('{', end="")
    for i in bias_u:
        print('%2.6f'%(i), end=", ")
        #print(i, end=", ")

    print('};')

vec = ['B.csv', 'C.csv', 'D.csv', 'E.csv', 'F.csv', 'G.csv', 
                'H-K.csv', 'I-J.csv', 'L.csv','M.csv', 'N.csv', 
                'O.csv', 'Q.csv', 'R.csv', 'S.csv', 'T.csv',
                'U.csv', 'V.csv', 'W.csv', 'Y.csv', 'X-Z.csv']


#A, B, C, D, E, F, G, H-K-P, I-J, L, M, N, O, Q, R, S, T, U, V, W, Y, X-Z
def arruma_dados():
    x = np.loadtxt("A.csv", delimiter=",")
    ret = x[:, :12]
    for i in vec:
        aux = np.loadtxt(i, delimiter=",")
        ret = np.append(ret, aux[:, :12], axis=0)
    return ret

def normalize(vetor):  
    minValue = [0,0,0,0,0,0,0,0,0,0,0,126]
    maxValue = [1480, 2188, 2415, 2809, 2047, 2158, 3519, 3442, 34, 2322, 3725, 3748]
    for i in range(12):
        vetor[i] =  2 * ( (vetor[i] - minValue[i]) / (maxValue[i] - minValue[i]) ) - 1  

def testeClassificacao(clf, InputValues, verbose=0, tanhActivation=True):
    weightHiddenOut = clf.coefs_[1]
    weightInHidden = clf.coefs_[0]

    biasHidden = clf.intercepts_[0]
    biasOutput = clf.intercepts_[1]

    numberOfInputs = len(weightInHidden)
    numberOfHidden = len(weightInHidden[0])
    numberofOutputs = len(weightHiddenOut[0])

    if(verbose):
        print("Size wHiOut: " + str(len(weightHiddenOut)) + "x" +str(len(weightHiddenOut[0])))
        print("Size wInHi: " + str(len(weightInHidden)) + "x" +str(len(weightInHidden[0])))
        print("Bias Output: " + str(len(biasOutput)))
        print("Bias Hidden: " + str(len(biasHidden)))

    Hidden = np.zeros((numberOfHidden))
    Out = np.zeros((numberofOutputs))
    normalize(InputValues)

    for i in range(numberOfHidden):
        for ii in range(numberOfInputs):
            Hidden[i] = (InputValues[ii] * weightInHidden[ii][i]) + Hidden[i]
                
        Hidden[i] = np.tanh(Hidden[i] + biasHidden[i])
       
        if(verbose):
            print('Oculto ' + str(i) + ': ' + str(Hidden[i]))


    for i in range(numberofOutputs):
        for ii in range(numberOfHidden):
            Out[i] = (Hidden[ii] * weightHiddenOut[ii][i]) + Out[i]
        
        Out[i] = Out[i] + biasOutput[i]
        
        if(verbose):
            print("Saida "+ str(i) + ": " + str(Out[i]))

    return Hidden, Out

def treinamento(activation, solver, hidden_layers, verbose=False):
    caracteristicas = arruma_dados()
    scaler = pre.MinMaxScaler(feature_range=(-1,1))
    scaler.fit(caracteristicas)
    caracteristicas = scaler.transform(caracteristicas)
    classes = np.array(())
    for i in range(len(vec) + 1):
        for _ in range(50):
            classes = np.append(classes, i)

    x_train, x_test, y_train, y_test = train_test_split(caracteristicas, classes, test_size=0.25)
    if(verbose):
        x = 10
    else:
        x = False
    clf = MLPClassifier(hidden_layer_sizes=(hidden_layers,), activation=activation, solver=solver, max_iter=1500, verbose=x,shuffle=True, n_iter_no_change=100)

    clf.fit(x_train, y_train)
    y_pred = clf.predict(x_test)


    if(verbose):
        print(accuracy_score(y_test, y_pred))
        print(confusion_matrix(y_test, y_pred))
        print(classification_report(y_test, y_pred))
        print('Score')
        print(clf.score(x_test, y_test))

    return clf

MLP = treinamento('tanh', 'adam', 21, verbose=20)

#GenerateCodeToCProgram(MLP)

import matplotlib.pyplot as plt

def plot_confusion_matrix(y_true, y_pred, classes,
                          normalize=False,
                          title=None,
                          cmap=plt.cm.Blues):
    """
    This function prints and plots the confusion matrix.
    Normalization can be applied by setting `normalize=True`.
    """
    if not title:
        if normalize:
            title = 'Normalized confusion matrix'
        else:
            title = 'Confusion matrix, without normalization'

    # Compute confusion matrix
    cm = confusion_matrix(y_true, y_pred)
    # Only use the labels that appear in the data
    classes = classes[unique_labels(y_true, y_pred)]
    if normalize:
        cm = cm.astype('float') / cm.sum(axis=1)[:, np.newaxis]
        print("Normalized confusion matrix")
    else:
        print('Confusion matrix, without normalization')

    print(cm)

    fig, ax = plt.subplots()
    im = ax.imshow(cm, interpolation='nearest', cmap=cmap)
    ax.figure.colorbar(im, ax=ax)
    # We want to show all ticks...
    ax.set(xticks=np.arange(cm.shape[1]),
           yticks=np.arange(cm.shape[0]),
           # ... and label them with the respective list entries
           xticklabels=classes, yticklabels=classes,
           title=title,
           ylabel='True label',
           xlabel='Predicted label')

    # Rotate the tick labels and set their alignment.
    plt.setp(ax.get_xticklabels(), rotation=45, ha="right",
             rotation_mode="anchor")

    # Loop over data dimensions and create text annotations.
    fmt = '.2f' if normalize else 'd'
    thresh = cm.max() / 2.
    for i in range(cm.shape[0]):
        for j in range(cm.shape[1]):
            ax.text(j, i, format(cm[i, j], fmt),
                    ha="center", va="center",
                    color="white" if cm[i, j] > thresh else "black")
    fig.tight_layout()
    return ax


#np.set_printoptions(precision=2)

# Plot non-normalized confusion matrix
#plot_confusion_matrix(y_test, y_pred, classes=class_names,title='Confusion matrix, without normalization')                      

#plt.show()