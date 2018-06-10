"""Implementing a decision tree model for classification of trucks, which is
observed to be performing better than the previously implemented Naive Bayes
Classifier

References :

1. https://en.wikipedia.org/wiki/Decision_tree
2. http://scikit-learn.org/stable/modules/tree.html

"""
import random
import numpy as np
from sklearn import tree
from sklearn.metrics import confusion_matrix, classification_report
from sklearn.model_selection import train_test_split
from sklearn.tree import _tree


# we consider the number of features i.e. fft values per sample to be
# considered for training and classification
num_freq = 1024

# extracting the truck fft values
f_truck = open("truck_after_filter_fft.txt", "r")
truck_features = []
for line in f_truck.readlines():
    if len(line) > 1:
        truck_features.append([[int(x) for x in line.split()[:num_freq]], 1])

# extracting the non truck fft values
f_no_truck = open("no_truck_after_filter_fft.txt", "r")
for line in f_no_truck.readlines():
    if len(line) > 1:
        truck_features.append([[int(x) for x in line.split()[:num_freq]], 2])

# randomly shuffle the samples for training to avoid high correlation between
# training values
random.shuffle(truck_features)

# split the features and labels after shuffling
data_labels = []
data_features = []

for i in range(len(truck_features)):
    if len(truck_features[i][0]) == num_freq:
        data_features.append(truck_features[i][0])
        data_labels.append(truck_features[i][1])  #

data_features = np.asarray([np.asarray(x) for x in data_features])
data_labels = np.asarray(data_labels)

np_mat = np.zeros((0, num_freq))

for r in data_features:
    na = np.asarray(r)
    np_mat = np.vstack([np_mat, na])

print 'NUMPY MATRIX SHAPE: ', np_mat.shape


# splitting the training data and testing data from the input feature set
tr_features, ts_features, tr_labels, ts_labels = \
    train_test_split(data_features, data_labels, test_size = 0.3,
                     train_size = 0.7)

# creating the decision tree classifier with tried and tested depth
# which yields better results
classifier = tree.DecisionTreeClassifier(max_depth = 7)
# fit the data to the model
classifier.fit(tr_features, tr_labels)
# extract the predicted label values for the testing features
y_pred = classifier.predict(ts_features)
# printing the confusion matrix to show the classification of the data
print confusion_matrix(ts_labels, y_pred)

# storing the decision tree in a file, which can be visualized later
tree_model = open("decisiontree.txt", "w")
tree_model = tree.export_graphviz(classifier, out_file = tree_model)
# printing the results of the decision tree classifier
print classification_report(ts_labels, y_pred)


# function to generate decision tree C code from the decision tree model
# produced above
# Reference : https://stackoverflow.com/questions/20224526/how-to-extract-the-decision-rules-from-scikit-learn-decision-tree
def tree_to_code(tree, feature_names):
    tree_ = tree.tree_
    feature_name = [
        feature_names[i] if i != _tree.TREE_UNDEFINED else "undefined!"
        for i in tree_.feature
    ]
    c_code = open("c_code.c", "w")
    
    c_code.write("bool decisionTree(){\n")

    def recurse(node, depth):
        indent = "  " * depth
        if tree_.feature[node] != _tree.TREE_UNDEFINED:
            name = feature_name[node]
            threshold = tree_.threshold[node]
            c_code.write("{}if (SrcBuf[{}] <= {}){{\n".format(indent, name, threshold))
            recurse(tree_.children_left[node], depth + 1)
            c_code.write("\n{}else{{ ".format(indent))
            recurse(tree_.children_right[node], depth + 1)
            c_code.write("\n{}}}".format(indent))
        else:
            if tree_.value[node][0][0] > tree_.value[node][0][1]:
                c_code.write("{}return true;\n{}}}".format(indent, indent))
            else:
                c_code.write("{}return false;\n{}}}".format(indent, indent))

    recurse(0, 1)

# storing the decision tree code to a .c file
tree_to_code(classifier, [str(x) for x in range(num_freq)])
