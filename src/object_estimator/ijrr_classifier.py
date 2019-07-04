#!/usr/bin/env python
"""
MIT License

Copyright (c) 2018 Daehyung Park

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
"""

# SYSTEM
import os, sys
import numpy as np
from copy import copy, deepcopy
import rospy, rospkg
from random import shuffle

# Private
import util as ut
import obj_state_classifier as osc
from preprocessing import *

from sklearn import metrics
from sklearn.model_selection import cross_val_score
np.random.seed(0)

class evaluation():

    def __init__(self, train_data_path='data', test_data_path='data',
                 saved_filename='empty_full_data.pkl',
                 preprocessing=False, 
                 input_labels=None):

        # Training Data
        data_path = os.path.join(rospack.get_path('object_estimator'), train_data_path)
        train_filename = os.path.join(data_path, "train_"+saved_filename)
        if preprocessing.find('true')>=0 or os.path.isfile(train_filename) is False:
            train_dict = extract_data(data_path, input_labels, 'husky8/EndpointState')
            ut.save_pickle(train_dict, train_filename)
        else:
            train_dict = ut.load_pickle(train_filename)
            assert train_dict is not None, "Data dict is empty. Did you copy or collected data?"

        # Test Data
        data_path = os.path.join(rospack.get_path('object_estimator'), test_data_path)
        test_filename = os.path.join(data_path, "test_"+saved_filename)
        if preprocessing.find('true')>=0  or os.path.isfile(test_filename) is False :
            test_dict = extract_rosbag_data(test_data_path, input_labels, 'husky8/EndpointState',
                                            force_thres=5)
            ut.save_pickle(test_dict, os.path.join(test_data_path, test_filename))
        else:
            test_dict = ut.load_pickle(os.path.join(test_data_path, test_filename))
            assert test_dict is not None, "Data dict is empty. Did you copy or collected data?"


        self.train_dict = train_dict
        self.test_dict = test_dict
        train_ratio=0.8

        if input_labels is not None:
            self.input_labels = input_labels
            self.key_to_label = {}
            for i, l in enumerate(input_labels):
                self.key_to_label[l] = i
        else:
            print "no input label"
            sys.exit()
            self.key_to_label = {'empty': 0, 'full':1}
            self.input_labels = ['empty', 'full']
        

        data   = []
        target = []
        for key in train_dict.keys():
            last_idx = int(float(len(train_dict[key]))*train_ratio)
            
            for i, x in enumerate(train_dict[key]):
                if i >= last_idx: break
                data.append(x)
                target.append(self.key_to_label[key])

        rnd_idx = range(len(target))
        shuffle(rnd_idx)
        data   = np.array(data)[rnd_idx].tolist()
        target = np.array(target)[rnd_idx].tolist()

        # train classifier 
        self.clf = osc.obj_state_classifier(evaluation=True)
        self.nState = 20
        self.clf.cov = 1.
        self.clf.scale = [7.5, 10.]
        self.clf._likelihood_ratio_ths=5
        if False:
            cv=10
            scores = cross_val_score(self.clf, data, target, cv=cv)
            print "-----------------------------------------------------"
            print "Avg. score: ", np.mean(scores), np.std(scores)
            print "-----------------------------------------------------"
            sys.exit()
        
        self.clf.fit(data, target)
        

    def get_test_data_info(self):
        """
        Print out a list of available object name and the number of data
        """
        print " --------- Object info ------------"
        for key in self.test_dict.keys():
            for label in self.key_to_label:
                if len(self.test_dict[key][label])>0:
                    print "name={}: label={}, #data={}".format(key, label, len(self.test_dict[key][label]))
                    break
        print "-----------------------------------------------------"
        return 

    def eval_all(self):
        """
        Evaluate all rosbag files
        """

        y_true = []
        y_pred = []
        l_ratio = [[],[]]
        
        for obj_name in self.test_dict.keys():
            data   = []
            target = []

            for label in self.key_to_label.keys():            
                if len(self.test_dict[obj_name][label])>0:
                    break

            ## self.test_dict[obj_name][label] = self.train_dict[label]                
            for i, x in enumerate(self.test_dict[obj_name][label]):        
                data.append(x)
                target.append(self.key_to_label[label])

            y_true += target
            ## y_pred += self.clf.predict(data)
            ## print "Target: " , target
            temp1, temp2 = self.clf.predict(data, debug=True, verbose=False)
            y_pred += temp1
            #y_pred += list(np.array(temp1)*-1+1)
            l_ratio[self.key_to_label[label]] += temp2

        ## print np.mean(l_ratio, axis=-1)
        ## print np.std(l_ratio, axis=-1)

        # viz confusion matrix
        ## metrics.confusion_matrix(y_true, y_pred, self.input_labels)
        print "-----------------------------------------------------"
        print "Confusion Matrix: "
        print metrics.confusion_matrix(y_true, y_pred, labels=[0,1])
        print "ACC: {}".format(metrics.accuracy_score(y_true, y_pred))
        print "-----------------------------------------------------"
        return 


    def eval_one(self, object_name, file_idx):
        """
        Evaluate single rosbag files
        """

        for true_label in self.key_to_label.keys():            
            if len(self.test_dict[object_name][true_label])>0:
                break
        assert file_idx<len(self.test_dict[object_name][true_label]), "Wrong file index"

        data = [self.test_dict[object_name][true_label][file_idx:file_idx+1]]
        target = [self.key_to_label[true_label]]

        # classification
        y_pred = self.clf.predict(data)

        for pred_label in self.key_to_label.keys():
            if self.key_to_label[pred_label] == y_pred[0]:
                break

        print "-----------------------------------------------------"
        print "Classification result of {} {}".format(object_name, file_idx)
        print "sucess={}, true_label={}, pred_label={}".format(y_pred[0]==target[0], true_label, pred_label)
        print "-----------------------------------------------------"
        return y_pred[0]==target[0], true_label, pred_label 


    def data_plot(self):

        import matplotlib.pyplot as plt
        fig = plt.figure(1)
        ax1 = fig.add_subplot(2, 1, 1)
        ax2 = fig.add_subplot(2, 1, 2)
            
        
        for obj_name in self.test_dict.keys():
            data   = []
            target = []

            for label in self.key_to_label.keys():            
                if len(self.test_dict[obj_name][label])>0:
                    break
            
            for i, x in enumerate(self.test_dict[obj_name][label]):        
                data.append(x)
                target.append(self.key_to_label[label])

            if self.key_to_label[label]==0: c = 'r-'
            else: c = 'b-'
            ax1.plot(np.array(data)[:,:,0].T, c, markersize=12)
            ax2.plot(np.array(data)[:,:,1].T, c, markersize=12)

        ## plt.ion()
        plt.show()


if __name__ == '__main__':
    import optparse
    p = optparse.OptionParser()
    p.add_option('--preprocessing', action='store', dest='preprocessing', type='string',
                 default='false', help='Preprocess raw data')    
    p.add_option('--train_data_path', action='store', dest='train_data_path', type='string',
                 default='data', help='Input data path')    
    p.add_option('--test_data_path', action='store', dest='test_data_path', type='string',
                 default='data', help='Input data path')    
    p.add_option('--saved_filename', action='store', dest='saved_filename', type='string',
                 default='empty_full_data.pkl', help='Input saved training data pickle file name')

    p.add_option('--input_labels', action='store', dest='in_labels', type='string',
                 default='empty_full', help='Input a list of labels (ex, light_heavy)')

    p.add_option('--evaluate_all', action='store_true', dest='eval_all',
                 help='evaluate all dataset')
    p.add_option('--evaluate_one', action='store_true', dest='eval_one',
                 help='evaluate all dataset')

    p.add_option('--object_name', action='store', dest='obj_name', type='string',
                 default='cabinet', help='Input an object name')
    p.add_option('--file_id', action='store', dest='file_id', type=int,
                 default=20, help='Input an id number out of 30')
    
    opt, args = p.parse_args()


    rospack   = rospkg.RosPack()

    # Input labels
    input_labels = []
    for l in opt.in_labels.split('_'):
        input_labels.append(l)


    ev = evaluation(opt.train_data_path, opt.test_data_path, opt.saved_filename,
                    opt.preprocessing,
                    input_labels=input_labels)
    ev.get_test_data_info()
    ## ev.data_plot()

    if opt.eval_all:
        ev.eval_all()
    elif opt.eval_one:
        ev.eval_one(opt.obj_name, opt.file_id)


