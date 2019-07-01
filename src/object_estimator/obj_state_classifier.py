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
import math
import numpy as np
from copy import copy, deepcopy
import threading
import collections

from sklearn.metrics import accuracy_score
from sklearn.model_selection import train_test_split
from sklearn.model_selection import cross_val_score
from random import shuffle

# ROS
import rospy, rosparam, rospkg, tf
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Point, Quaternion, PoseArray, Pose #PointStamped,

# Private
import util as ut
from object_estimator.hmm.learning_base import learning_base
from object_estimator.hmm import learning_hmm as hmm
from object_estimator.srv import None_String, None_StringResponse
from object_estimator.msg import EndpointState
from preprocessing import *

QUEUE_SIZE = 10


# NOTE
# light -> empty
# heavy -> full

class obj_state_classifier(learning_base):
    """Object state classifier class

    This class provides an object for binary state classification
    using hidden Markov models.
    """
    def __init__(self, input_labels=None, output_labels=None, state_topic=None, srv_topic=None,
                 debug=False, evaluation=False):
        """The constructor"""
        # parent class that provides sklearn related interfaces.
        learning_base.__init__(self)

        if input_labels is not None and output_labels is not None:
            self.key_to_label = {}
            for i, l in enumerate(input_labels):
                self.key_to_label[l] = i
            self.output_labels = output_labels
        else:
            self.key_to_label = {'full': 0, 'empty':1}
        
        self._state_topic  = state_topic
        self._srv_topic    = srv_topic
        self._debug_mode   = debug
        self._event_lock   = threading.RLock() ## joint state lock

        self._endpoint_force = 0
        self._window_len = min_length
        self._window     = collections.deque(maxlen=self._window_len)
        self._filt       = collections.deque(maxlen=filt_len)
        
        self._initParams()        
        if evaluation is False: self._initComms()
        

    def _initParams(self):
        """Load parameters from ROS server"""       
        # HMM parameters
        self.nState       = 20
        self.nEmissionDim = 2
        self.cov          = 2. #0.5
        self.scale        = [10,1] #[0.5,1.]
        self.cov_type     = 'full'
        self.cov_mult     = [self.cov]*(self.nEmissionDim**2)

        self._likelihood_ratio_ths = 0 #2.11 #0.75


    def _initComms(self):
        """Initialize ROS publishers/subscribers"""        
        rospy.Subscriber(self._state_topic, EndpointState, self._state_callback)
        rospy.Service(self._srv_topic, None_String, self._state_estimation_srv)
        

    def _state_callback(self, msg):
        '''Subscribe state'''        
        with self._event_lock:
            self._filt.append( [msg.pose.position.x, msg.wrench.force.x] )

            if len(self._filt)<filt_len: return

            sample = np.sum(self._filt, axis=0)/float(filt_len)
            self._window.append( sample )


    def _state_estimation_srv(self, req):
        '''Service to return the state of object'''
        with self._event_lock:
            x = np.array(deepcopy(self._window))

            if len(x)<5: return None_StringResponse("NA")
            
            # offset
            #x[:,0] -= np.mean(x[:10,0])
            x -= np.mean(x[:10], axis=0)
            for i, v in enumerate(x):
                #if v[0] > z_thres:
                if abs(v[1]) > force_thres:
                    break
            #i=len(x)-1

            idx = i-time_offset if i-time_offset >=0 else 0
            if len(x)<idx+data_length:
                x = list(x)
                while len(x) < idx+data_length:
                    x.append(x[-1])
            
            x = np.array(x)[idx:idx+data_length]
            x -= np.mean(x[:10], axis=0)
                        
            # pre-processing
            ## x = (x-np.amin(x,axis=0))/(np.amax(x,axis=0)-np.amin(x,axis=0))
            ## x -= np.mean(x[:10], axis=0)

            ###############################################3
            if self._debug_mode is True: self.debug_plot(x)
            ###############################################3
            
            x = np.expand_dims(x, axis=0)
            y_pred = self.predict(x)[0]

            if y_pred == 0: return None_StringResponse(self.output_labels[0])
            else: return None_StringResponse(self.output_labels[1])        


    def reset(self):
        '''Reset the trained model'''
        for i, key in enumerate(self.key_to_label.keys()):
            self._mls[self.key_to_label[key]].reset()        


    def fit(self, X, y):

        trainData = []
        self._mls = []
        for key in self.key_to_label.keys():
            trainData.append([])
            self._mls.append(None)
                
        for i, label in enumerate(y):
            trainData[label].append(X[i])

        for i, key in enumerate(self.key_to_label.keys()):
            ml = hmm.learning_hmm(self.nState, self.nEmissionDim, verbose=False)
            data = trainData[self.key_to_label[key]]
            data = np.swapaxes(data,0,2)
            data = np.swapaxes(data,1,2)#*self.scale
            for j in xrange(len(data)):
                data[j] *= self.scale[j]

            ret = ml.fit(data, cov_mult=self.cov_mult, \
                         use_pkl=False, cov_type=self.cov_type)
            if ret == 'Failure' or np.isnan(ret):
                print "hmm training failed"
                sys.exit()
            
            self._mls[self.key_to_label[key]] = ml 

                   
    def predict(self, X):

        y_pred = []
        for i, x in enumerate(X):
            if len(np.shape(x)) == 2:
                x = np.expand_dims(x, axis=0)
            testData = np.swapaxes(x,0,2)
            testData = np.swapaxes(testData,1,2)#*self.scale

            for j in xrange(len(testData)):
                testData[j] *= self.scale[j]

            
            l = []
            for j in xrange(len(self._mls)):
                logp = self._mls[j].loglikelihood(testData)
                l.append(logp)

            # TODO for multi classes
            # Likelihood ratio based binary classification
            if l[1]-l[0] > self._likelihood_ratio_ths:
                print "Label 1, Log-likelihood Ratio: {}, Threshold: {}".format(l[1]-l[0],
                                                                                self._likelihood_ratio_ths)
                y_pred.append( 1 )
            else:
                print "Label 0, Log-likelihood Ratio: {}, Threshold: {}".format(l[1]-l[0],
                                                                                self._likelihood_ratio_ths)
                y_pred.append( 0 )

        return y_pred


    def score(self, X, y_true):
        '''sklearn type score function'''
        y_pred = self.predict(X)
        return accuracy_score(y_true, y_pred)
    

    def run(self, data_dict, train_ratio=0.8):
        '''Main loop'''
        data   = []
        target = []
        for key in data_dict.keys():

            last_idx = int(float(len(data_dict[key]))*train_ratio)
            
            for i, x in enumerate(data_dict[key]):
                if i >= last_idx: break
                data.append(x)
                target.append(self.key_to_label[key])

        self.fit(data, target)

        rospy.loginfo("Start to run object state classifier")
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()


    def debug_plot(self, x):
        rospack   = rospkg.RosPack()
        data_path = os.path.join(rospack.get_path('object_estimator'),'data')
        d = ut.load_pickle(os.path.join(data_path, "empty_full_data.pkl"))
                
        import matplotlib.pyplot as plt
        fig = plt.figure(1)
        ax = fig.add_subplot(2, 1, 1)
        plt.plot(x[:,0], 'k*-', markersize=12)
        plt.plot(d[self.output_labels[0]][:,:,0].T, 'r-', markersize=12)
        plt.plot(d[self.output_labels[1]][:,:,0].T, 'b-', markersize=12)

        ax = fig.add_subplot(2, 1, 2)
        plt.plot(x[:,1], 'k*-', markersize=12)
        plt.plot(d[self.output_labels[0]][:,:,1].T, 'r-', markersize=12)
        plt.plot(d[self.output_labels[1]][:,:,1].T, 'b-', markersize=12)
        #from IPython import embed; embed(); sys.exit()

        plt.ion()
        plt.show()
        ## plt.savefig("test.pdf")
        #sys.exit()


def cross_eval(data_dict, labels, cv=5):
    '''sklearn type cross-evlation function'''

    key_to_label = {}
    for i, l in enumerate(labels):
        key_to_label[l] = i
    
    data   = []
    target = []
    for key in data_dict.keys():
        for i, x in enumerate(data_dict[key]):        
            data.append(x)
            target.append(key_to_label[key])
    
    rnd_idx = range(len(target))
    shuffle(rnd_idx)
    data   = np.array(data)[rnd_idx].tolist()
    target = np.array(target)[rnd_idx].tolist()

    clf = obj_state_classifier(evaluation=True)
    #clf.fit(data, target)
    scores = cross_val_score(clf, data, target, cv=cv)

    print "-----------------------------------------------------"
    print "Avg. score: ", np.mean(scores), np.std(scores)
    print "-----------------------------------------------------"


    


if __name__ == '__main__':
    import optparse
    p = optparse.OptionParser()
    p.add_option('--state_topic', action='store', dest='state_topic', type='string',
                 default='state', help='Input a ROS state topic name')    
    p.add_option('--srv_topic', action='store', dest='srv_topic', type='string',
                 default='object_state/left', help='Input a ROS service name')    
    p.add_option('--data_path', '--dp', action='store', dest='data_path', type='string',
                 default='data', help='Input data path')    
    p.add_option('--preprocessing', action='store', dest='preprocessing', type='string',
                 default='false', help='Preprocess raw data')    
    p.add_option('--saved_filename', action='store', dest='saved_filename', type='string',
                 default='empty_full_data.pkl', help='Input saved pickle file name')
    
    p.add_option('--input_labels', action='store', dest='in_labels', type='string',
                 default='light_heavy', help='Input a list of labels (ex, light_heavy)')    
    p.add_option('--output_labels', action='store', dest='out_labels', type='string',
                 default=None, help='Input a list of labels (default, same as in_labels)')    
    
    p.add_option('--plot', action='store', dest='plot', type='string',
                 default='false', help='Plot raw data')
    p.add_option('--cv', action='store', dest='cv', type='string',
                 default='false', help='cross evaluation')
    p.add_option('--debug', action='store', dest='debug', type='string',
                 default='false', help='Debug mode')
    opt, args = p.parse_args()


    rospack   = rospkg.RosPack()
    data_path = os.path.join(rospack.get_path('object_estimator'), opt.data_path)

    # Input labels
    input_labels = []
    for l in opt.in_labels.split('_'):
        input_labels.append(l)

    # Output labels
    if opt.out_labels is None:
        output_labels = opt.in_labels
    else:
        output_labels = []
        for l in opt.out_labels.split('_'):
            output_labels.append(l)

    if opt.debug == 'true': opt.debug = True
    else: opt.debug = False
            
    # Data
    if opt.preprocessing.find('true')>=0:
        d = extract_data(data_path, input_labels, opt.state_topic)
        ut.save_pickle(d, os.path.join(data_path, opt.saved_filename))
    else:
        d = ut.load_pickle(os.path.join(data_path, opt.saved_filename))
        assert d is not None, "Data dict is empty. Did you copy or collected data?"

    if opt.plot.find('true')>=0:# or opt.debug.find('true')>=0:
        plot_raw_data(d, input_labels)
    elif opt.cv.find('true')>=0:
        cross_eval(d, input_labels)    
    else:
        rospy.init_node('obj_state_classifier_node')
        wse = obj_state_classifier(input_labels, output_labels, opt.state_topic, opt.srv_topic,
                                   debug=opt.debug)
        wse.run(d)


        
