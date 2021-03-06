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
import os
import numpy as np
import collections

import rosbag
import matplotlib.lines as mlines

min_length   = 700 
data_length  = 150 #300
force_thres  = 0.2
filt_len     = 10
time_offset = 50
z_thres = 0.05

def extract_data(data_path, labels, state_topic):
    """Extract and preprocess data
    z_thres:     a lifting-time decision threshold [m]
    data_length: a length of data sequence (50~200 will be a good number)
    """
    key_to_label = {}
    for i, l in enumerate(labels):
        key_to_label[l] = i

    # set contrainers
    file_dict = {}
    data_dict = {}
    time_dict = {}
    for key in key_to_label:
        file_dict[key] = []
        data_dict[key] = []
        time_dict[key] = []

    # get all bags
    folder_list  = [d for d in os.listdir(data_path) if os.path.isdir(os.path.join(data_path,d))]
    file_list = []
    for d in folder_list:
        for key in key_to_label:
            if d.find(key)>=0:        
                sub_dir = os.path.join(data_path,d)
                for f in os.listdir(sub_dir):
                    if f.find('.bag')>=0: file_dict[key].append(os.path.join(sub_dir,f))
                break
                                        
    # extract data    
    for key in file_dict.keys():
        for f in file_dict[key]:
            # load bag file
            bag = rosbag.Bag(f, "r")

            max_len = filt_len
            filt = collections.deque(maxlen=max_len)

            # load topic msgs at each time step
            samples = []
            times   = []
            for topic, msg, t in bag.read_messages(topics=[state_topic]):
                filt.append([msg.pose.position.z, msg.wrench.force.z])
                if len(filt)<max_len: continue

                samples.append(np.sum(filt, axis=0)/float(max_len))
                times.append(msg.header.stamp.secs)

            # make the time start from 0
            data_dict[key].append(samples)
            time_dict[key].append(list(np.array(times)-times[0]))
            
    # select a length of sequences from the extracted data
    for key in data_dict.keys():
        rm_element_idx = []
        for i in xrange(len(data_dict[key])):
            # Remove white noise (offset)
            samples = np.array(data_dict[key][i])
            samples[:,0] = np.array(samples[:,0]) - np.mean(samples[:10,0])

            # Find a lifting timestep that z-height is over a certain threshold(z_thres)
            for j in xrange(len(samples)):
                if samples[j][0]>z_thres: break

            # Set start index
            data_dict[key][i] = samples
            idx = j-time_offset if j-time_offset >=0 else 0

            # Handle an exception that data is short. We repeat the last element as a solution.
            if len(data_dict[key][i])<idx+data_length:
                data_dict[key][i] = list(data_dict[key][i])
                while len(data_dict[key][i]) < idx+data_length:
                    data_dict[key][i].append(data_dict[key][i][-1])
                    time_dict[key][i].append(time_dict[key][i][-1]+
                                             (time_dict[key][i][-1]-time_dict[key][i][-2]))
            
            # Extract data from the start index to index+data_length
            data_dict[key][i] = data_dict[key][i][idx:idx+data_length]
            time_dict[key][i] = time_dict[key][i][idx:idx+data_length]

            samples = data_dict[key][i]
            data_dict[key][i] = samples

            # Remove data that still has a short length
            if len(data_dict[key][i]) != data_length: rm_element_idx.append(i)
        
        data_dict[key] = np.array([data_dict[key][i] for i in range(len(data_dict[key]))
                                   if i not in rm_element_idx ])
        time_dict[key] = np.array([time_dict[key][i] for i in range(len(time_dict[key]))
                                   if i not in rm_element_idx ])

    return data_dict


def plot_raw_data(data_dict):
    '''Plot raw data'''
    
    import matplotlib.pyplot as plt
    #plt.style.use('ggplot')
    fig = plt.figure(figsize=(12, 6))
    #fig = plt.figure(1)

    dim = len(data_dict['heavy'][0,0])
    for i in xrange(dim):

        ax = fig.add_subplot(1, dim, i+1)

        mu_light  = np.mean(data_dict['light'][:,:,i],axis=0)
        std_light = np.std(data_dict['light'][:,:,i],axis=0)
        mu_heavy = np.mean(data_dict['heavy'][:,:,i],axis=0)
        std_heavy = np.std(data_dict['heavy'][:,:,i],axis=0)
        
        plt.plot(mu_light, 'b-', label='Empty')
        plt.plot(mu_heavy, 'r-', label='Full')

        ax.fill_between(range(len(mu_light)),
                        mu_light+std_light,
                        mu_light-std_light,
                        facecolor='blue', alpha=0.5, linewidth=0)
        ax.fill_between(range(len(mu_heavy)),
                        mu_heavy+std_heavy,
                        mu_heavy-std_heavy,
                        facecolor='red', alpha=0.5, linewidth=0)

        if i==0:
            ax.set_ylabel("Lifting distance [m]", fontsize=18)
            #ax.xaxis.set_major_locator(plt.NullLocator())
        else:
            ax.set_ylabel("Z-axis force [N]", fontsize=18)

        ax.set_xlim([0, len(mu_light)])
        ax.set_xlabel("Time [s]", fontsize=18)
        
        plt.xticks([0, len(mu_light)/2, len(mu_light)], [0, 0.675, 1.35])

    blue_line = mlines.Line2D([], [], color='blue', alpha=0.5, markersize=30, label='Empty', lw=10)
    green_line = mlines.Line2D([], [], color='red', markersize=15, label='Full', lw=10)
    handles = [blue_line,green_line]
    labels = [h.get_label() for h in handles]
    fig.legend(handles=handles, labels=labels, loc='upper right', ncol=2, shadow=True, fancybox=True,
               prop={'size': 16})
    
    plt.tight_layout()
    ## plt.ion()
    plt.savefig("test.pdf")
    plt.show()
