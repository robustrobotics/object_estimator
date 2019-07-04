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
offset_length= 10 
data_length  = 50 #150 #200 #300
filt_len     = 3 #10
time_offset = 25
z_thres = 0.05
force_thres  = 5
vel_thres    = 0.03


def extract_rosbag_data(data_path, labels, state_topic, force_thres=5):
    """Extract and preprocess data from multi-object rosbags
    """
    global data_length
    global time_offset
    global offset_length
    data_length = 25
    time_offset = 12
    offset_length = 2
    
    obj_dict = {}

    folder_list  = [d for d in os.listdir(data_path) if os.path.isdir(os.path.join(data_path,d))]
    for d in folder_list:

        obj_name  = d.split('_')[0]
        obj_label = d.split('_')[1]
        data_dict = extract_data(data_path,
                                 labels, state_topic,
                                 folder_list=[d],
                                 force_thres=force_thres)
        obj_dict[obj_name] = data_dict
    
    return obj_dict


def extract_data(data_path, labels, state_topic, folder_list=None, force_thres=5):
    """Extract and preprocess data
    z_thres:     a movement-time decision threshold [m]
    force_thres: a contact-time decision threshold [N]
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
    if folder_list is None:
        folder_list  = [d for d in os.listdir(data_path) if os.path.isdir(os.path.join(data_path,d))]
        
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
                filt.append([msg.pose.position.x, msg.wrench.force.x])
                if len(filt)<max_len: continue

                samples.append(np.sum(filt, axis=0)/float(max_len))
                temp = msg.header.stamp
                times.append(temp.to_sec())

            # make the time start from 0
            data_dict[key].append(samples)
            time_dict[key].append(list(np.array(times)-times[0]))
            
    # select a length of sequences from the extracted data
    for key in data_dict.keys():
        rm_element_idx = []
        for i in xrange(len(data_dict[key])):
            # Remove white noise (offset)
            samples = np.array(data_dict[key][i])
            #samples[:,0] = np.array(samples[:,0]) - np.mean(samples[:10,0])
            #samples[:,1] = np.array(samples[:,1]) - np.mean(samples[:10,1])
            samples = np.array(samples) - np.mean(samples[:offset_length], axis=0)
            times   = np.array(time_dict[key][i])

            

            # Find a lifting timestep that z-height is over a certain threshold(z_thres)
            # Find contact_force over a certain threshold
            move_flag = False
            for j in xrange(len(samples)-3):
                #if samples[j][0]>z_thres: break
                if abs(np.mean(samples[j:j+3][1]))>force_thres:
                    break
                ## if abs(samples[j+3][0]-samples[j][0])/(times[j+3]-times[j]) >vel_thres and move_flag is False:
                ##     move_flag = True
                ## if abs(samples[j+3][0]-samples[j][0])/(times[j+3]-times[j]) <vel_thres/3. and move_flag is True:
                ##     break
            
                
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
            samples = np.array(samples) - np.mean(samples[:offset_length], axis=0)
            data_dict[key][i] = samples

            # Remove data that still has a short length
            if len(data_dict[key][i]) != data_length: rm_element_idx.append(i)
        
        data_dict[key] = np.array([data_dict[key][i] for i in range(len(data_dict[key]))
                                   if i not in rm_element_idx ])
        time_dict[key] = np.array([time_dict[key][i] for i in range(len(time_dict[key]))
                                   if i not in rm_element_idx ])

    return data_dict


def plot_raw_data(data_dict, labels):
    '''Plot raw data'''
    
    import matplotlib.pyplot as plt
    #plt.style.use('ggplot')
    fig = plt.figure(figsize=(12, 6))
    #fig = plt.figure(1)

    dim = len(data_dict[labels[0]][0,0])
    for i in xrange(dim):

        ax = fig.add_subplot(1, dim, i+1)

        mu_light  = np.mean(data_dict[labels[0]][:,:,i],axis=0)
        std_light = np.std(data_dict[labels[0]][:,:,i],axis=0)
        mu_heavy = np.mean(data_dict[labels[1]][:,:,i],axis=0)
        std_heavy = np.std(data_dict[labels[1]][:,:,i],axis=0)
        
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
            ax.set_ylabel("Approach distance [m]", fontsize=18)
            #ax.xaxis.set_major_locator(plt.NullLocator())
        else:
            ax.set_ylabel("Z-axis force [N]", fontsize=18)

        ax.set_xlim([0, len(mu_light)])
        ax.set_xlabel("Time [s]", fontsize=18)
        
        #plt.xticks([0, len(mu_light)/2, len(mu_light)], [0, 0.675, 1.35])

    blue_line = mlines.Line2D([], [], color='blue', alpha=0.5, markersize=30, label='Empty', lw=10)
    green_line = mlines.Line2D([], [], color='red', markersize=15, label='Full', lw=10)
    handles = [blue_line,green_line]
    labels = [h.get_label() for h in handles]
    fig.legend(handles=handles, labels=labels, loc='best',
               bbox_to_anchor=(-0.02, -0.25, 0.5, 0.5),
               ncol=2, shadow=True, fancybox=True,
               prop={'size': 16})
    fig.legend(handles=handles, labels=labels, loc='best',
               bbox_to_anchor=(0.4, -0.25, 0.5, 0.5),
               ncol=2, shadow=True, fancybox=True,
               prop={'size': 16})
    
    plt.tight_layout()
    ## plt.ion()
    plt.savefig("test.pdf")
    plt.show()
