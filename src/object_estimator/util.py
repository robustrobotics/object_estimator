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
import sys
import tty
import numpy as np
import cPickle as pk
import time
import math

## read a pickle and return the object.
# @param filename - name of the pkl
# @return - object that had been pickled.
def load_pickle(filename):
    try:
        p = open(filename, 'r')
    except IOError:
        print "hrl_lib.util: Pickle file cannot be opened."
        return None
    try:
        picklelicious = pk.load(p)
    except ValueError:
        print 'util.load_pickle failed once, trying again'
        p.close()
        p = open(filename, 'r')
        picklelicious = pk.load(p)

    p.close()
    return picklelicious

## Pickle an object.
# @param object - object to be pickled
# @param filename - name of the pkl file
def save_pickle(object, filename):
    pickle_file = open(filename, 'w')
    pk.dump(object, pickle_file)
    pickle_file.close()
