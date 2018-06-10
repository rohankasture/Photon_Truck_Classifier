from __future__ import print_function
import scipy.io.wavfile as wavfile
import scipy
import scipy.fftpack
import numpy as np
from matplotlib import pyplot as plt
import os
import math

matrix_len = 705
#probability matrix for truck
prob_ma = [0 for x in range(matrix_len)]
#probability matrix for non-truck
prob_non_truck = [0 for x in range(matrix_len)]


#train with truck samples in current directory
clip_names = []
directory = "."
for filename in os.listdir(directory):
    if filename.endswith(".wav"):
        clip_names.append(filename)

print (clip_names)
clip_number = 0

for clip in clip_names :
    print (clip)
    check = [0 for c in range(matrix_len)]
    fs_rate, signal = wavfile.read(clip)
    print ("Frequency sampling", fs_rate)
    l_audio = len(signal.shape)
    print ("Channels", l_audio)
    if l_audio == 2:
        signal = signal.sum(axis=1) / 2
    N = signal.shape[0]
    print ("Complete Samplings N", N)
    secs = N / float(fs_rate)
    print ("secs", secs)
    Ts = 1.0/fs_rate # sampling interval in time
    print ("Timestep between samples Ts", Ts)
    t = scipy.arange(0, secs, Ts) # time vector as scipy arange field / numpy.ndarray
    #FFT = abs(scipy.fft(signal))
    FFT = scipy.fft(signal)

    f = open("fs100.txt", "w+")
    print (len(FFT))
    for i in FFT:
        f.write(str(i))
        f.write(",")
    f.close()


    abs_FFT = abs(FFT)[:801]
    for index,value in enumerate(abs_FFT) :
        if value >= 0.70*max(abs_FFT[100:]) and index>=100 and index<=800 :
            try :
                if check[(index)-100] != 1 :
                    prob_ma[(index)-100]+=1
                    check[(index)-100]=1
            except IndexError:
                print (clip_number,index)
                break
    clip_number+=1

#train with non_truck samples in non_trucks directory
clip_names = []
directory = "no_trucks"
for filename in os.listdir(directory):
    if filename.endswith(".wav"):
        clip_names.append("no_trucks//"+ filename)

print (clip_names)
clip_number = 0

for clip in clip_names :
    print (clip)
    check = [0 for c in range(matrix_len)]
    fs_rate, signal = wavfile.read(clip)
    print ("Frequency sampling", fs_rate)
    l_audio = len(signal.shape)
    print ("Channels", l_audio)
    if l_audio == 2:
        signal = signal.sum(axis=1) / 2
    N = signal.shape[0]
    print ("Complete Samplings N", N)
    secs = N / float(fs_rate)
    print ("secs", secs)
    Ts = 1.0/fs_rate # sampling interval in time
    print ("Timestep between samples Ts", Ts)
    t = scipy.arange(0, secs, Ts) # time vector as scipy arange field / numpy.ndarray
    #FFT = abs(scipy.fft(signal))
    FFT = scipy.fft(signal)

    f = open("fs100.txt", "w+")
    print (len(FFT))
    for i in FFT:
        f.write(str(i))
        f.write(",")
    f.close()


    abs_FFT = abs(FFT)[:801]
    for index,value in enumerate(abs_FFT) :
        if value >= 0.70*max(abs_FFT[100:]) and index>=100 and index<=800 :
            try :
                if check[(index)-100] != 1 :
                    prob_non_truck[(index)-100]+=1
                    check[(index)-100]=1
            except IndexError:
                print (clip_number,index)
                break
    clip_number+=1

#testing clips
test_clip_names = []
directory = "../test_truck_clips"
# print  test_clip_names
for filename in os.listdir(directory):
    if filename.endswith(".wav"):
        test_clip_names.append(filename)

# print ("Rohan")
print (test_clip_names)
# print ("rohan")

#evaluate resuls
div = 192
div_non_truck = 191
result = []
false_rt = []
# minimum = 9999
for clip in test_clip_names:
    temp = []
    truck_prob = math.log(0.5)
    non_truck_prob = math.log(0.5)
    #run time
    #def calc_fft(fname):
    fs_rate, signal = wavfile.read("../test_truck_clips//" + clip)
    print ("Frequency sampling", fs_rate)
    l_audio = len(signal.shape)
    print ("Channels", l_audio)
    if l_audio == 2:
        signal = signal.sum(axis=1) / 2
    N = signal.shape[0]
    print ("Complete Samplings N", N)
    secs = N / float(fs_rate)
    print ("secs", secs)
    Ts = 1.0/fs_rate # sampling interval in time
    print ("Timestep between samples Ts", Ts)
    t = scipy.arange(0, secs, Ts) # time vector as scipy arange field / numpy.ndarray
    #FFT = abs(scipy.fft(signal))
    FFT_T = scipy.fft(signal)

    abs_FFT_T = abs(FFT_T)[:801]

    for index,value in enumerate(abs_FFT_T) :
        if value >= 0.70*max(abs_FFT_T[100:]) and index>=100 and index<=800 :
            if prob_ma[(index)-100] == 0 :
                truck_prob =  truck_prob + math.log(0.001)
            else :
                truck_prob = truck_prob +  math.log((prob_ma[(index)-100] * 1.0) / div)

            if prob_non_truck[(index)-100] == 0 :
                non_truck_prob = non_truck_prob +  math.log(0.001)
            else :
                non_truck_prob = non_truck_prob +  math.log((prob_non_truck[(index)-100] * 1.0 )/div_non_truck)

    deno = math.log(math.exp(truck_prob) + math.exp(non_truck_prob))
    pt = math.exp(truck_prob - deno)
    pnt = math.exp(non_truck_prob - deno)


    temp.append(pt)
    temp.append(pnt)
    if(pnt > pt):
        false_rt.append(clip)
    result.append(temp)

print (result)
print (false_rt)