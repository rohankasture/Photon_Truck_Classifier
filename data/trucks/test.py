from __future__ import print_function
import scipy.io.wavfile as wavfile
from scipy.signal import butter, lfilter
import scipy
import scipy.fftpack
import numpy as np
from matplotlib import pyplot as plt
# from sklearn.tree import DecisionTreeClassifier
import os
import math

matrix_len = 26
#probability matrix for truck
prob_ma = [0 for x in range(matrix_len)]
#probability matrix for non-truck
prob_non_truck = [0 for x in range(matrix_len)]

# def butter_bandpass(lowcut, highcut, fs, order=5):
#     nyq = 0.5 * fs
#     low = lowcut / nyq
#     high = highcut / nyq
#     b, a = butter(order, [low, high], btype='band')
#     return b, a
#
# def butter_bandpass_filter(data, lowcut, highcut, fs, order=5):
#     b, a = butter_bandpass(lowcut, highcut, fs, order=order)
#     y = lfilter(b, a, data)
#     return y

#train with truck samples in current directory
clip_names = []
directory = "baby"
for filename in os.listdir(directory):
    if filename.endswith(".wav"):
        clip_names.append(filename)

print (clip_names)
clip_number = 0

count_truck = 0
count_nt = 0


for clip in clip_names :
    count_truck = count_truck + 1
    print (clip)
    check = [0 for c in range(matrix_len)]
    fs_rate, signal = wavfile.read("baby//" + clip)
    signal = signal[:1024]
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

    # data = butter_bandpass(signal,200,800,44100)
    # data = butter_bandpass_filter(signal, 200, 800,44100)

    FFT = scipy.fft(signal)

    f = open("fs100.txt", "w+")
    print (len(FFT))
    for i in FFT:
        f.write(str(i))
        f.write(",")
    f.close()
    # rms_value = 0
    # for mag in abs_FFT:
    #     rms_value += mag*mag
    # rms_value = math.sqrt(rms_value)
    abs_FFT = abs(FFT)[:26]
    for index,value in enumerate(abs_FFT) :
        if value >= 0.70*max(abs_FFT):# and index>=100 and index<=800 :
            try :
                if check[(index)] != 1 :
                    prob_ma[(index)]+=1
                    check[(index)]=1
            except IndexError:
                print (clip_number,index)
                break
    clip_number+=1

<<<<<<< HEAD
print (max(prob_ma))

div = (max(prob_ma))

truck_prob = 0.5
#run time
#def calc_fft(fname):
fs_rate, signal = wavfile.read("../../no_truck.wav")
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

abs_FFT_T = abs(FFT_T)[:5001]

for index,value in enumerate(abs_FFT_T) :
    if value >= 0.60*max(abs_FFT_T[3000:]) and index>=3000 and index<=5000 :
        if prob_ma[(index/10)-300] == 0 :
            truck_prob *= 0.001
        else :
            truck_prob *= prob_ma[(index/10)-300]*1.0/div

print (truck_prob)








=======
#train with non_truck samples in non_trucks directory
clip_names = []
directory = "no_baby"
for filename in os.listdir(directory):
    if filename.endswith(".wav"):
        clip_names.append("no_baby//"+ filename)
>>>>>>> 743fbc0cc3033ae1370b845d90d9e594f4518d0d

print (clip_names)
clip_number = 0

for clip in clip_names :
    count_nt = count_nt + 1
    print (clip)
    check = [0 for c in range(matrix_len)]
    fs_rate, signal = wavfile.read(clip)
    signal = signal[:1024]
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
    print ('LENGTH of fft inpit:', len(signal))
    #FFT = abs(scipy.fft(signal))

    FFT = scipy.fft(signal)

    f = open("fs100.txt", "w+")
    print (len(FFT))
    for i in FFT:
        f.write(str(i))
        f.write(",")
    f.close()


    abs_FFT = abs(FFT)[:26]
    for index,value in enumerate(abs_FFT) :
        if value >= 0.70*max(abs_FFT): #and index>=100 and index<=800 :
            try :
                if check[(index)] != 1 :
                    prob_non_truck[(index)]+=1
                    check[(index)]=1
            except IndexError:
                print (clip_number,index)
                break
    clip_number+=1

#testing clips
test_clip_names = []
directory = "no_baby_test"
# print  test_clip_names
for filename in os.listdir(directory):
    if filename.endswith(".wav"):
        test_clip_names.append("no_baby_test//" + filename)
# print ("Rohan")
print (test_clip_names)
# print ("rohan")

#evaluate resuls
div = count_truck
div_non_truck = count_nt
result = []
false_rt = []
# minimum = 9999
count = 0
for clip in test_clip_names:
    count = count + 1
    temp = []
    truck_prob = math.log(0.5)
    non_truck_prob = math.log(0.5)
    #run time
    #def calc_fft(fname):
    fs_rate, signal = wavfile.read(clip)
    signal = signal[:1024]
    print ("Frequency sampling", fs_rate)
    l_audio = len(signal    .shape)
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

    abs_FFT_T = abs(FFT_T)[:26]

    for index,value in enumerate(abs_FFT_T) :
        if value >= 0.70*max(abs_FFT_T):# and index>=100 and index<=800 :
            if prob_ma[(index)] == 0 :
                truck_prob =  truck_prob + math.log(0.001)
            else :
                truck_prob = truck_prob +  math.log((prob_ma[(index)] * 1.0) / div)
            if prob_non_truck[(index)] == 0 :
                non_truck_prob = non_truck_prob +  math.log(0.001)
            else :
                non_truck_prob = non_truck_prob +  math.log((prob_non_truck[(index)] * 1.0 )/div_non_truck)

    # print (count)
    # if(count == 51):
    #     continue
    #     print ("hello")
    deno = math.log(math.exp(truck_prob) + math.exp(non_truck_prob))
    pt = math.exp(truck_prob - deno)
    pnt = math.exp(non_truck_prob - deno)


    temp.append(clip)
    temp.append(pt)
    temp.append(pnt)
    if(pt > pnt):
        false_rt.append(clip)
    result.append(temp)

# clf_gini = DecisionTreeClassifier(criterion = "gini",random_state = 100,max_depth=3, min_samples_leaf=5)
# clf_gini.fit(prob_ma, prob_non_truck)


print (prob_ma)
print (prob_non_truck)
# print (clf_gini)
print (result)
print  (count_truck)
print  (count_nt)
print (false_rt)


for i in range(len(prob_ma)):
    prob_ma[i] = prob_ma[i] *1.0/count_truck
for i in range(len(prob_non_truck)):
    prob_non_truck[i] = prob_non_truck[i] * 1.0/count_nt

print (prob_ma)
print (prob_non_truck)
