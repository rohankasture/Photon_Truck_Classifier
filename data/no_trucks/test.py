from __future__ import print_function
import scipy.io.wavfile as wavfile
import scipy
import scipy.fftpack
import numpy as np
from matplotlib import pyplot as plt
import os

#probability matrix for truck
prob_ma = [0 for x in range(205)]
#probability matrix for non-truck



clip_names = []
directory = "."
for filename in os.listdir(directory):
    if filename.endswith(".wav"):
        clip_names.append(filename)

print (clip_names)
clip_number = 0

for clip in clip_names :
    print (clip)
    check = [0 for c in range(205)]
    #def calc_fft(fname):
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
            #print str(abs(c[i]))
            #print "\n"
    f.close()


    #abs(-564647+)
    #print(FFT[:5])
    #print (FFT[22049:22052])
    abs_FFT = abs(FFT)[:5001]
    #rint(abs_FFT.argmax())
    #print (abs_FFT[abs_FFT.argmax()])
    for index,value in enumerate(abs_FFT) :
        if value >= 0.60*max(abs_FFT[3000:]) and index>=3000 and index<=5000 :
            #print(clip_number, index)
            try :
                if check[(index/10)-300] != 1 :
                    prob_ma[(index/10)-300]+=1
                    check[(index/10)-300]=1
            except IndexError:
                print (clip_number,index)
                break
    clip_number+=1

print (max(prob_ma))
div = (max(prob_ma))

truck_prob = 0.5
#run time
#def calc_fft(fname):
fs_rate, signal = wavfile.read("../../truck_1.wav")
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















#
# FFT_side = FFT[range(N/2)] # one side FFT range
# freqs = scipy.fftpack.fftfreq(signal.size, t[1]-t[0])
# fft_freqs = np.array(freqs)
# freqs_side = freqs[range(N/2)] # one side frequency range
# fft_freqs_side = np.array(freqs_side)
# plt.subplot(311)
# p1 = plt.plot(t, signal, "g") # plotting the signal
# plt.xlabel('Time')
# plt.ylabel('Amplitude')
# plt.subplot(312)
# p2 = plt.plot(freqs, FFT, "r") # plotting the complete fft spectrum
# plt.xlabel('Frequency (Hz)')
# plt.ylabel('Count dbl-sided')
# plt.subplot(313)
#
# # f = open(".txt", "w+")
# # for i in FFT_side:
# #     f.write(str(abs(i)))
# #     f.write(",")
# #         #print str(abs(c[i]))
# #         #print "\n"
# # f.close()
#
# p3 = plt.plot(freqs_side, abs(FFT_side), "b") # plotting the positive fft spectrum
# plt.xlabel('Frequency (Hz)')
# plt.ylabel('Count single-sided')
# plt.show()
#
#
# # directory = "."
# # for filename in os.listdir(directory):
# #     if filename.endswith(".wav"):
# #         calc_fft(filename)
