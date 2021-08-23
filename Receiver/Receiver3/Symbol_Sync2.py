import scipy
import numpy as np
import math
import matplotlib.pylab as plt
import random


def BPSK_data (Number_of_bits):
    Data = []
    for i in range (Number_of_bits):
        random_binary = random.randint(0,1)
        BPSK = 2*random_binary-1
        Data.append(BPSK)
    return Data

def Pulse_shape (Sample_per_symbol,Data):
    Sine = []
    Data_up = np.zeros(len(Data)*Sample_per_symbol)
    index = [i for i in np.arange(0,len(Data_up),Sample_per_symbol)]
    
    for i in range(Sample_per_symbol):
        Sine_wave = math.sin(2*np.pi*i/(2*Sample_per_symbol))
        Sine.append(Sine_wave)
        
    for j in range(len(Data)):
        aux = index[j]
        Data_up[aux] = Data[j]

    Pulse_shape = np.convolve(Data_up,Sine)
    return Pulse_shape



Symbols = 3*10**4
Sample_per_Symbol = 100


MSE = np.zeros(5)
BER_sim = np.zeros(5)
    
Data = BPSK_data(Symbols)
Pulse = Pulse_shape(Sample_per_Symbol,Data)

SNR = []
normal_rand = []
SNRdB = [i for i in np.arange(2,11,2)]
for i in SNRdB:
    SNR.append(10 ** (i/10))


Data = BPSK_data(Symbols)
Pulse = Pulse_shape(Sample_per_Symbol,Data)

for cv in range(0,len(SNR)):
    re = []

    noise = np.zeros(len(Pulse))
    for j in range (len(Pulse)):
        noise[j] = math.sqrt(1/(2*SNR[cv]))*np.random.randn()
    Received = Pulse + noise
##    plt.subplot(2,1,1)
##    plt.plot(Received[0:700])
##    plt.subplot(2,1,2)
##    plt.plot(Pulse[0:700])
##    plt.show()

    tau = 0
    a = np.zeros(Symbols)
    cenpoint = np.zeros(Symbols)
    remind = np.zeros(Symbols)
    delta = int(Sample_per_Symbol/2)
    center = int(60)
    avgsamples = 6
    stepsize = 1
    rit = -1
    GA = np.zeros(avgsamples)
    tauvector = np.zeros(1900)

    uor = -1

    for j in np.arange((Sample_per_Symbol/2),len(Received)-Sample_per_Symbol/2-1,Sample_per_Symbol):
        rit = rit+1
        re.append(rit)
        midsample = Received[center]
        latesample = Received[center+delta]
        earlysample = Received[center-delta]
        a[rit] = earlysample

        sub = latesample-earlysample
        GA[np.mod(rit,avgsamples)] = sub*midsample

        if GA.mean(axis=0) > 0:
            tau = -stepsize
        else:
            tau = stepsize

        cenpoint[rit] = center
        remind[rit] = np.remainder((center-Sample_per_Symbol/2),Sample_per_Symbol)

        if rit >= 99 and rit < 1999:
            uor = uor + 1
            tauvector[uor] = (remind[rit] - Sample_per_Symbol/2)**2
        center=center+Sample_per_Symbol+tau
        
        if center >= len(Received)-(Sample_per_Symbol/2)-2:
            break
        
    MSE[cv] = tauvector.mean(axis=0)
    plt.figure
    Symb = 200
    plt.subplot(2,1,1)
    plt.plot(remind[0:200],"-*")

    lim1 = 40*np.ones(Symb)
    lim2 = 60*np.ones(Symb)

    plt.plot(lim1)
    plt.plot(lim2)


    print("SNRdB =",SNRdB[cv])
    plt.title('Convergence plot for BPSKGardner')
    plt.ylabel('tau axis')
    plt.xlabel('iterations')

    plt.subplot(2,1,2)
    Symb = 2000
    plt.plot(remind[0:Symb],"-*")
    lim1 = 40*np.ones(Symb)
    lim2 = 60*np.ones(Symb)
    plt.plot(lim1)
    plt.plot(lim2)
    plt.title('Convergence plot for BPSKGardner')
    plt.ylabel('tau axis')
    plt.xlabel('iterations')
    plt.show()

    
    Error = 0
    for k in range(0,Symbols-1):
        if (a[k] > 0 and Data[k] == -1) or (a[k]<0 and Data[k] == 1):
            Error = Error + 1
    BER_sim[cv] = Error/(Symbols-2)
    print(BER_sim[cv])

    BER_th = []


for k in SNR:
    BER_th_aux = (1/2)*math.erfc(math.sqrt(2*k)/math.sqrt(2))
    BER_th.append(BER_th_aux)
plt.figure()
plt.semilogy(SNRdB,BER_th,"k-",linewidth = 2)
plt.semilogy(SNRdB,BER_sim,"r-",linewidth = 2)

plt.show()


    



