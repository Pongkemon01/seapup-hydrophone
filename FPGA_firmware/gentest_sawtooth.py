import numpy as np
from matplotlib import pyplot as plt

fs = 1000000    # Sampling frequency
f = 25000       # Frequency of the pinger
period = 0.01   # Period of the signal activity (10 ms)
pre_silent1 = 0.0003 # Silent period before signal of hydrophone 1
pre_silent2 = 0.00031 # Silent period before signal of hydrophone 2
pre_silent3 = 0.00031 # Silent period before signal of hydrophone 3
pre_silent4 = 0.00033 # Silent period before signal of hydrophone 4

# Silent period of each channel after signal. Channel 1 is the master. all others are calculate from channel 1 to make array length same
post_silent1 = 0.0003 # Silent period after signal of hydrophone 1
total1 = pre_silent1 + post_silent1
post_silent2 = total1 -  pre_silent2 # Silent period after signal of hydrophone 2
post_silent3 = total1 -  pre_silent3 # Silent period after signal of hydrophone 3
post_silent4 = total1 -  pre_silent4 # Silent period after signal of hydrophone 4

# Generate time axis
t = np.linspace(0, period, np.int32(fs * period))
# Generate signal
total_signal = np.int32( ( fs * period ) - total1 )
y1 = np.zeros( total_signal )
y2 = np.zeros( total_signal )
y3 = np.zeros( total_signal )
y4 = np.zeros( total_signal )

y1_max = 680
y2_max = 100
y3_max = 150
y4_max = 60

y1_count = 0
y2_count = 0
y3_count = 0
y4_count = 0

for x in range( total_signal ):
    y1[x] = y1_count
    y2[x] = y2_count
    y3[x] = y3_count
    y4[x] = y4_count

    y1_count = y1_count + 1
    y2_count = y2_count + 1
    y3_count = y3_count + 1
    y4_count = y4_count + 1

    if( y1_count > y1_max ):
        y1_count = 0
    if( y2_count > y2_max ):
        y2_count = 0
    if( y3_count > y3_max ):
        y3_count = 0
    if( y4_count > y4_max ):
        y4_count = 0


# Final signal
s1 = np.concatenate( ( np.zeros(np.int32(pre_silent1 * fs)), y1, np.zeros(np.int32(post_silent1 * fs)) ) )
s2 = np.concatenate( ( np.zeros(np.int32(pre_silent2 * fs)), y2, np.zeros(np.int32(post_silent2 * fs)) ) )
s3 = np.concatenate( ( np.zeros(np.int32(pre_silent3 * fs)), y3, np.zeros(np.int32(post_silent3 * fs)) ) )
s4 = np.concatenate( ( np.zeros(np.int32(pre_silent4 * fs)), y4, np.zeros(np.int32(post_silent4 * fs)) ) )

# convert to int16
s1 = np.uint16(np.int16(s1))
s2 = np.uint16(np.int16(s2))
s3 = np.uint16(np.int16(s3))
s4 = np.uint16(np.int16(s4))

minsize = np.min([s1.size, s2.size, s3.size, s4.size, 10000])
for i in range(minsize):
    print("%04X%04X%04X%04X"%(s1[i],s2[i],s3[i],s4[i]))

