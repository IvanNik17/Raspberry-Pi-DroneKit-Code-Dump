# -*- coding: utf-8 -*-
"""
@author: Kasper Skou Ladefoged
"""
import numpy as np
import matplotlib.mlab as mlab
import matplotlib.pyplot as plt
from sweeppy import Sweep

def main():
	with Sweep('COM5') as sweep:
		sweep.start_scanning()

	    # Infinit loop, the loop i executed everytime a new scan is present from the LIDAR
		for scan in sweep.get_scans():
			lidarSamples = scanToNpArray(scan)
			filterSignalStrength(lidarSamples,40)
			filterDistance(lidarSamples,1,40000)
			# histogramSignalStrength(lidarSamples)
	pass

def histogramSignalStrength(data):
	# the histogram of the data
	# print(data[:,2])
	n, bins, patches = plt.hist(data[:,2], 50, facecolor='green', alpha=0.75)

	plt.ion()
	# plt.axis([40, 160, 0, 0.03])
	plt.grid(True)
	plt.show()
	plt.pause(0.05)
	plt.gcf().clear()


# Outputs in the order of, Angle, Distance, SignalStrength
def scanToNpArray(scan):
	npSamples = np.zeros((len(scan.samples),3))
	for idx, s in enumerate(scan.samples):
		npSamples[idx,0] = s.angle
		npSamples[idx,0] = npSamples[idx,0]/1000
		npSamples[idx,1] = s.distance
		npSamples[idx,1] = npSamples[idx,1]*10
		npSamples[idx,2] = s.signal_strength

	return npSamples

def filterSignalStrength(data,threshValue):
	mask = np.ones(len(data), dtype=bool)
	for idx, s in enumerate(data):
		if s[2] < threshValue:
			mask[idx] = False
	return data[mask,...], data[np.invert(mask),...]

def filterDistance(data,minDist,maxDist):
	mask = np.ones(len(data), dtype=bool)
	for idx, s in enumerate(data):
		if s[1] < minDist or s[1] > maxDist:
			mask[idx] = False
	return data[mask,...], data[np.invert(mask),...]


def changeSettings(sweep, motorSpeed, sampleRate):
	if(sweep.get_motor_speed() != motorSpeed):
		sweep.set_motor_speed(motorSpeed)

	if(sweep.get_sample_rate() != sampleRate):
		sweep.set_sample_rate(sampleRate)

if __name__ == "__main__":
	main() 
