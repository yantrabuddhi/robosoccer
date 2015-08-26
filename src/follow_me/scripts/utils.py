import time

def clamp(low, value, high):
	return max(min(high,value),low)

def waitForBT():
	print "Waiting for bluetooth to connect..."
	time.sleep(6)
	print "Connected"
def delay(t_sec):
	time.sleep(t_sec)
