import serial
from json import JSONEncoder
import requests
import re
from datetime import datetime
 
serial_port = '/dev/cu.usbserial-A603G08X'
baud_rate = 115200
base_url = "https://zapier.com/hooks/catch/ofplp7/"
#base_url = "http://requestb.in/1kbnu5p1"
 
ser = serial.Serial(serial_port, baud_rate)
json = JSONEncoder()
while True:
	line = ser.readline()
	print line
	if line.startswith('['):
		results = re.findall("(\w+)=(\d+)", line)
		dic = dict((k,int(v)) for k,v in dict(results).iteritems())
		dic['time'] = datetime.now().isoformat(' ')
		data = json.encode(dic)
		print data
		r = requests.post(base_url, data)
		print r.status_code
		print r.content
		
ser.close()






