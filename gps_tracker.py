#!/usr/bin/python

from socket import error as socket_error
from socket import timeout as socket_timeout

from Crypto.Random.random import StrongRandom
from Crypto.Cipher import AES

import datetime
import hashlib
import hmac
import os
import random
import signal
import socket
import sqlite3
import struct
import sys
import time

database_file = 'gps_coords_database.sqlite'

aesKey = '0123456789abdef0123456789abdef'

shaKeys = [
	'0123456789abcdef',
	'123456789abcdef0',
	'23456789abcdef01',
	'3456789abcdef012',
	'456789abcdef0123',
	'56789abcdef01234',
	'6789abcdef012345',
	'789abcdef0123456',
	'89abcdef01234567',
	'9abcdef012345678',
	'abcdef0123456789',
	'bcdef0123456789a',
	'cdef0123456789ab',
	'def0123456789abc',
	'ef0123456789abcd',
	'f0123456789abcde'
]

hexdigits = {
	'0' : 0,
	'1' : 1,
	'2' : 2,
	'3' : 3,
	'4' : 4,
	'5' : 5,
	'6' : 6,
	'7' : 7,
	'8' : 8,
	'9' : 9,
	'a' : 10,
	'b' : 11,
	'c' : 12,
	'd' : 13,
	'e' : 14,
	'f' : 15
}

cbcblock = None
aescomkey = None

myHost = ''
myPort = 7000
datalength = 1024
timeout = 60

conn = None
connInfo = None

connStep = 1

def readSocketLine():

	data = ''

	while True:
		data += conn.recv(datalength)
		if len(data) == 0:
			break
		elif data[-1] == '\n':
			break
		elif data[-1] == '\r':
			break
		elif len(data) == datalength:
			break

	return data.strip()

def doCommandRECVJPG():

	fileindex = 1
	filename = None

	jpgchrarray = []

	while True:

		filename = 'jpgfile' + str(fileindex).zfill(3) + '.jpg'

		if not os.path.exists(filename):
			break

		fileindex += 1

	jpgout = open(filename, 'wb')
	conn.send('OK\n')

	print 'Receiving JPEG.'

	while True:

		data = readSocketLine()

		if data == 'ENDRECVJPG':
			break

		if (len(data) > 0) and (not (len(data) % 2)):

			for dataindex in range(0, (len(data)/2), 1):

				byteval = (hexdigits[data[(dataindex*2)]]*16)+(hexdigits[data[(dataindex*2)+1]])
				jpgout.write(chr(byteval))

	print 'Receive JPEG complete.'
	jpgout.close()

def doCommandRECVCOORDS():

	data_tokens = None

	conn.send('OK\n')
	data = readSocketLine()

	data_tokens = data.split(',')

	if len(data_tokens) == 8:

		gps_date = '000000'
		gps_time = '000000'

		gps_latitude_degrees = None
		gps_latitude_minutes = None
		gps_latitude_direction = None

		gps_longitude_degrees = None
		gps_longitude_minutes = None
		gps_longitude_direction = None

		gps_altitude_meters = None

		try:
			gps_date = str(int(data_tokens[0])).zfill(6)
		except ValueError:
			pass

		try:
			gps_time = str(int(float(data_tokens[1]))).zfill(6)
		except ValueError:
			pass

		try:
			gps_latitude_degrees = float(data_tokens[2])
			gps_latitude_degrees = int(gps_latitude_degrees/100)
			gps_latitude_minutes = float(data_tokens[2]) - float(gps_latitude_degrees*100)
		except ValueError:
			pass

		try:
			gps_latitude_direction = str(data_tokens[3]).strip()
		except ValueError:
			pass

		try:
			gps_longitude_degrees = float(data_tokens[4])
			gps_longitude_degrees = int(gps_longitude_degrees/100)
			gps_longitude_minutes = float(data_tokens[4]) - float(gps_longitude_degrees*100)
		except ValueError:
			pass

		try:
			gps_longitude_direction = str(data_tokens[5]).strip()
		except ValueError:
			pass

		try:
			gps_altitude_meters = float(data_tokens[6])
		except ValueError:
			pass

		if (
			gps_latitude_degrees != None and
			gps_latitude_minutes != None and
			gps_latitude_direction != None and
			gps_longitude_degrees != None and
			gps_longitude_minutes != None and
			gps_longitude_direction != None
		):

			dbconn = sqlite3.connect(database_file)
			dbconn.cursor().execute("""
				INSERT INTO gps_readings (
					gps_date_year,
					gps_date_month,
					gps_date_day,
					gps_time_hour,
					gps_time_minute,
					gps_time_seconds,
					gps_latitude_degrees,
					gps_latitude_minutes,
					gps_latitude_direction,
					gps_longitude_degrees,
					gps_longitude_minutes,
					gps_longitude_direction,
					gps_altitude_meters,
					server_time
				) VALUES (
					?,?,?,?,?,?,?,?,?,?,?,?,?,?
				);
			""", (
				str(gps_date).zfill(6)[4:6],
				str(gps_date).zfill(6)[2:4],
				str(gps_date).zfill(6)[0:2],
				str(gps_time).zfill(6)[0:2],
				str(gps_time).zfill(6)[2:4],
				str(gps_time).zfill(6)[4:6],
				gps_latitude_degrees,
				gps_latitude_minutes,
				gps_latitude_direction,
				gps_longitude_degrees,
				gps_longitude_minutes,
				gps_longitude_direction,
				gps_altitude_meters,
				datetime.datetime.now()
			))

			print data

			dbconn.commit()
			dbconn.close()

			dbconn = None

		conn.send('OK\n')

	else:
		conn.send('ERROR\n')

def doClientCommands():

	global connStep

	data = readSocketLine()
	datatokens = data.split(' ')

	if datatokens[0] == 'RECVCOORDS':
		doCommandRECVCOORDS()
	elif datatokens[0] == 'RECVJPG':
		doCommandRECVJPG()
	elif datatokens[0] == 'BYE':
		connStep = 0

def doCommandHELLO():

	global connStep, cbcblock, aescomkey

	cbcblock = []
	aescomkey = []

	conn.send('HELLO\n')
	print 'Sent HELLO'
	print datetime.datetime.now()

	data = readSocketLine()

	if data == 'HELLO':
		connStep += 1
		return 1

	connStep = 0
	return 0

def doCommandSENDCOORDS():

	global connStep

	conn.send('SENDCOORDS\n')
	print 'Requested SENDCOORDS'

	data = readSocketLine()

	if data == 'RECVCOORDS':

		doCommandRECVCOORDS()
		data = readSocketLine()

		if data == 'OK':
			connStep += 1
			return 1

	connStep = 0
	return 0

def doCommandCHALLENGE():

	global connStep, cbcblock, aescomkey

	challengeFail = False

	strrand = StrongRandom()
	sysrand = random.SystemRandom()

	cipher = AES.new(aesKey, AES.MODE_ECB)

	for challengeCounter in range(0, 32, 1):

		aesblock1 = ''
		aesblock2 = ''

		msge = ''

		for i in range(0, AES.block_size, 1):
			aesblock1 += struct.pack('B', (strrand.getrandbits(8)^sysrand.randint(0, 255)))
			aesblock2 += struct.pack('B', (strrand.getrandbits(8)^sysrand.randint(0, 255)))

		randsalt = aesblock1 + aesblock2

		if challengeCounter < 16:
			cbcblock.append(aesblock1[(ord(aesblock2[0])%16)])

		aescomkey.append(aesblock2[(ord(aesblock1[0])%16)])

		aesblock1 = cipher.encrypt(aesblock1)
		aesblocktemp = aesblock2
		aesblock2 = ''

		for i in range(0, AES.block_size, 1):
			aesblock2 += chr(ord(aesblock1[i])^ord(aesblocktemp[i]))	
		aesblock2 = cipher.encrypt(aesblock2)

		msge += aesblock1.encode('hex')
		msge += aesblock2.encode('hex')

		challengeMsg = None
		challengeMsg = 'CHALLENGE ' + msge + '\n'
		conn.send(challengeMsg)

		hashchal = None
		hashchal = hmac.new(shaKeys[int(challengeCounter/2)], randsalt,  hashlib.sha256)

		data = readSocketLine()
		datatokens = data.split(' ')

		if len(datatokens) != 2 or datatokens[0] != 'RESPONSE':
			datatokens = (None, None)

		aesblocktemp = aesblock2
		aesblock1 = ''
		aesblock2 = ''

		for i in range(0, AES.block_size, 1):
			aesblock1 += chr(ord(hashchal.digest()[i])^ord(aesblocktemp[i]))
		aesblock1 = cipher.encrypt(aesblock1)

		for i in range(0, AES.block_size, 1):
			aesblock2 += chr(ord(hashchal.digest()[AES.block_size+i])^ord(aesblock1[i]))
		aesblock2 = cipher.encrypt(aesblock2)

		aesblocktemp = aesblock1 + aesblock2

		if datatokens[1] != aesblocktemp.encode('hex'):

			print
			print 'Failed challenge ' + str(challengeCounter+1) + '.'
			print shaKeys[int(challengeCounter/2)] + '\t' + randsalt.encode('hex')

			print data
			print

			challengeFail = True
		else:
			print 'Passed challenge ' + str(challengeCounter+1) + '.'

		challengeCounter += 1

	if challengeFail:

		print 'Authentication failed.'
		conn.send('CHALLENGEFAIL\n')

		data = readSocketLine()

	else:

		print 'Authentication successful.'

		#print ''.join(cbcblock).encode('hex')
		#print ''.join(aescomkey).encode('hex')

		conn.send('CHALLENGEPASS\n')

		data = readSocketLine()

		if data == 'OK':
			connStep += 1
			return 1

	connStep = 0
	return 0

def doCommandOKCLIENT():
	conn.send('OKCLIENT\n')
	doClientCommands()

def doCommandBYE():

	global connStep

	conn.send('BYE\n')
	conn.close()
	print 'Connection closed.'

	time.sleep(1)
	connStep = 1

	return 1

def serverCommands():

	global connStep

	if connStep == 1:
		return doCommandHELLO()
	elif connStep == 2:
		#return doCommandSENDCOORDS()
		connStep += 1
	elif connStep == 3:
		#return doCommandCHALLENGE()
		connStep += 1
	elif connStep == 4:
		return doCommandOKCLIENT()
	elif connStep == 0:
		return doCommandBYE()
	else:
		connStep = 0

def clientCommands():
	pass

if __name__ == '__main__':

	def sigHandler(signum = None, frame = None):

		print 'Signal handler called with signal', signum

		conn.close()
		time.sleep(1)

		sys.exit(0)

	dbconn = sqlite3.connect(database_file)
	
	try:
		dbconn.cursor().execute("""
			CREATE TABLE gps_readings (
				gps_date_year INTEGER,
				gps_date_month INTEGER,
				gps_date_day INTEGER,
				gps_time_hour INTEGER,
				gps_time_minute INTEGER,
				gps_time_seconds INTEGER,
				gps_latitude_degrees REAL,
				gps_latitude_minutes REAL,
				gps_latitude_direction TEXT,
				gps_longitude_degrees REAL,
				gps_longitude_minutes REAL,
				gps_longitude_direction TEXT,
				gps_altitude_meters REAL,
				server_time TEXT
			);
		""")
		dbconn.commit()

	except sqlite3.OperationalError, oer:
		if oer.args[0] == 'table gps_readings already exists':
			print('Create table skipped')
		else:
			print(oer.args[0])

	dbconn.close()
	dbconn = None

	socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)	# create a TCP socket
	socket.bind((myHost, myPort))		# bind it to the server port
	socket.listen(1)				# pending connections

	for sigIn in [signal.SIGTERM, signal.SIGINT, signal.SIGHUP, signal.SIGQUIT]:
		signal.signal(sigIn, sigHandler)
	
	while True:

		try:

			# wait for next client to connect
			conn, connInfo = socket.accept()	# connection is a new socket
			conn.settimeout(timeout)

			print "Received connection from " + connInfo[0]

			while True:
				serverCommands()
				if connStep == 1:
					break

		except socket_timeout, stim:

			print 'Socket timed out.'
			#doCommandBYE()
			conn.close()
			connStep = 1

		except socket_error, serr:

			if serr[0] == 104:
				print 'Received socket.error: (104, "Connection reset by peer")'
				connStep = 0
			elif serr[0] == 32:
				print 'Received socket.error: (32, "Broken pipe")'
				conn.close()
				connStep = 1
			else:
				raise serr
