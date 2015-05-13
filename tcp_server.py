#!/usr/bin/python

from socket import error as socket_error
from socket import timeout as socket_timeout

from Crypto.Random.random import StrongRandom
from Crypto.Cipher import AES

import hashlib
import hmac
import os
import random
import signal
import socket
import struct
import sys
import time

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
		if data[-1] == '\n':
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
	
def doClientCommands():

	global connStep

	data = readSocketLine()
	datatokens = data.split(' ')

	if datatokens[0] == 'RECVJPG':
		doCommandRECVJPG()
	elif datatokens[0] == 'BYE':
		connStep = 0

def doCommandHELLO():

	global connStep, cbcblock, aescomkey

	helloCounter = 1

	cbcblock = []
	aescomkey = []

	while helloCounter <= 10:

		conn.send('HELLO\n')
		print 'Sent HELLO'

		data = readSocketLine()

		if data == 'HELLO':
			connStep += 1
			return 1

		helloCounter += 1

	connStep = 0
	return 0

def doCommandCHALLENGE():

	global connStep, cbcblock, aescomkey

	challengeFail = False

	strrand = StrongRandom()
	sysrand = random.SystemRandom()

	cipher = AES.new(aesKey, AES.MODE_ECB)

	for challengeCounter in range(0, 32, 1):

		randsalt1 = ''
		randsalt2 = ''

		msge = ''

		for i in range(0, AES.block_size, 1):
			randsalt1 += struct.pack('B', (strrand.getrandbits(8)^sysrand.randint(0, 255)))
			randsalt2 += struct.pack('B', (strrand.getrandbits(8)^sysrand.randint(0, 255)))

		randsalt = randsalt1 + randsalt2

		if challengeCounter < 16:
			cbcblock.append(randsalt1[(ord(randsalt2[0])%16)])

		aescomkey.append(randsalt2[(ord(randsalt1[0])%16)])

		randsalt1 = cipher.encrypt(randsalt1)
		randsalttemp2 = randsalt2
		randsalt2 = ''

		for i in range(0, AES.block_size, 1):
			randsalt2 += chr(ord(randsalt1[i])^ord(randsalttemp2[i]))	

		msge += randsalt1.encode('hex')
		msge += cipher.encrypt(randsalt2).encode('hex')

		conn.send('CHALLENGE ' + msge + '\n')

		hashchal = None
		hashchal = hmac.new(shaKeys[int(challengeCounter/2)], randsalt,  hashlib.sha256)

		data = readSocketLine()
		datatokens = data.split(' ')

		if len(datatokens) != 2 or datatokens[0] != 'RESPONSE':
			datatokens = (None, None)

		if datatokens[1] != hashchal.hexdigest():

			print shaKeys[int(challengeCounter/2)] + '\t' + randsalt
			print hashchal.hexdigest()
			print 'Failed challenge ' + str(challengeCounter+1) + '.'
			print data

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

		print ''.join(cbcblock).encode('hex')
		print ''.join(aescomkey).encode('hex')

		conn.send('CHALLENGEPASS\n')

		data = readSocketLine()

		if data == 'OK':
			connStep += 1
			return 1

	connStep = 0
	return 0

def doCommandOK():
	conn.send('OK\n')
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
		return doCommandCHALLENGE()
	elif connStep == 3:
		return doCommandOK()
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
			doCommandBYE()

		except socket_error, serr:

			if serr[0] == 104:
				
				print 'Received socket.error: (104, "Connection reset by peer")'
				connStep = 0
			else:
				raise serr
