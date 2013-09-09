from quick2wire.spi import *
from array import array
import time
import fcntl
#import select
import pyaudio
import wave
import os
import sys, tty, termios
from struct import unpack, pack

import subprocess as sub  

from threading import Thread
import socket

data = ''
quit = False

speedRight = 100
speedLeft = 135

forwardPos = 0
backwardPos = 1
leftPos = 2
rightPos = 3
  
def Run(cmd):  
    """ 
    - Run command and return stdout as first argument of a 
      tuple and stderr as the second argument of the tuple. 
    - Returns None on error. 
    """  
    try:  
        #print "run ", cmd
        p = sub.Popen(cmd, stdout=sub.PIPE, stderr=sub.PIPE, shell=True)  
        #print "open "
        p.wait()  
        print "waiting "
        if p.returncode:  
            print "failed with code: %s" % str(p.returncode)  
        return p.communicate()  
    except OSError as e:  
        print "OSError ", e

#THRESHOLD = 17500
THRESHOLD = 500
CHUNK_SIZE = 512
FORMAT = pyaudio.paInt16
RATE = 16000

def is_silent(L):
    "Returns `True` if below the 'silent' threshold"
    return max(L) < THRESHOLD
   
def normalize(L):
    "Average the volume out"
    MAXIMUM = 16384
    times = float(MAXIMUM)/max(abs(i) for i in L)

    LRtn = array('h')
    for i in L:
        LRtn.append(int(i*times))
    return LRtn

def add_silence(L, seconds):
    "Add silence to the start and end of `L` of length `seconds` (float)"
    LRtn = array('h', [0 for i in xrange(int(seconds*RATE))])
    LRtn.extend(L)
    LRtn.extend([0 for i in xrange(int(seconds*RATE))])
    return LRtn
    
def trim(L):
    "Trim the blank spots at the start and end"
    def _trim(L):
        snd_started = False
        LRtn = array('h')

        for i in L:
            if not snd_started and abs(i)>500:
                snd_started = True
                LRtn.append(i)

            elif snd_started:
                LRtn.append(i)
        return LRtn

    # Trim to the left
    L = _trim(L)
    
    # Trim to the right
    L.reverse()
    L = _trim(L)
    L.reverse()
    return L
    
def record_to_file(path, sample_width, data):
    "Records from the microphone and outputs the resulting data to `path`"
    #sample_width, data = record()
    data = pack('<' + ('h'*len(data)), *data)

    wf = wave.open(path, 'wb')
    wf.setnchannels(1)
    wf.setsampwidth(sample_width)
    wf.setframerate(RATE)
    wf.writeframes(data)
    wf.close()

class GetCh:
    def __init__(self):
        import tty, sys

    def __call__(self):        
#        try:
            
        hasData = True
        #try:
        #    filePos = sys.stdin.tell()
        #except IOError:
        #    print "ioerror"
        #    hasData = False
        #if select.select([sys.stdin,],[],[],0.0)[0]:
            #print "Have data!"
        ch = 'u'
        try:
            ch = sys.stdin.read(1)               
        except:
            #print "No data ", ch, "\r"
            ch = 0
 #       finally:
            #termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

def sendByte(spiDevice, byteToSend):
    global speedRight
    global speedLeft
    send3Bytes(spiDevice, byteToSend, speedRight, speedLeft)
    return
		
def send3Bytes(spiDevice, byteToSend1, byteToSend2, byteToSend3):
    #print "sending", byteToSend, "\r"
    out = bytearray(4)
    indata = bytearray(4)
    out[0] = byteToSend1
    out[1] = byteToSend2
    out[2] = byteToSend3;
    out[3] = 0;
    #out[2] = 16
    #out[3] = 15
    #out = bytes(bytearray([byteToSend1, byteToSend2]))
    out = bytes(out)
    print "length %d \r" % len(out)
    p = create_string_buffer(out, len(out))
    print sizeof(p), repr(p.raw)
    #print bytes(out)
    reply = ard.transaction( duplex(out))
    #reply = ard.transaction( writing(out))
    #reply = ard.transaction( writing(bytearray([byteToSend1])) )
    #reply = ard.transaction( writing(bytearray([byteToSend2])) )
    
    #print reply
    #print ord(reply[0][1])+(ord(reply[0][2])<<8)
    print "Voltage %f" % (((float)(ord(reply[0][1])+(ord(reply[0][2])<<8)))/1024*3.3)
    print "\r"
    time.sleep(0.01)
    return

def networkfunc(mystring,*args):
    global data 
    global quit
    print mystring

    host = '192.168.178.21'
    port = 50000
    size = 4
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    print "socket created"
    s.connect((host,port))
    print "connected"
    #s.send('Hello, world')

    while not quit:
        #print "recieving"
        data = s.recv(4)
        #print type(data)
        print "recv %d\n" % size
	print data
        time.sleep(0.01)

try:
    Thread(target=networkfunc, args=('MyStringHere',1)).start()
except Exception, errtxt:
    print errtxt

ard = SPIDevice(0, 0)
print "org speed %d " % ard.speed_hz
ard.clock_mode = 1
# Set to 250000, default speed for arduino
ard.speed_hz = 4000
ard.delay = 0
mode = 1
print "mode: ", mode, "\n"
print "speed: ", ard.speed_hz, "\n"
channel = 0
leftEnable = 5;
leftForwards = 6;
leftBackwards = 7;
rightEnable = 4;
rightForwards = 3;
rightBackwards = 2;
leftStop = 15;
rightStop = 16;

#p = pyaudio.PyAudio()
print "pyAudio on\r"
#stream = p.open(format=FORMAT, channels=1, rate=RATE, 
#                input=True, 
#                output=True,
#                frames_per_buffer=CHUNK_SIZE)
#sample_width = p.get_sample_size(FORMAT)
num_silent = 0
num_since_start = 0
snd_started = False
LRtn = array('h')

fd = sys.stdin.fileno()
old_settings = termios.tcgetattr(fd)
tty.setraw(fd)
fl = fcntl.fcntl(fd, fcntl.F_GETFL)
fcntl.fcntl(fd, fcntl.F_SETFL, fl | os.O_NONBLOCK)

speechkey = 0

title_trans=''.join(chr(c) if chr(c).isupper() or chr(c).islower() else ' ' for c in range(256))

try:
    getch=GetCh()
    while not quit:
        #try:        
        key = getch()
        if key != 0:
            print "Key received ", key, "\r"
        else:
            key = speechkey
            if speechkey != 0:
                print "Speech key received ", key, "\r"
            speechkey = 0

        if key == 0 and (data):
            #print "datastr %s\n" % data
            if (ord(data[forwardPos])):
                print ("forward %d\r" % ord(data[forwardPos]))
            elif (ord(data[backwardPos])):
                print ("backward %d\r" % ord(data[backwardPos]))
            elif (ord(data[leftPos])):
                print ("left %d\r" % ord(data[leftPos]))
                sendByte(ard, rightBackwards)
                sendByte(ard, leftForwards)
                key = 0
            elif (ord(data[rightPos])):
                print ("right %d\r" % ord(data[rightPos]))
                sendByte(ard, leftBackwards)
                sendByte(ard, rightForwards)
		key = 0
            else:
                print("stop\r")
                sendByte(ard, leftStop)
                sendByte(ard, rightStop)
        
#        data = ''
        reply = 0;
        bytesToSend = 0
        #time.sleep(0.1)
        if key == 'q':
            quit = True
            sys.exit()
        elif key == 'w':
            sendByte(ard, rightForwards)
            sendByte(ard, leftForwards)
        elif key == 's':
            sendByte(ard, rightBackwards)
            sendByte(ard, leftBackwards)
        elif key == 'a':
            sendByte(ard, rightStop)
            sendByte(ard, leftForwards)
        elif key == 'd':
            sendByte(ard, leftStop)
            sendByte(ard, rightForwards)
        elif key == 'x':
            sendByte(ard, rightStop)
            sendByte(ard, leftStop)		
            #reply = ard.transaction( duplex_bytes( 1 , ( ( 8 + 1 ) << 4 ) , 0 ) )
            #print ( reply, "\n" )
        elif key == '+':
            #try record some data
            try:
                data = stream.read(CHUNK_SIZE)
            except IOError as e:
                print "I/O error({0}): {1} ".format(e.errno, e.strerror), e.strerror
                if e[1] != pyaudio.paInputOverflowed:
                    raise
                data = '\x00' * CHUNK_SIZE  # or however you choose to handle it, e.g. return None
                                
            #print "datalen ",len(data), "\r"
                
            if num_silent < 50:
                L = unpack('<' + ('h'*(len(data)/2)), data) # little endian, signed short
                L = array('h', L)
                LRtn.extend(L)

            silent = is_silent(L)
            #print silent, num_silent, L[:10]
            
            if silent and snd_started:
                #print "silent\r"
                num_silent += 1
            elif not silent and not snd_started:
                print "found something\r" 
                snd_started = True
            elif silent and not snd_started:
                #reset the data
                #print "reset\r" 
                LRtn = LRtn[len(LRtn)-CHUNK_SIZE*3: len(LRtn)]
            else:
                num_since_start += 1
                
            if False and snd_started and (num_silent > 10 or num_since_start  > 20) :
                trimStart = time.time()
                stream.stop_stream()
                #LRtn = normalize(LRtn)
                print "savelen ",len(LRtn), "\r"
                
                #LRtn = trim(LRtn)
                trimEnd = time.time()
                print "savelen ",len(LRtn), "\r"
                LRtn = add_silence(LRtn, 0.5)
                silenceEnd = time.time()
                record_to_file('demo.wav', sample_width, LRtn)
                fileEnd = time.time()
                #res = Run("flac demo.wav -f --best --sample-rate 16000 -o out.flac; wget -O - -o /dev/null --post-file out.flac --header=\"Content-Type: audio/x-flac; rate=16000\" http://www.google.com/speech-api/v1/recognize?lang=pt-BR | sed -e 's/[{}]/''/g'| awk -v k=\"text\" '{n=split($0,a,\",\"); for (i=1; i<=n; i++) print a[i]; exit }' | awk -F: 'NR==3 { print $3; exit }'")
				#res = Run("flac demo.wav -f --best --sample-rate 16000 -o out.flac; wget -O - -o /dev/null --post-file out.flac --header=\"Content-Type: audio/x-flac; rate=16000\" http://www.google.com/speech-api/v1/recognize?lang=en-US | sed -e 's/[{}]/''/g'| awk -v k=\"text\" '{n=split($0,a,\",\"); for (i=1; i<=n; i++) print a[i]; exit }' | awk -F: 'NR==3 { print $3; exit }'")
                res = Run("flac demo.wav -f --best --sample-rate 16000 -o out.flac;")
                flacEnd = time.time()
                res = Run("wget -O - -o /dev/null --post-file out.flac --header=\"Content-Type: audio/x-flac; rate=16000\" http://www.google.com/speech-api/v1/recognize?lang=en-US | sed -e 's/[{}]/''/g'| awk -v k=\"text\" '{n=split($0,a,\",\"); for (i=1; i<=n; i++) print a[i]; exit }' | awk -F: 'NR==3 { print $3; exit }'");
                runEnd = time.time()

                print ("trimT %f silenceT %f fileT %f flacT %f runT %f total %f" % (trimEnd-trimStart,silenceEnd-trimEnd,fileEnd-silenceEnd,flacEnd - fileEnd,runEnd-flacEnd,runEnd-trimStart))
				
                #print Run('ls')
                #fmt         = audiolab.Format('flac', 'pcm16')
                #nchannels   = 1
                #afile =  audiolab.Sndfile('out.flac', 'w', fmt, nchannels, 16000)
                snd_started = False
                num_silent = 0
                num_since_start = 0
                LRtn = array('h')
                tmp = '\"left\"'
                #newres = res[0].translate(title_trans).strip()
                newres = res[0]
                print "google answer", newres, "a ", type(res[0]), " ", tmp, " ", type(tmp),"\r"
                
                if "left" in newres:
                    #print "compare success"
                    speechkey = 'a'
                elif "finish" in newres:
                    speechkey = 'x'
                elif "right" in newres:
                    speechkey = 'd'
                elif "forward" in newres:
                    speechkey = 'w'
                elif "backward" in newres:
                    speechkey = 's'
                elif "quit" in newres:
                    speechkey = 'q'                    
                stream.start_stream()
                #break                

            
        #except KeyboardInterrupt:
        #    break
except Exception as e:
    print "execption", e, "\r"
finally:
    print "exit"
    ard.close()
    termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
