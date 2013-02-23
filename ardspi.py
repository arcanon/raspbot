# A robot script with keyboard and voice control using raspberry pi and atmega/adrunio.

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
    #print "sending", byteToSend, "\r"
    reply = ard.transaction( writing(bytearray([byteToSend])) )
    print "\r"
    time.sleep(0.01)
    return

ard = SPIDevice(0, 0)
ard.clock_mode = 1
ard.speed_hz = 500000
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
quit = False

p = pyaudio.PyAudio()
print "pyAudio on\r"
stream = p.open(format=FORMAT, channels=1, rate=RATE, 
                input=True, 
                output=True,
                frames_per_buffer=CHUNK_SIZE)
sample_width = p.get_sample_size(FORMAT)
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
        reply = 0;
        bytesToSend = 0
        #time.sleep(0.1)
        if key == 'q':
            quit = True
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
        else:
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
                print "silent\r"
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
                
            if snd_started and (num_silent > 10 or num_since_start  > 20) :
                stream.stop_stream()
                #LRtn = normalize(LRtn)
                print "savelen ",len(LRtn), "\r"
                LRtn = trim(LRtn)
                print "savelen ",len(LRtn), "\r"
                LRtn = add_silence(LRtn, 0.5)
                record_to_file('demo.wav', sample_width, LRtn)
                #res = Run("flac demo.wav -f --best --sample-rate 16000 -o out.flac; wget -O - -o /dev/null --post-file out.flac --header=\"Content-Type: audio/x-flac; rate=16000\" http://www.google.com/speech-api/v1/recognize?lang=pt-BR | sed -e 's/[{}]/''/g'| awk -v k=\"text\" '{n=split($0,a,\",\"); for (i=1; i<=n; i++) print a[i]; exit }' | awk -F: 'NR==3 { print $3; exit }'")
                res = Run("flac demo.wav -f --best --sample-rate 16000 -o out.flac; wget -O - -o /dev/null --post-file out.flac --header=\"Content-Type: audio/x-flac; rate=16000\" http://www.google.com/speech-api/v1/recognize?lang=en-US | sed -e 's/[{}]/''/g'| awk -v k=\"text\" '{n=split($0,a,\",\"); for (i=1; i<=n; i++) print a[i]; exit }' | awk -F: 'NR==3 { print $3; exit }'")
                
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
