import sys
import os
sys.path.append(os.path.dirname(os.path.realpath(__file__)))
from z3 import Bool, Reals, And, Or, is_true, BoolSort, simplify, Ints
from z3 import is_rational_value, Solver, sat, Not, RealVal
from z3 import *

from BPpy.model.bprogram import BProgram
from BPpy.model.event_selection.experimental_smt_event_selection_strategy import ExperimentalSMTEventSelectionStrategy, Request, Block, WaitFor

import socket
import time
import threading
import math

leaderSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
roverSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

def SplitInData (s):
       s1=s.split(';')
       s2=s1[0].split(',')
       return s2



def getTelem():
    time.sleep(0.03)
##    print ("started telem")
    roverSocket.sendall(b"player2,GPS()\n")
    rData = repr(roverSocket.recv(1024))
    RoverGPSData = SplitInData(rData)
    RoverPx = float(RoverGPSData[1])
    RoverPy = float(RoverGPSData[2])
##    print ("Rover Pos x:", RoverPx)
##    print ("Rover Pos y:", RoverPy)
        
    roverSocket.sendall(b"player2,getCompass()\n")
    rData = repr(roverSocket.recv(1024))
    RoverCompassData = SplitInData(rData)
    Compass = float(RoverCompassData[1])
##    print ("Compass:", Compass)
        
    roverSocket.sendall(b"ball,GPS()\n")
    lData = repr(roverSocket.recv(1024))
    LeaderGPSData = SplitInData(lData)
    LeaderPx = float(LeaderGPSData[1])
    LeaderPy = float(LeaderGPSData[2])
##    print ("Leader Pos x:", LeaderPx)
##    print ("Leader Pos y:", LeaderPy)
        
    LeaderDistanceData = pow(((RoverPx-LeaderPx)*(RoverPx-LeaderPx)+(RoverPy-LeaderPy)*(RoverPy-LeaderPy)),(1/2))
    Dist = LeaderDistanceData
##    print ("Distance:", Dist)

    LRDeg = math.atan2((LeaderPx - RoverPx), -(LeaderPy - RoverPy))
    LRDeg = (LRDeg / math.pi) * 180
    DDeg = (90 - Compass) - LRDeg

    if (abs(DDeg) >= 360):
        if (DDeg > 0):
            DDeg = DDeg - 360;
        else:
            DDeg = DDeg + 360;

    if (abs(DDeg) > 180):
        if (DDeg > 180):
            DDeg = DDeg - 360
            
        if (DDeg < (-180)):
            DDeg = DDeg + 360
##    print ("DDeg:",DDeg)
##    print ("#################################")

    GLRDeg = math.atan2((-54 - RoverPx), -(0 - RoverPy))
    GLRDeg = (GLRDeg / math.pi) * 180
    GDDeg = (90 - Compass) - GLRDeg

    if (abs(GDDeg) >= 360):
        if (GDDeg > 0):
            GDDeg = GDDeg - 360;
        else:
            GDDeg = GDDeg + 360;

    if (abs(GDDeg) > 180):
        if (GDDeg > 180):
            GDDeg = GDDeg - 360
            
        if (GDDeg < (-180)):
            GDDeg = GDDeg + 360
##    print ("GDDeg:",GDDeg)
##    print ("#################################")
    return [Dist, DDeg, GDDeg]
    


true = BoolSort().cast(True)
false = BoolSort().cast(False)

reset = Bool('reset')
BInRobot = Bool('BInRobot')
forward,spin, Su = Ints('forward spin Su')
dist = Real( 'dist')
degToBall = Real( 'degToBall')
degToGoal = Real( 'degToGoal')

# B-Threads
global TooFar, TooClose, MaxPower, MaxSpin, MaxAngToBall, MaxAngToGoal

TooFar=5
TooClose=3.7
MaxPower=100
MaxSpin=100
MaxAngToBall=10
MaxAngToGoal=3

def bounds():
       yield {Block: Not(And( forward>= -MaxPower, forward <= MaxPower,Or(spin==0,spin==MaxSpin,spin==-MaxSpin))),WaitFor: false}



       
def forwardMotion():
       m=yield {}
       while True:
              Dist=toFloat(m[dist])
              if (Dist>TooClose):
                     if (Dist<TooFar):
                            m=yield {Request([spin,forward]): forward == ((Dist-TooClose)/(TooFar-TooClose))*MaxPower}
                     else:
                            m=yield {Request([spin,forward]): forward == MaxPower}
                     
              else:
                     if (Dist>(2*TooClose-TooFar)):
                            m=yield {Request([spin,forward]): forward==((Dist-TooClose)/(TooFar-TooClose))*MaxPower}
                     else:
                            m=yield {Request([spin,forward]): forward==-MaxPower} 


def spinToBall():
       m=yield {}
       while True:
           DegToBall=toFloat(m[degToBall])
           if is_false(m[BInRobot]):
              if (DegToBall > MaxAngToBall):
                     m=yield {Request([spin,forward]): spin>0,
                              Block: spin<=0}
              elif (DegToBall < -MaxAngToBall):
                     m=yield {Request([spin,forward]): spin<0,
                              Block: spin>=0}
              else:
                     m=yield {Request([spin,forward]): spin==0}
           else:
              m=yield {}
              

def spinToGoal():
       m=yield {}
       while True:
           DegToGoal=toFloat(m[degToGoal])   
           if is_true(m[BInRobot]):
              if (DegToGoal > MaxAngToGoal):
                     m=yield {Request([spin,forward]): spin>0,
                              Block: spin<=0}
              elif (DegToGoal < -MaxAngToGoal):
                     m=yield {Request([spin,forward]): spin<0,
                              Block: spin>=0}
              else:
                     m=yield {Request([spin,forward]): spin==0}
           else:
              m=yield {}


def getBall():
       m=yield {}
       while True:
           Dist=toFloat(m[dist])   
           if is_false(m[BInRobot]):
              if Dist<4.3:
                     m=yield {Request([Su,BInRobot]):And(Su==-100, BInRobot==true),
                              Block: Or(Su!=-100, BInRobot==false)}
              else:
                     m=yield {Request([Su,BInRobot]):And(Su==-100, BInRobot==false),
                              Block: Or(Su!=-100, BInRobot==true)}  
           else:
               m=yield {}
           


def shootBall():
       m=yield {}
       while True:
            DegToGoal=toFloat(m[degToGoal])  
            if is_true(m[BInRobot]):
              if abs(DegToGoal)<=MaxAngToGoal:
                  m=yield {Request([Su,BInRobot]):And(Su==100, BInRobot==false),
                           Block: Or(Su!=100, BInRobot==true)}
              else:
                  m=yield {Request([Su,BInRobot]):And(Su==-100, BInRobot==true),
                           Block: Or(Su!=-100, BInRobot==false)}     
            else:
                m=yield {}

def noForwardWhileNotAligned():
      yield {}
      yield {Block: Not(Implies(Not(spin==0), forward==0)),WaitFor: false}
##       m=yield {}
##       while True:
##           DegToGoal=toFloat(m[degToGoal])
##           DegToBall=toFloat(m[degToBall])
##           if is_true(m[BInRobot]):
##              if (abs(DegToGoal)>8):
##                      m=yield {Block: forward!=0}
##              else:
##                      m=yield {}
##           else:
##              if (abs(DegToBall)>10):
##                      m=yield {Block: forward!=0}
##              else:
##                      m=yield {}

	   
def telemUpdater():
       yield {}
       while True:
              [Dist, DDeg, GDDeg]=getTelem()
              yield {Request([dist, degToBall, degToGoal]):And(dist==Dist, degToBall==DDeg,degToGoal==GDDeg),
                     Block: Or(dist!=Dist, degToBall!=DDeg,degToGoal!=GDDeg)}

def initReq():
       m=yield {Request([forward,spin,Su,BInRobot,dist, degToBall, degToGoal]):And(forward == 0, spin == 0, Su==0, BInRobot==False,dist==0, degToBall==0, degToGoal==0),
                Block: Or(forward != 0, spin != 0, Su!=0, BInRobot==True,dist!=0, degToBall!=0, degToGoal!=0)}
       

       
def logger():
       m=yield {}
       m=yield {}
       while True:
              m=yield {}

              ForwardVal=m[forward]
              SpinVal=m[spin]
              Suction=m[Su]
  
              stringout="player2,moveForward("+str(ForwardVal)+")\n"
              roverSocket.sendall(stringout.encode('utf-8'))
              print (stringout)
              time.sleep(0.01)   

              stringout="player2,setSuction("+str(Suction)+")\n"
              roverSocket.sendall(stringout.encode('utf-8'))
              print (stringout)
              time.sleep(0.01)

              stringout="player2,spin("+str(SpinVal)+")\n"
              roverSocket.sendall(stringout.encode('utf-8'))
              print (stringout)
              
              if (SpinVal.as_long()!=0):
                     time.sleep(0.03)
                     stringout="player2,spin(0)\n"
                     roverSocket.sendall(stringout.encode('utf-8'))
                     print (stringout)

                     

def toFloat(r):
    return r.numerator_as_long() / r.denominator_as_long()

roverSocket.connect(('127.0.0.1', 9003))
print ("after connect")
getTelem()
time.sleep(0.1)

if __name__ == "__main__":
    b_program = BProgram(bthreads=[
       bounds(),
       initReq(),
       forwardMotion(),
       spinToGoal(),
       spinToBall(),
       getBall(),
       shootBall(),
       logger(),
       telemUpdater(),
       noForwardWhileNotAligned()
    ], event_selection_strategy=ExperimentalSMTEventSelectionStrategy())
b_program.run()


