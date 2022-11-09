#!/usr/bin/python
# -*- coding: utf-8 -*-
import serial
import threading
import re
import time
import struct
from array import *
import numpy as np
import time
import crc8


class CEvoHub(object):
#{
    def __init__(self):
    #{
        self.LastDistance = [1,1,1,1]
        self.portname = 'COM7'  # /dev/ttyACM0 To be adapted if using UART backboard
        self.baudrate = 115200  # 3000000 for UART backboard

        # Configure the serial connections (the parameters differs on the device you are connecting to)

        self.port = serial.Serial(port=self.portname,
                                  baudrate=self.baudrate,
                                  parity=serial.PARITY_NONE,
                                  stopbits=serial.STOPBITS_ONE,
                                  bytesize=serial.EIGHTBITS)

        self.port.isOpen()
        self.serial_lock = threading.Lock()
    #}
    
    
    def send_command(self, command):
    #{
        # #command = "\x00\x11\x01\x45" ##text formatna

        with self.serial_lock:  # This avoid concurrent writes/reads of serial
        #{
            self.port.flushInput()
            self.port.write(command)  #
            self.port.flushOutput()
        #}
        return 1
    #}

    def SendTowerMode( self ):
    #{
        if self.send_command( bytearray( [ 0x00, 0x31, 0x03, 0xE5] ) ): 
        #{
            BytesReaded = self.Read( 4 )
        #}
    #}
    
    def SendPrintOutMode( self ):
    #{
        if self.send_command( bytearray( [ 0x00, 0x11, 0x02, 0x4C ] ) ):
        #{
            BytesReaded = self.Read( 4 )
        #}
    #}
    
    def SendTest( self ):
    #{
        if self.send_command( bytearray( [ 0x54, 0x45, 0x53, 0x54 ] ) ):
        #{
            BytesReaded = self.Read( 4 )
        #}
    #}
    
    def start_sensor(self):
    #{
        #print( str.encode("\x00"))
        if self.send_command( bytearray( [ 0x00, 0x52, 0x02, 0x01, 0xDF ] ) ):
        #{
            BytesReaded = self.Read( 4 )
            #print( "type ", type(BytesReaded) )             
        #if self.send_command( str.encode( "\x00\x11\x02\x4C" ) ):
            if len( BytesReaded ) >= 4:
            #{
                #if BytesReaded[0] == 0x30 and BytesReaded[1] == 0x05 and BytesReaded[2] == 0xFF and BytesReaded[3] == 0x53:
                #if np.array_equal( BytesReaded, [ 0x30,0x05,0xFF,0x53] ) :
                if np.array_equal( BytesReaded, [ 0x30,0x05,0x00,0xA0] ) :
                #{
                    print( 'Sensor started successfully' )
                #}
                else:
                #{
                    print( 'Sensor return ', len( BytesReaded ), " - ", BytesReaded )
                #}
            #}
        #}       
        #tem que retornar 30 05 00 A0

    # def text_sensor(self):
    #    ....if self.send_command("\x00\x11\x01\x45"):
    # ........print "Sensor in text mode successfully"
    #}

    def CheckCRC( self, BytesReaded ) :
    #{
        Result = 0
        hash = crc8.crc8()
       #my_array = np.array( BytesReaded[1:31] )
       #print( my_array )
       #print( BytesReaded )
        SValue = ""
        for i in BytesReaded[0:19] :
            #hash.update( hex( i ).encode("UTF-8") )
            SValue = SValue + format(i, '02X')
       
        #print( SValue ) 
        #hash.update( ( "0x"+SValue ).encode( "UTF-8" )  )
        #hash.update( bytes.fromhex( SValue ) )
        hash.update( bytearray.fromhex( SValue ) )
        #hash.update( bytes.fromlist( BytesReaded[0:18] ) )
        #print( SValue.encode("UTF-8") )
        #print( "CRC ", int( hash.hexdigest(), 16 ), " ", BytesReaded[19] )
        
        if int( hash.hexdigest(), 16 ) == BytesReaded[19] :
        #{
            #Result = 1
            #print( "CRC ok " )

            hash = crc8.crc8()
            #my_array = np.array( BytesReaded[1:31] )
            #print( my_array )
            #print( BytesReaded )
            SValue = ""
            for i in BytesReaded[20:31] :
                #print(i)
                SValue = SValue + format(i, '02X')
                #hash.update( hex( i ).encode("UTF-8") )
                #print( hex( i ).encode("UTF-8") ) 
                #hash.update( BytesReaded.tounicode() ) 
            
            hash.update( bytearray.fromhex( SValue ) )
            #print( "CRC ", int( hash.hexdigest(), 16 ), " ", BytesReaded[31] )
            
            if int( hash.hexdigest(), 16 ) == BytesReaded[31] :
                Result = 1
                #print( "CRC ok " )
        #}    
        return Result;
    #}

    def ReadSensor( self, SensorCount ):
    
    #  54 48 09 b7 00 01 09 c2 00 01 09 c1 00 01 09 c2 00 01 55 3a 49 4d 01 00 8f c0 71 07 7a 00 11 6b 
    # 84, 72, 9, 204, 0, 1, 9, 204, 0, 1, 9, 190, 0, 1, 9, 194, 0, 1, 85, 135, 73, 77, 1, 0, 142, 192, 113, 7, 121, 0, 45, 189
    # 84, 72 - [ 9, 204, 0, 1, 9, 204, 0, 1, 9, 190, 0, 1, 9, 194, 0, 1, 85, 135,]  73, 77 - [ 1, 0, 142, 192, 113, 7, 121, 0, 45, 189]

    # 20 dos sensores de distância e 12 dos sensores de orientação
    
    # depois do 73 e 77 ignorar, leitura dos sensores de orientação
    # 0x54 0x48 
    # 2 bytes por sensor
    # 1 byte - mascara, cada bit corresponde a um sensor
    # 1 byte - CRC8 
        #BytesReaded = self.Read( 2+(SensorCount*2)+1+1 )
        StrDistance = ""
        BytesReaded = self.Read( 32 )
        #print( BytesReaded )
        if len( BytesReaded ) >= 32 :
        #{
           if self.CheckCRC( BytesReaded ):
           #{
               if BytesReaded[0] == 0x54 and BytesReaded[1] == 0x48:
               #{
                    #print( 'ok' )
                    Pos = 0
                    for i in range(2,16,4) :
                    #{
                        Distance = BytesReaded[i]*256 + BytesReaded[i+1] 
                        StrDistance =  StrDistance + str(Pos) + ": " + str( Distance/10 ) + "cm "
                        #print( "-", Pos, Distance, self.LastDistance[Pos]  ) 
                        #if Distance / self.LastDistance[Pos] > 10 :
                            #StrDistance =  StrDistance + str(Pos) + ": " + str( Distance/10 ) + "cm "
                            #self.LastDistance[Pos] = Distance 
                       
                        #if Distance / self.LastDistance[Pos] < 100 :
                         #   StrDistance = StrDistance + str(Pos) + ": " + str( Distance/10 ) + "cm "
                          #  self.LastDistance[Pos] = Distance
                       
                        Pos = Pos + 1   
                  #}
                #}
            #}
          
               
        #}   
        print( "Distance ", StrDistance )
        
        return BytesReaded
    
    def Read(self, Size ):
        with self.serial_lock:
            bytes = self.port.read(Size)

        #print( "byte ", bytes )
        bytes = struct.unpack('>' + str( Size ) + 'B', bytes)
        #print( "Bytes ", bytes )
        return np.array( array( 'i', bytes ) )



    def run(self):
    #{

        # print "mesafe"

        self.port.flushInput()
        #self.SendTest()
        #time.sleep( 1 )
        self.SendPrintOutMode()
        #time.sleep( 1 )        
        #self.SendPrintOutMode()
        # se não mandar esse comando, responde 32 bytes.
        self.SendTowerMode()
        
        self.start_sensor()
        #print( "Start ", self.get_mesafe() )
    #}
    
#}

    
if __name__ == '__main__':
#{
    EvoHub = CEvoHub()
    EvoHub.run()

    # print "mesafe"
    while 1:
    #{
      #x = evo_60m.get_mesafe()  # you can use x as returned distance
      #print( "X ", x )
      EvoHub.ReadSensor( 8 )
      #time.sleep( 1 )
    #}
    
 #}