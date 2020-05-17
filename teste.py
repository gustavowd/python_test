import sys
from PyQt5.QtCore import pyqtSlot
from PyQt5.QtWidgets import QApplication, QDialog, QMainWindow
from PyQt5.uic import loadUi
from PyQt5 import QtWidgets, uic
from pyqtgraph import PlotWidget
import pyqtgraph as pg

from PySide2 import QtCore
from PyQt5.QtCore import QDate, QTime, QDateTime, Qt

from ctypes import cdll, c_long, c_int, c_char_p, create_string_buffer

import serial
import numpy as np
from random import randint

import threading

class MainWindow(QMainWindow):
    graph_data_x = []
    graph_data_y = []
    counter = 0
    idx = 0.0
    overflow = False
    pen = pg.mkPen(color=(255, 0, 0))
    data_line = 0

    def __init__(self):
        super(MainWindow, self).__init__()
        loadUi('teste.ui', self)
        self.setWindowTitle('Teste')
        self.pushButton.clicked.connect(self.on_pushButton_clicked)
        self.pushButton2.clicked.connect(self.on_pushButton2_clicked)
        self.pushButton3.clicked.connect(self.on_pushButton3_clicked)
        self.pushButton4.clicked.connect(self.on_pushButton4_clicked)
        self.timer1 = QtCore.QTimer()
        self.timer1.timeout.connect(self.showTime)
        #self.timer1.start(100)
        self.counter = 0
        self.graphWidget.setLabel('left', "<span style=\"color:red;font-size:30px\">Paw (cmH2O)</span>")
        self.graphWidget.setLabel('bottom', "<span style=\"color:red;font-size:30px\">Time (s)</span>")
        self.graphWidget.setBackground('w')
        self.graphWidget.setYRange(0.0, 45.0, padding=0)
        self.graphWidget.setXRange(0.0, 20.0, padding=0)

    def showTime(self):
        time = QTime.currentTime()
        self.upTime.setText(time.toString(Qt.DefaultLocaleLongDate))

        if (self.counter > 200):
            self.overflow = True
            self.counter = 0
            self.idx = 0.0

        if self.overflow == True:
            #self.graph_data_y[self.counter] = self.counter#np.nan
            #self.graph_data_x = self.graph_data_x[1:]  # Remove the first y element.
            #self.graph_data_x.append(self.graph_data_x[-1] + 0.1)  # Add a new value 1 higher than the last.

            #self.graph_data_y = self.graph_data_y[1:]  # Remove the first 
            regs = [0]
            read_input_reg(10, 50000, 1, regs)
            value = float(regs[0]) / 10.0
            #self.graph_data_y.append(value)
            self.graph_data_y[self.counter] = value
            for i in range(self.counter + 1, self.counter + 10):
                if (i > 200):
                    break
                self.graph_data_y[i] = 0#np.nan

            #con = np.isfinite(self.graph_data_y)
            #print(con)
            #self.graph_data_y.append( self.counter)  # Add a new random value.
            #self.data_line.setData(self.graph_data_x, self.graph_data_y, connect=np.logical_and(con, np.roll(con, -1)))  # Update the data.
            self.data_line.setData(self.graph_data_x, self.graph_data_y, connect="finite")  # Update the data.
        else:
            self.graph_data_x.insert(self.counter, self.idx)
            #self.graph_data_y.insert(self.counter, self.counter ** 2)
            regs = [0]
            read_input_reg(10, 50000, 1, regs)
            value = float(regs[0]) / 10.0
            self.graph_data_y.insert(self.counter, value)

            if self.data_line == 0:
                self.data_line = self.graphWidget.plot(self.graph_data_x, self.graph_data_y, connect="finite", pen=self.pen)
            else:
                self.data_line.setData(self.graph_data_x, self.graph_data_y, connect='finite')  # Update the data.
            
        self.counter += 1
        self.idx += 0.1
    
    def on_pushButton_clicked(self):
        self.label.setText('Ligado!')

    def on_pushButton2_clicked(self):
        testlib = cdll.LoadLibrary("./libtest.so")
        mult = testlib.mult
        soma = testlib.sum
        a = soma(int(self.lineEdit2.text()), int(self.lineEdit3.text()))
        b = mult(int(self.lineEdit2.text()), int(self.lineEdit3.text()))
        self.label2.setText(str(a))
        self.label3.setText(str(b))
        
    def on_pushButton3_clicked(self):
        modbuslib = cdll.LoadLibrary("./libmmodbus.so")
        modbus_open = modbuslib.modbusOpen
        modbus_onoff = modbuslib.escreve_onoff
        port = self.lineEdit4.text()
        b_port = port.encode('utf-8')
        modbus_open(b_port)
        modbus_onoff(1)

    def on_pushButton4_clicked(self):
        if self.pushButton4.text() == 'ON':
            print('Ligou!')
            regs = [1, 10]
            status = write_holding_reg(10, 40009, 2, regs)
            if status == 0:
                self.timer1.start(100)
                self.pushButton4.setText('OFF')
        else:
            print('Desligou!')
            regs = [0, 10]
            status = write_holding_reg(10, 40009, 2, regs)
            if status == 0:
                self.graphWidget.clear()
                self.counter = 0
                self.idx = 0.0
                self.overflow = False
                self.timer1.stop()
                self.graph_data_x.clear()
                self.graph_data_y.clear()
                self.data_line = 0
                self.pushButton4.setText('ON')


fruits = ['apple', 'banana', 'cherry']
for x in fruits:
    print(x)

for x in range(10):
    print(x)

for x in range(1,9):
    print(x)

for x in range(0,10,2):
    print(x)

print('while')
x = 0
while(x<10):
    print(x)
    x += 1

print('while com break')
x = 0
while(x<10):
    print(x)
    if x == 5:
        break
    x += 1

print('while com continue')
i = 0
while i < 6:
  i += 1
  if i == 3:
    continue
  print(i)

# Creates bytearray from byte literal 
arr1 = bytearray(b"abcd") 
  
# iterating the value 
for value in arr1: 
    print(value) 
      
# Create a bytearray object 
arr2 = bytearray(b"aaaacccc") 
  
# count bytes from the buffer 
print("Count of c is:", arr2.count(b"c"))

# simple list of integers 
list = [1, 2, 3, 4] 
  
# iterable as source 
array = bytearray(list) 
  
print(array) 
print("Count of bytes:", len(array)) 

# Build array/vector:
x = np.linspace(-np.pi, np.pi, 10)
print(x)

arr = []
arr.insert(0,1)
arr.insert(1,10)
arr.insert(2,100)
print(arr)

print(0o10)
print(0x10)
print(0b10)

type(10)
type(0x10)
type(0b10)
type(.4e7)

ser = serial.Serial()

aucCRCHi = [0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40]

aucCRCLo = [0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7,
    0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E,
    0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09, 0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9,
    0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC,
    0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
    0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32,
    0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D,
    0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A, 0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 
    0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF,
    0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
    0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1,
    0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4,
    0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 
    0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA,
    0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
    0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0,
    0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97,
    0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C, 0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E,
    0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89,
    0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
    0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83,
    0x41, 0x81, 0x80, 0x40]

def usMBCRC16(pucFrame, usLen):
    ucCRCHi = 0xFF
    ucCRCLo = 0xFF
    iIndex = 0
    i = 0

    while(usLen > 0):
        iIndex = ucCRCLo ^ pucFrame[i]
        i += 1
        ucCRCLo = ((ucCRCHi ^ aucCRCHi[iIndex]) & 0xFF)
        ucCRCHi = (aucCRCLo[iIndex] & 0xFF)
        usLen -= 1
    
    return ((ucCRCHi << 8) | ucCRCLo )

def write_holding_reg(slave_address, init_address, num_reg_write, regs):
    funcao = 16
    mensagem = []
    tamanho_pergunta = 7+(2*num_reg_write)
    crc = 0
    init_address -= 1
    mensagem.insert(0,slave_address)
    mensagem.insert(1,funcao)
    mensagem.insert(2,init_address >> 8)
    mensagem.insert(3,init_address & 0xFF)
    mensagem.insert(4,num_reg_write >> 8)
    mensagem.insert(5,num_reg_write & 0xFF)
    mensagem.insert(6,num_reg_write * 2)

    for i in range(0,num_reg_write):
        mensagem.insert(7+(i*2),regs[i] >> 8)
        mensagem.insert(8+(i*2),regs[i] & 0xFF)

    crc=usMBCRC16(mensagem,tamanho_pergunta)
    mensagem.insert(tamanho_pergunta, crc & 0xFF)
    mensagem.insert(tamanho_pergunta+1, crc >> 8)
    print(mensagem)
    if 'ser' in locals():
        print('Porta serial aberta')
    else:
        ser = serial.Serial('/dev/ttyACM0', 115200, timeout=20)  # open serial port
    
    #array_message = bytearray(mensagem)
    #print(array_message)
    ser.write(mensagem)     # write a string
    #values = bytearray([4, 9, 62, 144, 56, 30, 147, 3, 210, 89, 111, 78, 184, 151, 17, 129])
    #ser.write(values)

    resposta = ser.read(8)
    if resposta[1] != funcao:
        return -1
    
    #Teste de CRC
    crc=usMBCRC16(resposta, 6)
    crc_resp=(resposta[7]<<8) | (resposta[6])
    if crc != crc_resp:
        print('Erro crc!')
        return -1
    else:
        print('crc correto!')
        return 0

    #ser.close()             # close port

def read_input_reg(slave_address, init_address, num_reg_read, regs):
    funcao = 4
    mensagem = []
    tamanho_resposta = 5+(2*num_reg_read)
    crc = 0
    init_address -= 1
    mensagem.insert(0,slave_address)
    mensagem.insert(1,funcao)
    mensagem.insert(2,init_address >> 8)
    mensagem.insert(3,init_address & 0xFF)
    mensagem.insert(4,num_reg_read >> 8)
    mensagem.insert(5,num_reg_read & 0xFF)

    crc=usMBCRC16(mensagem,6)
    mensagem.insert(6, crc & 0xFF)
    mensagem.insert(7, crc >> 8)
    #print(mensagem)

    if 'ser' in locals():
        print('Porta serial aberta')
    else:
        ser = serial.Serial('/dev/ttyACM0', 115200, timeout=20)  # open serial port
    
    ser.write(mensagem)     # write a string
    #print(tamanho_resposta)
    resposta = ser.read(tamanho_resposta)
    #print(resposta)
    #print(len(resposta))
    if resposta[1] != funcao:
        return -1
    
    #Teste de CRC
    crc=usMBCRC16(resposta, tamanho_resposta-2)
    crc_resp=(resposta[tamanho_resposta-1]<<8) | (resposta[tamanho_resposta-2])
    if crc != crc_resp:
        print('Erro crc!')
        return -1
    else:
        for i in range ((tamanho_resposta - 5)>>1):
            regs[i] =  (resposta[3+(i*2)] << 8) | (resposta[4+(i*2)] & 0xFF)
        #print('crc correto!')
        return 0

    #ser.close()             # close port

def printit():
    threading.Timer(5.0, printit).start()
    print ("Hello, World!")

printit()

app=QApplication(sys.argv)
widget=MainWindow()
widget.show()
sys.exit(app.exec_())
