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

import modbus as mb
import numpy as np

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
            mb.read_input_reg(10, 50000, 1, regs)
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
            mb.read_input_reg(10, 50000, 1, regs)
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
        #file = open('abc.txt', 'a+') #abre arquivo com append
        file = open('abc.txt', 'w+')
        file.write('Teste de escrita\n')
        #file.seek(0, 0)    #volta ao inicio
        #file.readline()    #le a linha
        file.close()

        with open('abcd.txt', 'w+') as file:
            file.write('linha 1\n')
            file.write('linha 2\n')

        try:
            file = open('leitura.txt', 'r')
            self.lineEdit.setText(file.readline())
        finally:
            file.close()

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
            status = mb.write_holding_reg(10, 40009, 2, regs)
            if status == 0:
                self.timer1.start(100)
                self.pushButton4.setText('OFF')
        else:
            print('Desligou!')
            regs = [0, 10]
            status = mb.write_holding_reg(10, 40009, 2, regs)
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

def printit():
    threading.Timer(5.0, printit).start()
    print ("Hello, World!")

printit()

app=QApplication(sys.argv)
widget=MainWindow()
widget.show()
sys.exit(app.exec_())
