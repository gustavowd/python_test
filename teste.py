import sys
from PyQt5.QtCore import pyqtSlot
from PyQt5.QtWidgets import QApplication, QDialog, QMainWindow
from PyQt5.uic import loadUi

from ctypes import cdll, c_long, c_int, c_char_p, create_string_buffer

import serial

class MainWindow(QMainWindow):
    def __init__(self):
        super(MainWindow, self).__init__()
        loadUi('teste.ui', self)
        self.setWindowTitle('Teste')
        self.pushButton.clicked.connect(self.on_pushButton_clicked)
        self.pushButton2.clicked.connect(self.on_pushButton2_clicked)
        self.pushButton3.clicked.connect(self.on_pushButton3_clicked)
    
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
        #ser = serial.Serial(self.lineEdit4.text(), 115200, timeout=20)  # open serial port
        #ser.write(b'hello')     # write a string
        #ser.close()             # close port


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

app=QApplication(sys.argv)
widget=MainWindow()
widget.show()
sys.exit(app.exec_())