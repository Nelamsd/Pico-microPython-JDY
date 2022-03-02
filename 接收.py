import machine
from machine import Pin, PWM
import utime
import _thread

uart = machine.UART(0, baudrate=115200, tx=machine.Pin(16), rx=machine.Pin(17))

ATNAME = "AT+NAME123\r\n"
ATID = "AT+NETID123456789ABC\r\n"
all_io_L = [0xAA,0xfc,0xff,0xff,0xe7,0xf0,0x00]     #此指令功能为将所有蓝牙模块核心内OUT引脚设置为高电平
all_io_H = [0xAA,0xfc,0xff,0xff,0xe7,0xff,0xff]     #此指令功能为将所有蓝牙模块核心内OUT引脚设置为低电平
UARTFLAG = False                                    #判断蓝牙模块是否初始化完成标志位
LEDFLAG = False                                     #判断LED模式标志位 FALSE为摇杆按下全亮，TRUE为使用摇杆模式

def uart_reader_thread():
    global LEDFLAG
    while True:
        if uart.any():
            cmd = uart.readline()
            print(cmd)
            if len(cmd) > 3 and UARTFLAG == True:   #判断接收到收到数据的长度以及判断蓝牙模块是否初始化完成
                #print(cmd[-3])
                if cmd[-3] == 90:                   #如果接收到的数组下标为[-3]的值为90则LED全灭
                    allLed(0)
                    LEDFLAG = True
                elif cmd[-3] == 91:                 #如果接收到的数组下标为[-3]的值为91则LED全亮
                    allLed(1)
                    LEDFLAG = False
                if LEDFLAG == True and cmd[-3] != 90 and cmd[-3] != 91:
                    if cmd[-3] == 0:                #如果接收到的数组下标为[-3]的值为0则LED全灭（视作摇杆未操作）
                        allLed(0)
                    else:
                        directionLed(0,2,0,0)       
                        directionLed(1,3,0,0)
                        yLed = cmd[-3]%10
                        xLed = int(cmd[-3]/10)
                        yPWM = cmd[-2] * 1000
                        xPWM = cmd[-1] * 1000
                        directionLed(xLed,yLed,xPWM,yPWM)
        utime.sleep(0.1)

#四个LED选择
def directionLed(L1,L2,iX,iY):  #参数L1,L2为X轴和Y轴上的任意2个灯，iX,iY为写入的占空比
    led1 = PWM(Pin(L1))
    led1.freq(1000)
    led2 = PWM(Pin(L2))
    led2.freq(1000)
    led1.duty_u16(iX)
    led2.duty_u16(iY)

#按键功能 按下全亮，再按全灭
def allLed(ledFlag):
    initLED()
    if ledFlag == 1:             #判断标志位
        for i in range (0,4):    #从0-4循环，全部设置成高电平
            Pin.on(Pin(i))
    elif ledFlag == 0:
        for i in range (0,4):    #拉低电平
            Pin.off(Pin(i))

#配置串口向蓝牙模块写命令
def initUART():
    global UARTFLAG
    print(uart)
    _thread.start_new_thread(uart_reader_thread, ())  #启用一条专门用于读数据的线程
    machine.Pin(25, machine.Pin.OUT).value(0)    #配置开始前板载LED灭
    uart.write(ATID)                             #使用AT命令写入组网ID
    utime.sleep(1)
    uart.write(ATNAME)                           #使用AT命令写入组网名
    utime.sleep(1)
    #uart.write("T+MADDR22\r\n")
    #utime.sleep(1)
    uart.write("AT+RESET\r\n")                   #AT命令重启蓝牙模块
    utime.sleep(1)
    uart.write(bytes(all_io_H))                  #拉高电平
    uart.write(bytes(all_io_L))                  #拉低蓝牙模块5个端口的电平
    machine.Pin(25, machine.Pin.OUT).value(1)    #配置完成点亮板载LED
    UARTFLAG = True

#LED使用IO口初始化
def initLED():
    for i in range(0,4):             #循环
        Pin(i, Pin.OUT)              #设置GPIO口为输出
        Pin.off(Pin(i))

#主程序入口
if __name__ == '__main__':
    initUART()                       #初始化串口
    initLED()                        #初始化LED