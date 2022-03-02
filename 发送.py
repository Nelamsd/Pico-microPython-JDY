import machine
from machine import Pin, PWM
import utime
import _thread

key = Pin(0, Pin.IN, Pin.PULL_UP)
uart = machine.UART(0, baudrate=115200, tx=machine.Pin(16), rx=machine.Pin(17))
ledFlag = True

ATNAME = "AT+NAME123\r\n"
ATID = "AT+NETID123456789ABC\r\n"
all_io_L = [0xAA,0xfc,0xff,0xff,0xe7,0xf0,0x00]         #此指令功能为将所有蓝牙模块核心内OUT引脚设置为高电平
all_io_H = [0xAA,0xfc,0xff,0xff,0xe7,0xff,0xff]         #此指令功能为将所有蓝牙模块核心内OUT引脚设置为低电平
#MESH_UART_data = [0xAA,0xFB,0xFF,0xFF,0x31,0x32,0x33]  #广播发送
MESH_UART_data = [0xAA,0xFB,0x00,0x5E,0x31,0x32,0x33]   #向Mac地址0x005E发送0x31,0x32,0x33(默认数据)

#接收数据函数
def uart_reader_thread():
    while True:
        if uart.any():
            cmd = uart.readline()
            print(cmd)
            #if cmd == "b'on'":
            #    machine.Pin(25, machine.Pin.OUT).value(1)
            #if cmd == "b'off'":
            #    machine.Pin(25, machine.Pin.OUT).value(0)
        utime.sleep(0.1)


#摇杆X,Y变化检测函数
def rockerChange(urY,urX):
    intY = (urY.read_u16()*2)-65535               #读摇杆上的X,Y的数据，实际上应该是urY.read_u16()
    intX = (urX.read_u16()*2)-65535               #乘2再减65536是需要把数据分成正负好进行一条轴可以控制2个灯
    if abs(intY) >=5000 or abs(intX) >=5000:      #若x和y值的绝对值脱离定义的范围（5000），则视为摇杆使用
        controlCommand(intY,intX)                 #调用写无线命令控制LED函数
    else:
        MESH_UART_data[-1] = eval(hex(0))         #若执行else则视为摇杆未使用，写入命令为‘0’
        MESH_UART_data[-2] = eval(hex(0))         #因为摇杆模块在不使用时，读取的值在一个浮动范围内
        MESH_UART_data[-3] = eval(hex(0))         #此操作主要为消除误差
        sendMessage()

#读取摇杆状态发送命令
#判断读到的值为正还是负，然后将读到的值写入到写命令字符串中
def controlCommand(intY,intX):#0上，1下，2左，3右
    if intY < 0 and intX < 0:
        #print("X:左，Y:上")
        MESH_UART_data[-3] = eval(hex(20))
        MESH_UART_data[-2] = eval(hex(int(abs(intY)/1000)))
        MESH_UART_data[-1] = eval(hex(int(abs(intX)/1000)))
    elif intY >= 0 and intX < 0:
        #print("X:左，Y:下")
        MESH_UART_data[-3] = eval(hex(21))
        MESH_UART_data[-2] = eval(hex(int(abs(intY)/1000)))
        MESH_UART_data[-1] = eval(hex(int(abs(intX)/1000)))
    elif intY < 0 and intX >= 0:
        #print("X:右，Y:上")
        MESH_UART_data[-3] = eval(hex(30))
        MESH_UART_data[-2] = eval(hex(int(abs(intY)/1000)))
        MESH_UART_data[-1] = eval(hex(int(abs(intX)/1000)))
    elif intY >= 0 and intX >= 0:
        #print("X:右，Y:下")
        MESH_UART_data[-3] = eval(hex(31))
        MESH_UART_data[-2] = eval(hex(int(abs(intY)/1000)))
        MESH_UART_data[-1] = eval(hex(int(abs(intX)/1000)))
    sendMessage()

#按键中断
def external_interrupt(key):
    global ledFlag
    utime.sleep_ms(500)                         # 消除抖动                              
    if key.value() == 0:                        # 再次判断按键是否被按下
        print('The button is pressed')
        if ledFlag == True:
            ledFlag = False
            print(ledFlag)
            MESH_UART_data[-3] = eval(hex(90))
        elif ledFlag == False:
            ledFlag = True
            print(ledFlag)
            MESH_UART_data[-3] = eval(hex(91))
        sendMessage()

#发送数据函数
def sendMessage():
    uart.write(bytes(MESH_UART_data))            #将包含目标地址，数据的数组通过串口发送给蓝牙，由蓝牙发送
    utime.sleep(0.2)
    
#初始化ADC
def initADC():
    urY = machine.ADC(0)                         #摇杆Y轴使用ADC0引脚
    urX = machine.ADC(1)                         #摇杆X轴使用ADC1引脚
    return urY,urX

#配置串口向蓝牙模块写命令
def initUART():
    print(uart)
    _thread.start_new_thread(uart_reader_thread, ())
    machine.Pin(25, machine.Pin.OUT).value(0)    #配置开始前板载LED灭
    uart.write(ATID)                             #使用AT命令写入组网ID
    utime.sleep(1)
    uart.write(ATNAME)                           #使用AT命令写入组网名
    utime.sleep(1)
    #uart.write("T+MADDR11\r\n")
    #utime.sleep(1)
    uart.write("AT+RESET\r\n")                   #AT命令重启蓝牙模块
    utime.sleep(1)
    uart.write(bytes(all_io_H))                  #拉高电平
    uart.write(bytes(all_io_L))                  #拉低蓝牙模块5个端口的电平
    machine.Pin(25, machine.Pin.OUT).value(1)    #配置完成点亮板载LED

#主程序入口
if __name__ == '__main__':
    initUART()
    key.irq(external_interrupt, Pin.IRQ_FALLING) #按键中断
    urY,urX = initADC()                          #初始化摇杆使用模数转换引脚
    while True:
        rockerChange(urY,urX)                    #调用检测摇杆变化函数
        utime.sleep(0.02)