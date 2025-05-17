# *****************************************************************************
# * | File        :      Pico_ePaper-2.9.py
# * | Author      :   Waveshare team
# * | Function    :   Electronic paper driver
# * | Info        :
# *----------------
# * | This version:   V1.0
# * | Date        :   2021-03-16
# # | Info        :   python demo
# -----------------------------------------------------------------------------
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documnetation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to  whom the Software is
# furished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS OR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
#

from machine import Pin, SPI, WDT
import framebuf
import utime

# Display resolution
EPD_WIDTH       = 128
EPD_HEIGHT      = 296

watchdog: WDT

class EPD_2in9_B_V4_Portrait():

    def __init__(self, bus: int, sda: int, sck: int, cs: int, dc: int, rst: int, busy: int, wdt: WDT) -> None:
        self.reset_pin = Pin(rst, Pin.OUT)
        
        self.busy_pin = Pin(busy, Pin.IN, Pin.PULL_UP)
        self.cs_pin = Pin(cs, Pin.OUT)
        if EPD_WIDTH % 8 == 0:
            self.width = EPD_WIDTH
        else :
            self.width = (EPD_WIDTH // 8) * 8 + 8
        self.height = EPD_HEIGHT
        
        self.spi = SPI(bus,baudrate=4000_000,sck=Pin(sck),mosi=Pin(sda))
        self.dc_pin = Pin(dc, Pin.OUT)
        
        
        self.buffer_balck = bytearray(self.height * self.width // 8)
        self.buffer_red = bytearray(self.height * self.width // 8)
        self.imageblack = framebuf.FrameBuffer(self.buffer_balck, self.width, self.height, framebuf.MONO_HLSB)
        self.imagered = framebuf.FrameBuffer(self.buffer_red, self.width, self.height, framebuf.MONO_HLSB)
        self.watchdog: WDT = wdt 
        self.init()

    def digital_write(self, pin, value):
        pin.value(value)

    def digital_read(self, pin):
        return pin.value()

    def delay_ms(self, delaytime):
        utime.sleep(delaytime / 1000.0)

    def spi_writebyte(self, data):
        self.spi.write(bytearray(data))

    def module_exit(self):
        self.digital_write(self.reset_pin, 0)

    # Hardware reset
    def reset(self):
        self.digital_write(self.reset_pin, 1)
        self.delay_ms(50)
        self.digital_write(self.reset_pin, 0)
        self.delay_ms(2)
        self.digital_write(self.reset_pin, 1)
        self.delay_ms(50)


    def send_command(self, command):
        self.digital_write(self.dc_pin, 0)
        self.digital_write(self.cs_pin, 0)
        self.spi_writebyte([command])
        self.digital_write(self.cs_pin, 1)

    def send_data(self, data):
        self.digital_write(self.dc_pin, 1)
        self.digital_write(self.cs_pin, 0)
        self.spi_writebyte([data])
        self.digital_write(self.cs_pin, 1)
        
    def send_data1(self, buf):
        self.digital_write(self.dc_pin, 1)
        self.digital_write(self.cs_pin, 0)
        self.spi.write(bytearray(buf))
        self.digital_write(self.cs_pin, 1)
        
    def ReadBusy(self):
        print('busy')
        while(self.digital_read(self.busy_pin) == 1): 
            self.watchdog.feed()
            self.delay_ms(10) 
        print('busy release')
        self.delay_ms(20)
        
    def TurnOnDisplay(self):
        self.send_command(0x22) #Display Update Control
        self.send_data(0xF7)
        self.send_command(0x20) #Activate Display Update Sequence
        self.ReadBusy()

    def TurnOnDisplay_Base(self):
        self.send_command(0x22) #Display Update Control
        self.send_data(0xF4)
        self.send_command(0x20) #Activate Display Update Sequence
        self.ReadBusy()
        
    def TurnOnDisplay_Fast(self):
        self.send_command(0x22) #Display Update Control
        self.send_data(0xC7)
        self.send_command(0x20) #Activate Display Update Sequence
        self.ReadBusy()
        
    def TurnOnDisplay_Partial(self):
        self.send_command(0x22) #Display Update Control
        self.send_data(0x1C)
        self.send_command(0x20) #Activate Display Update Sequence
        self.ReadBusy()


    def init(self):
        # EPD hardware init start
        self.reset()

        self.ReadBusy()   
        self.send_command(0x12)  #SWRESET
        self.ReadBusy()   

        self.send_command(0x01) #Driver output control      
        self.send_data((self.height-1)%256)    
        self.send_data((self.height-1)//256)
        self.send_data(0x00)

        self.send_command(0x11) #data entry mode       
        self.send_data(0x03)

        self.send_command(0x44) #set Ram-X address start/end position   
        self.send_data(0x00)
        self.send_data(self.width//8-1)   

        self.send_command(0x45) #set Ram-Y address start/end position          
        self.send_data(0x00)
        self.send_data(0x00) 
        self.send_data((self.height-1)%256)    
        self.send_data((self.height-1)//256)

        self.send_command(0x3C) #BorderWavefrom
        self.send_data(0x05)	

        self.send_command(0x21) #  Display update control
        self.send_data(0x00)		
        self.send_data(0x80)	

        self.send_command(0x18) #Read built-in temperature sensor
        self.send_data(0x80)	

        self.send_command(0x4E)   # set RAM x address count to 0
        self.send_data(0x00)
        self.send_command(0x4F)   # set RAM y address count to 0X199    
        self.send_data(0x00)    
        self.send_data(0x00)
        self.ReadBusy()
        
        return 0
    
    def init_Fast(self):
        # EPD hardware init start
        self.reset()

        self.ReadBusy()   
        self.send_command(0x12)  #SWRESET
        self.ReadBusy()   	

        self.send_command(0x18) #Read built-in temperature sensor
        self.send_data(0x80)

        self.send_command(0x22) # Load temperature value
        self.send_data(0xB1)		
        self.send_command(0x20)	
        self.ReadBusy()   

        self.send_command(0x1A) # Write to temperature register
        self.send_data(0x5a)		# 90		
        self.send_data(0x00)	
                    
        self.send_command(0x22) # Load temperature value
        self.send_data(0x91)		
        self.send_command(0x20)	
        self.ReadBusy()  

        self.send_command(0x01) #Driver output control      
        self.send_data((self.height-1)%256)    
        self.send_data((self.height-1)//256)
        self.send_data(0x00)

        self.send_command(0x11) #data entry mode       
        self.send_data(0x03)

        self.send_command(0x44) #set Ram-X address start/end position   
        self.send_data(0x00)
        self.send_data(self.width//8-1)   

        self.send_command(0x45) #set Ram-Y address start/end position          
        self.send_data(0x00)
        self.send_data(0x00) 
        self.send_data((self.height-1)%256)    
        self.send_data((self.height-1)//256)	

        self.send_command(0x4E)   # set RAM x address count to 0
        self.send_data(0x00)
        self.send_command(0x4F)   # set RAM y address count to 0X199    
        self.send_data(0x00)    
        self.send_data(0x00)
        self.ReadBusy()	
        
        return 0
    
    def display(self): # ryimage: red or yellow image
        self.send_command(0x24)
        self.send_data1(self.buffer_balck)

        self.send_command(0x26)
        self.send_data1(self.buffer_red)

        self.TurnOnDisplay()

    def display_Fast(self): # ryimage: red or yellow image
        self.send_command(0x24)
        self.send_data1(self.buffer_balck)

        self.send_command(0x26)
        self.send_data1(self.buffer_red)

        self.TurnOnDisplay_Fast()

    def Clear(self):
        self.send_command(0x24)
        self.send_data1([0xFF] * self.height * int(self.width / 8))
        
        self.send_command(0x26)
        self.send_data1([0x00] * self.height * int(self.width / 8))
                                
        self.TurnOnDisplay()

    def display_Partial(self, Image, Xstart, Ystart, Xend, Yend):
        if((Xstart % 8 + Xend % 8 == 8 & Xstart % 8 > Xend % 8) | Xstart % 8 + Xend % 8 == 0 | (Xend - Xstart)%8 == 0):
            Xstart = Xstart // 8
            Xend = Xend // 8
        else:
            Xstart = Xstart // 8 
            if Xend % 8 == 0:
                Xend = Xend // 8
            else:
                Xend = Xend // 8 + 1
                
        if(self.width % 8 == 0):
            Width = self.width // 8
        else:
            Width = self.width // 8 +1
        Height = self.height

        Xend -= 1
        Yend -= 1
	
        self.send_command(0x44)       # set RAM x address start/end, in page 35
        self.send_data(Xstart & 0xff)    # RAM x address start at 00h
        self.send_data(Xend & 0xff)    # RAM x address end at 0fh(15+1)*8->128 
        self.send_command(0x45)       # set RAM y address start/end, in page 35
        self.send_data(Ystart & 0xff)    # RAM y address start at 0127h
        self.send_data((Ystart>>8) & 0x01)    # RAM y address start at 0127h
        self.send_data(Yend & 0xff)    # RAM y address end at 00h
        self.send_data((Yend>>8) & 0x01)   

        self.send_command(0x4E)   # set RAM x address count to 0
        self.send_data(Xstart & 0xff)
        self.send_command(0x4F)   # set RAM y address count to 0X127    
        self.send_data(Ystart & 0xff)
        self.send_data((Ystart>>8) & 0x01)

        self.send_command(0x24)   #Write Black and White image to RAM
        for j in range(Height):
            for i in range(Width):
                if((j > Ystart-1) & (j < (Yend + 1)) & (i > Xstart-1) & (i < (Xend + 1))):
                    self.send_data(Image[i + j * Width])
        self.TurnOnDisplay_Partial()

    def sleep(self):
        self.watchdog.feed()
        self.send_command(0x10) 
        self.send_data(0x01)
        
        self.delay_ms(2000)
        self.module_exit()

