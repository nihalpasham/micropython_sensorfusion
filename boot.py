# This file is executed on every boot (including wake-boot from deepsleep)
#import esp
#esp.osdebug(None)
#import webrepl
#webrepl.start()

import network
import imu

def wifi_connect():
    sta_if = network.WLAN(network.STA_IF)                         # Instantiate the WLAN interface and set it to station mode
    if not sta_if.isconnected():                                  # check to see if its connected
        print('connecting to wifi')           
        sta_if.active(True)                                       # if not ..activate interface
        nets = sta_if.scan()                                      # scan for networks
        for net in nets:
            if net[0] == b'iPhone':
                print('===network found===')
                sta_if.connect('iPhone', 'shaila@41')             # connect to a trusted wifi network
                while not sta_if.isconnected():                   # check to see if its connected or keeping looping till you're connected   
                    pass
            elif net[0] == b'TP-Link_09A0':
                print('===network found===')
                sta_if.connect('TP-Link_09A0', 'Shaila@44')       # connect to a trusted wifi network
                while not sta_if.isconnected():                   # check to see if its connected or keeping looping till you're connected   
                    pass
    if not sta_if.isconnected():
        print('no trusted networks found')
    print('network config ....', sta_if.ifconfig())                    # print/log network configuration paramenters

wifi_connect()
imu.read_mpu6050()