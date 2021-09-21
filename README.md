# scitonBoson

Using /dev/ttyACM0 to send commands and receive data from Boson

To get Serial Number

Frame to send (17 bytes)
8E 00 00 00 00 01 00 05 00 02 FF FF FF FF 01 0B AE

Frame received (21 bytes)
8E 00 00 00 00 01 00 05 00 02 00 00 00 00 00 01 6D 94 30 10 AE

Get the current focal plane array (FPA) temperature in Celsius.

        Returns
        -------
            float
                FPA temperature in Celsius

>>> Frame to send (17 bytes)                                                    
>>> 8E 00 00 00 00 01 00 05 00 30 FF FF FF FF 49 66 AE        
                 
>>> Frame received (19 bytes)                                                   
>>> 8E 00 00 00 00 01 00 05 00 30 00 00 00 00 01 7C BD CE AE                    


Walked into a note on my desk that you requested the HEX commands to enable the radiometry readings, I assume over USB.

 

For the USB out to function properly you will need to send it sysctrlSetUsbVideoIR16Mode() ID: 0x000E000D

 

8E 00 00 00 00 C2 00 0E 00 0D FF FF FF FF 00 00 00 02 C4 F2 AE – Enable Tlinear Output

>>>   ( to change to black-hot : scitonBoson -p/dev/ttyAMC0 -b921600 c000B003 x0 x0 x0 x1 v a )

>>>   ( to change to white-hot : scitonBoson -p/dev/ttyAMC0 -b921600 c000B0003 x0 x0 x0 x0 v a )

