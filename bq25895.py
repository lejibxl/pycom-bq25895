from machine import I2C
import ubinascii
VBUS_TYPE =['NONE',
            'SDP',
	        'CDP (1.5A)',
	        'USB_DCP (3.25A)',
	        'MAXC (1.5A)',
	        'UNKNOWN (500mA)',
	        'NONSTAND (1A/2A/2.1A/2.4A',
	        'VBUS_OTG']

CHRG_STAT =['Not Charging',
            'Pre-charge',
            'Fast Charging',
            'Charge Termination Done']

PG_STAT =  ['Not Power Good',
            'Power Good']

SDP_STAT = ['USB100 input is detected',
            'USB500 input is detected']

#i2c = I2C(0)
#i2c.writeto_mem(0x6A,0x00,0b00111111)
#bin(i2c.readfrom_mem(0x6A, 0x0B, 1)[0])

class BQ25895:
    I2CADDR=const(0x6A)
    def __init__(self, sda = 'P9', scl = 'P10'):
        from machine import I2C
        self.i2c = I2C(0, mode=I2C.MASTER, pins=(sda, scl))
        self._setBit(0x02,[None,1,None,None,None,None,None,None])
        #self._setBit(0x00,[None,1,1,1,1,1,1,1])
        #self._setBit(0x04,[1,None,None,None,None,None,None,None])  #enable current pulse control
        #self._setBit(0x04,[None,None,None,None,None,None,1,None])   #enable current pulse control UP & DN
    def _setBit(self, reg, values):
        if len(values) == 8:
            values.reverse()
            regVal = self.i2c.readfrom_mem(I2CADDR , reg, 1)[0]
            regValOld = regVal
            for i,value in enumerate(values):
                if value == 1:
                    mask = 1 << i
                    regVal = (regVal | mask)
                elif value == 0:
                    mask = ~(1 << i)
                    regVal = (regVal & mask)
            if regValOld != regVal:
                self.i2c.writeto_mem(I2CADDR, reg, regVal)
                print("write : {} to {}".format(bin(regVal),reg))

    def readBit(self, reg):
        regVal = self.i2c.readfrom_mem(I2CADDR , reg, 1)[0]
        return regVal

    def reset(self):
        self._setBit(0x14,[1,None,None,None,None,None,None,None])

    def charge_enable(self, enable):
        self._setBit( 0x03,[None,None,None,1 if enable == True else 0,None,None,None])
        pass
    def vbus_type(self):
        ret = self.i2c.readfrom_mem(I2CADDR,0x0B, 1)
        return ret[0] >> 5

    def vbus_type_str(self):
        ret = self.vbus_type()
        return VBUS_TYPE[ret]

    def chrg_stat(self):
        ret = self.i2c.readfrom_mem(I2CADDR,0x0B, 1)
        return (ret[0] & 0b00011000) >> 3

    def chrg_stat_str(self):
        ret = self.chrg_stat()
        return CHRG_STAT[ret]

    def pg_stat(self):
        ret = self.i2c.readfrom_mem(I2CADDR,0x0B, 1)
        return (ret[0] & 0b00000100) >> 2

    def pg_stat_str(self):
        ret = self.pg_stat()
        return PG_STAT[ret]

    def vsys_stat(self):
        ret = self.i2c.readfrom_mem(I2CADDR,0x0B, 1)
        return (ret[0] & 0b00000001)

    def vsys_stat_str(self):
        ret = self.pg_stat()
        return "Regulation" if ret == 1 else "Not Regulation"

    def read_battery_volt(self):
        ret = self.i2c.readfrom_mem(I2CADDR,0x0E, 1)
        volt = 2304 + (int(ret[0] & 0b01111111) * 20)
        return volt

    def read_sys_volt(self):
        ret = self.i2c.readfrom_mem(I2CADDR,0x0F, 1)
        volt = 2304 + (int(ret[0] & 0b01111111) * 20)
        return volt

    def read_TS_per(self):
        ret = self.i2c.readfrom_mem(I2CADDR,0x10, 1)
        volt = 21 + (int(ret[0] & 0b01111111) * 0.465)
        return volt

    def read_stat(self):
        ret1 = self.i2c.readfrom_mem(I2CADDR,0x0B, 1)
        ret2 = self.i2c.readfrom_mem(I2CADDR,0x0C, 1)
        print("0x0B:{:08b}, 0x0C:{:08b}".format(ord(ret1),ord(ret2)))
        return [ret1, ret2]

    def read_vbus_volt(self):
        ret = self.i2c.readfrom_mem(I2CADDR,0x11, 1)
        volt = 2600 + (int(ret[0] & 0b01111111) * 100)
        return volt

    def read_temperature(self): #???????????
        ret = self.i2c.readfrom_mem(I2CADDR,0x10, 1)
        temp = 21 + (int(ret[0] & 0b01111111) * 0.465)
        return temp

    def read_charge_current(self):
        ret = self.i2c.readfrom_mem(I2CADDR,0x12 ,1)
        amp = 0 + (int(ret[0] & 0b01111111) * 50)
        return amp

    def set_charge_current(self,m_A):
        if m_A > 5056 :
            m_A = 5056
        regVal=int(m_A/64)
        _setBit( 0x04,[
            None,
            1 if (regVal & 0b01000000) > 0 else 0,
            1 if (regVal & 0b00100000) > 0 else 0,
            1 if (regVal & 0b00010000) > 0 else 0,
            1 if (regVal & 0b00001000) > 0 else 0,
            1 if (regVal & 0b00000100) > 0 else 0,
            1 if (regVal & 0b00000010) > 0 else 0,
            1 if (regVal & 0b00000001) > 0 else 0]
            )

    def read_input_current_max(self):
        ret = self.i2c.readfrom_mem(I2CADDR,0x00 ,1)
        amp =  (100 + int(ret[0] & 0b00111111) * 50)
        return amp

    def set_input_current_max(self,m_A):
        if m_A > 3250 :
            m_A = 3250
        elif m_A < 100 :
            m_A = 100
        regVal=int((m_A - 100)/50)
        self._setBit( 0x00,[
            None,
            None,
            1 if (regVal & 0b00100000) > 0 else 0,
            1 if (regVal & 0b00010000) > 0 else 0,
            1 if (regVal & 0b00001000) > 0 else 0,
            1 if (regVal & 0b00000100) > 0 else 0,
            1 if (regVal & 0b00000010) > 0 else 0,
            1 if (regVal & 0b00000001) > 0 else 0]
            )
if __name__ == '__main__':
    from machine import I2C
    import time
    from machine import ADC
    bq25895=BQ25895()
    bq25895.reset()
    time.sleep_ms(550)
    bq25895._setBit(0x14,[1,None,None,None,None,None,None,None])

    adc = ADC(bits=12)
    adc_ILIM = adc.channel(attn=ADC.ATTN_0DB,pin='P20')

    while True:
        #bq25895.set_input_current_max(3250)
        #bq25895._setBit(0x07,[None,None,0,0,None,None,None,None])
        print("status: {}, TS: {}".format(bq25895.read_stat(),bq25895.read_TS_per()))
        print("IN Stat: {}, vbus:{}.{}, in Imax:{}, in U:{}".format(bq25895.pg_stat_str(),bq25895.vbus_type(),bq25895.vbus_type_str(), bq25895.read_input_current_max(),bq25895.read_vbus_volt()))
        print("Bat stat:{}, I:{}, V:{}".format(bq25895.chrg_stat_str(), bq25895.read_charge_current(), bq25895.read_battery_volt()))
        print("SYS status:{}, V:{}".format(bq25895.vsys_stat_str(), bq25895.read_sys_volt()))
        print("PMID Boost status" )

        VILIM = adc_ILIM.voltage()
        RILIM = 768
        KILIM = 365
        PWIN_I = (KILIM * VILIM) / (RILIM * 0.8)
        print("RPY_I:{}, CRG_I:{} ".format(PWIN_I,bq25895.read_charge_current()))


        """
            print("##")
            print("IN Stat: {}, vbus:{:03b}.{}, in Imax:{}, in U:{}".format(bq25895.pg_stat_str(),bq25895.vbus_type(),bq25895.vbus_type_str(), bq25895.read_input_current_max(),bq25895.read_vbus_volt()))
            i=0
            while i <= 0x14:
                print("{:02x}:{:08b}".format(i,bq25895.readBit(i)), end=' ')
                i = i + 1
            print("---")
        """
        time.sleep(3)