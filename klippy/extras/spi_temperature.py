# Support for common SPI based thermocouple and RTD temperature sensors
#
# Copyright (C) 2018  Petri Honkala <cruwaller@gmail.com>
# Copyright (C) 2018  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import math
from . import bus
from bisect import bisect


######################################################################
# SensorBase
######################################################################

REPORT_TIME = 0.300

class SensorBase:
    def __init__(self, config, chip_type, config_cmd=None, spi_mode=1):
        self.printer = config.get_printer()
        self.chip_type = chip_type
        self._callback = None
        self.min_sample_value = self.max_sample_value = 0
        self._report_clock = 0
        self.spi = bus.MCU_SPI_from_config(
            config, spi_mode, pin_option="sensor_pin", default_speed=4000000)
        if config_cmd is not None:
            self.spi.spi_send(config_cmd)
        self.mcu = mcu = self.spi.get_mcu()
        # Reader chip configuration
        self.oid = oid = mcu.create_oid()
        mcu.register_response(self._handle_spi_response,
                              "thermocouple_result", oid)
        mcu.register_config_callback(self._build_config)
    def setup_minmax(self, min_temp, max_temp):
        adc_range = [self.calc_adc(min_temp), self.calc_adc(max_temp)]
        self.min_sample_value = min(adc_range)
        self.max_sample_value = max(adc_range)
    def setup_callback(self, cb):
        self._callback = cb
    def get_report_time_delta(self):
        return REPORT_TIME
    def _build_config(self):
        self.mcu.add_config_cmd(
            "config_thermocouple oid=%u spi_oid=%u thermocouple_type=%s" % (
                self.oid, self.spi.get_oid(), self.chip_type))
        clock = self.mcu.get_query_slot(self.oid)
        self._report_clock = self.mcu.seconds_to_clock(REPORT_TIME)
        self.mcu.add_config_cmd(
            "query_thermocouple oid=%u clock=%u rest_ticks=%u"
            " min_value=%u max_value=%u" % (
                self.oid, clock, self._report_clock,
                self.min_sample_value, self.max_sample_value), is_init=True)
    def _handle_spi_response(self, params):
        temp = self.calc_temp(params['value'], params['fault'])
        next_clock      = self.mcu.clock32_to_clock64(params['next_clock'])
        last_read_clock = next_clock - self._report_clock
        last_read_time  = self.mcu.clock_to_print_time(last_read_clock)
        self._callback(last_read_time, temp)
    def fault(self, msg):
        self.printer.invoke_async_shutdown(msg)


######################################################################
# MAX31856 thermocouple
######################################################################

MAX31856_CR0_REG           = 0x00
MAX31856_CR0_AUTOCONVERT   = 0x80
MAX31856_CR0_1SHOT         = 0x40
MAX31856_CR0_OCFAULT1      = 0x20
MAX31856_CR0_OCFAULT0      = 0x10
MAX31856_CR0_CJ            = 0x08
MAX31856_CR0_FAULT         = 0x04
MAX31856_CR0_FAULTCLR      = 0x02
MAX31856_CR0_FILT50HZ      = 0x01
MAX31856_CR0_FILT60HZ      = 0x00

MAX31856_CR1_REG           = 0x01
MAX31856_CR1_AVGSEL1       = 0x00
MAX31856_CR1_AVGSEL2       = 0x10
MAX31856_CR1_AVGSEL4       = 0x20
MAX31856_CR1_AVGSEL8       = 0x30
MAX31856_CR1_AVGSEL16      = 0x70

MAX31856_MASK_REG                          = 0x02
MAX31856_MASK_COLD_JUNCTION_HIGH_FAULT     = 0x20
MAX31856_MASK_COLD_JUNCTION_LOW_FAULT      = 0x10
MAX31856_MASK_THERMOCOUPLE_HIGH_FAULT      = 0x08
MAX31856_MASK_THERMOCOUPLE_LOW_FAULT       = 0x04
MAX31856_MASK_VOLTAGE_UNDER_OVER_FAULT     = 0x02
MAX31856_MASK_THERMOCOUPLE_OPEN_FAULT      = 0x01

MAX31856_CJHF_REG          = 0x03
MAX31856_CJLF_REG          = 0x04
MAX31856_LTHFTH_REG        = 0x05
MAX31856_LTHFTL_REG        = 0x06
MAX31856_LTLFTH_REG        = 0x07
MAX31856_LTLFTL_REG        = 0x08
MAX31856_CJTO_REG          = 0x09
MAX31856_CJTH_REG          = 0x0A
MAX31856_CJTL_REG          = 0x0B
MAX31856_LTCBH_REG         = 0x0C
MAX31856_LTCBM_REG         = 0x0D
MAX31856_LTCBL_REG         = 0x0E

MAX31856_SR_REG            = 0x0F
MAX31856_FAULT_CJRANGE     = 0x80  # Cold Junction out of range
MAX31856_FAULT_TCRANGE     = 0x40  # Thermocouple out of range
MAX31856_FAULT_CJHIGH      = 0x20  # Cold Junction High
MAX31856_FAULT_CJLOW       = 0x10  # Cold Junction Low
MAX31856_FAULT_TCHIGH      = 0x08  # Thermocouple Low
MAX31856_FAULT_TCLOW       = 0x04  # Thermocouple Low
MAX31856_FAULT_OVUV        = 0x02  # Under Over Voltage
MAX31856_FAULT_OPEN        = 0x01

MAX31856_SCALE = 5
MAX31856_MULT = 0.0078125

class MAX31856(SensorBase):
    def __init__(self, config):
        SensorBase.__init__(self, config, "MAX31856",
                            self.build_spi_init(config))
    def calc_temp(self, adc, fault):
        if fault & MAX31856_FAULT_CJRANGE:
            self.fault("Max31856: Cold Junction Range Fault")
        if fault & MAX31856_FAULT_TCRANGE:
            self.fault("Max31856: Thermocouple Range Fault")
        if fault & MAX31856_FAULT_CJHIGH:
            self.fault("Max31856: Cold Junction High Fault")
        if fault & MAX31856_FAULT_CJLOW:
            self.fault("Max31856: Cold Junction Low Fault")
        if fault & MAX31856_FAULT_TCHIGH:
            self.fault("Max31856: Thermocouple High Fault")
        if fault & MAX31856_FAULT_TCLOW:
            self.fault("Max31856: Thermocouple Low Fault")
        if fault & MAX31856_FAULT_OVUV:
            self.fault("Max31856: Over/Under Voltage Fault")
        if fault & MAX31856_FAULT_OPEN:
            self.fault("Max31856: Thermocouple Open Fault")
        adc = adc >> MAX31856_SCALE
        # Fix sign bit:
        if adc & 0x40000:
            adc = ((adc & 0x3FFFF) + 1) * -1
        temp = MAX31856_MULT * adc
        return temp
    def calc_adc(self, temp):
        adc = int( ( temp / MAX31856_MULT ) + 0.5 ) # convert to ADC value
        adc = max(0, min(0x3FFFF, adc)) << MAX31856_SCALE
        return adc
    def build_spi_init(self, config):
        cmds = []
        value = MAX31856_CR0_AUTOCONVERT
        if config.getboolean('tc_use_50Hz_filter', False):
            value |= MAX31856_CR0_FILT50HZ
        cmds.append(0x80 + MAX31856_CR0_REG)
        cmds.append(value)

        types = {
            "B" : 0b0000,
            "E" : 0b0001,
            "J" : 0b0010,
            "K" : 0b0011,
            "N" : 0b0100,
            "R" : 0b0101,
            "S" : 0b0110,
            "T" : 0b0111,
        }
        value = config.getchoice('tc_type', types, default="K")
        averages = {
            1  : MAX31856_CR1_AVGSEL1,
            2  : MAX31856_CR1_AVGSEL2,
            4  : MAX31856_CR1_AVGSEL4,
            8  : MAX31856_CR1_AVGSEL8,
            16 : MAX31856_CR1_AVGSEL16
        }
        value |= config.getchoice('tc_averaging_count', averages, 1)
        cmds.append(value)

        value = (MAX31856_MASK_VOLTAGE_UNDER_OVER_FAULT |
                 MAX31856_MASK_THERMOCOUPLE_OPEN_FAULT)
        cmds.append(value)
        return cmds


######################################################################
# MAX31855 thermocouple
######################################################################

MAX31855_SCALE = 18
MAX31855_MULT = 0.25

class MAX31855(SensorBase):
    def __init__(self, config):
        SensorBase.__init__(self, config, "MAX31855", spi_mode=0)
    def calc_temp(self, adc, fault):
        if adc & 0x1:
            self.fault("MAX31855 : Open Circuit")
        if adc & 0x2:
            self.fault("MAX31855 : Short to GND")
        if adc & 0x4:
            self.fault("MAX31855 : Short to Vcc")
        adc = adc >> MAX31855_SCALE
        # Fix sign bit:
        if adc & 0x2000:
            adc = ((adc & 0x1FFF) + 1) * -1
        temp = MAX31855_MULT * adc
        return temp
    def calc_adc(self, temp):
        adc = int( ( temp / MAX31855_MULT ) + 0.5 ) # convert to ADC value
        adc = max(0, min(0x1FFF, adc)) << MAX31855_SCALE
        return adc


######################################################################
# MAX6675 thermocouple
######################################################################

MAX6675_SCALE = 3
MAX6675_MULT = 0.25

class MAX6675(SensorBase):
    def __init__(self, config):
        SensorBase.__init__(self, config, "MAX6675", spi_mode=0)
    def calc_temp(self, adc, fault):
        if adc & 0x02:
            self.fault("Max6675 : Device ID error")
        if adc & 0x04:
            self.fault("Max6675 : Thermocouple Open Fault")
        adc = adc >> MAX6675_SCALE
        # Fix sign bit:
        if adc & 0x2000:
            adc = ((adc & 0x1FFF) + 1) * -1
        temp = MAX6675_MULT * adc
        return temp
    def calc_adc(self, temp):
        adc = int( ( temp / MAX6675_MULT ) + 0.5 ) # convert to ADC value
        adc = max(0, min(0x1FFF, adc)) << MAX6675_SCALE
        return adc


######################################################################
# MAX31865 (RTD sensor)
######################################################################

MAX31865_CONFIG_REG            = 0x00
MAX31865_RTDMSB_REG            = 0x01
MAX31865_RTDLSB_REG            = 0x02
MAX31865_HFAULTMSB_REG         = 0x03
MAX31865_HFAULTLSB_REG         = 0x04
MAX31865_LFAULTMSB_REG         = 0x05
MAX31865_LFAULTLSB_REG         = 0x06
MAX31865_FAULTSTAT_REG         = 0x07

MAX31865_CONFIG_BIAS           = 0x80
MAX31865_CONFIG_MODEAUTO       = 0x40
MAX31865_CONFIG_1SHOT          = 0x20
MAX31865_CONFIG_3WIRE          = 0x10
MAX31865_CONFIG_FAULTCLEAR     = 0x02
MAX31865_CONFIG_FILT50HZ       = 0x01

MAX31865_FAULT_HIGHTHRESH      = 0x80
MAX31865_FAULT_LOWTHRESH       = 0x40
MAX31865_FAULT_REFINLOW        = 0x20
MAX31865_FAULT_REFINHIGH       = 0x10
MAX31865_FAULT_RTDINLOW        = 0x08
MAX31865_FAULT_OVUV            = 0x04

MAX31865_ADC_MAX = 1<<15

# Callendar-Van Dusen constants for platinum resistance thermometers (RTD)
CVD_A = 3.9083e-3
CVD_B = -5.775e-7

class MAX31865(SensorBase):
    def __init__(self, config):
        rtd_nominal_r = config.getfloat('rtd_nominal_r', 100., above=0.)
        rtd_reference_r = config.getfloat('rtd_reference_r', 430., above=0.)
        adc_to_resist = rtd_reference_r / float(MAX31865_ADC_MAX)
        self.adc_to_resist_div_nominal = adc_to_resist / rtd_nominal_r
        SensorBase.__init__(self, config, "MAX31865",
                            self.build_spi_init(config))
    def calc_temp(self, adc, fault):
        if fault & 0x80:
            self.fault("Max31865 RTD input is disconnected")
        if fault & 0x40:
            self.fault("Max31865 RTD input is shorted")
        if fault & 0x20:
            self.fault(
                "Max31865 VREF- is greater than 0.85 * VBIAS, FORCE- open")
        if fault & 0x10:
            self.fault("Max31865 VREF- is less than 0.85 * VBIAS, FORCE- open")
        if fault & 0x08:
            self.fault("Max31865 VRTD- is less than 0.85 * VBIAS, FORCE- open")
        if fault & 0x04:
            self.fault("Max31865 Overvoltage or undervoltage fault")
        if fault & 0x03:
            self.fault("Max31865 Unspecified error")
        adc = adc >> 1 # remove fault bit
        R_div_nominal = adc * self.adc_to_resist_div_nominal
        # Resistance (relative to rtd_nominal_r) is calculated using:
        #  R_div_nominal = 1. + CVD_A * temp + CVD_B * temp**2
        # Solve for temp using quadratic equation:
        #  temp = (-b +- sqrt(b**2 - 4ac)) / 2a
        discriminant = math.sqrt(CVD_A**2 - 4. * CVD_B * (1. - R_div_nominal))
        temp = (-CVD_A + discriminant) / (2. * CVD_B)
        return temp
    def calc_adc(self, temp):
        # Calculate relative resistance via Callendar-Van Dusen formula:
        #  resistance = rtd_nominal_r * (1 + CVD_A * temp + CVD_B * temp**2)
        R_div_nominal = 1. + CVD_A * temp + CVD_B * temp * temp
        adc = int(R_div_nominal / self.adc_to_resist_div_nominal + 0.5)
        adc = max(0, min(MAX31865_ADC_MAX, adc))
        adc = adc << 1 # Add fault bit
        return adc
    def build_spi_init(self, config):
        value = (MAX31865_CONFIG_BIAS |
                 MAX31865_CONFIG_MODEAUTO |
                 MAX31865_CONFIG_FAULTCLEAR)
        if config.getboolean('rtd_use_50Hz_filter', False):
            value |= MAX31865_CONFIG_FILT50HZ
        if config.getint('rtd_num_of_wires', 2) == 3:
            value |= MAX31865_CONFIG_3WIRE
        cmd = 0x80 + MAX31865_CONFIG_REG
        return [cmd, value]


######################################################################
# ADS1118 thermocouple
# https://github.com/jonathanimb/ADS1118
# https://github.com/denkitronik/ADS1118
# https://github.com/moonglow/FlashForge_Marlin
# https://srdata.nist.gov/its90/download/type_k.tab
######################################################################


DEGREES = [-30, -26, -23, -20, -16, -13, -10, -6, -3, 0,
3, 6, 10, 13, 16, 20, 23, 26, 30, 33, 36, 40, 43, 46, 50, 53, 56, 60, 63, 66, 70, 73, 76, 80, 83, 86, 90, 93, 96, 100,
103, 106, 110, 113, 116, 120, 123, 126, 130, 133, 136, 140, 143, 146, 150, 153, 156, 160, 163, 166, 170, 173, 176, 180, 183, 186, 190, 193, 196, 200,
203, 206, 210, 213, 216, 220, 223, 226, 230, 233, 236, 240, 243, 246, 250, 253, 256, 260, 263, 266, 270, 273, 276, 280, 283, 286, 290, 293, 296, 300,
303, 306, 310, 313, 316, 320, 323, 326, 330, 333, 336, 340, 343, 346, 350, 353, 356, 360, 363, 366, 370, 373, 376, 380, 383, 386, 390, 393, 396, 400,
403, 406, 410, 413, 416, 420, 423, 426, 430, 433, 436, 440, 443, 446, 450, 453, 456, 460, 463, 466, 470, 473, 476, 480, 483, 486, 490, 493, 496, 497,
516, 535, 554, 573, 592, 611, 630, 649, 668, 687, 706, 725, 744, 763, 782, 801, 820, 839, 858, 877, 896, 915, 934, 953, 972, 991, 1010, 1029, 1048,
1067, 1086, 1105, 1124, 1143, 1200 ]

UVOLTS = [-1156, -1006, -892, -778, -624, -508, -392, -236, -118, 0,
119, 238, 397, 517, 637, 798, 919, 1041, 1203, 1326, 1448, 1612, 1735, 1858, 2023, 2147, 2271, 2436, 2561, 2685, 2851, 2976, 3100, 3267,
3391, 3516, 3682, 3806, 3931, 4096, 4220, 4344, 4509, 4633, 4756, 4920, 5043, 5165, 5328, 5450, 5572, 5735, 5856, 5977, 6138, 6259, 6380, 6540, 6660,
6781, 6941, 7060, 7180, 7340, 7460, 7579, 7739, 7859, 7979, 8138, 8258, 8378, 8539, 8659, 8779, 8940, 9061, 9181, 9343, 9464, 9585, 9747, 9869, 9991,
10153, 10276, 10398, 10561, 10684, 10807, 10971, 11094, 11217, 11382, 11506, 11630, 11795, 11919, 12043, 12209, 12333, 12457, 12624, 12748, 12873, 13040,
13165, 13290, 13457, 13582, 13707, 13874, 14000, 14126, 14293, 14419, 14545, 14713, 14839, 14965, 15133, 15259, 15385, 15554, 15680, 15806, 15975, 16102,
16228, 16397, 16524, 16651, 16820, 16947, 17074, 17243, 17370, 17497, 17667, 17794, 17921, 18091, 18218, 18346, 18516, 18643, 18771, 18941, 19068, 19196,
19366, 19494, 19622, 19792, 19920, 20048, 20218, 20346, 20474, 20516, 21326, 22137, 22947, 23757, 24565, 25373, 26179, 26983, 27784, 28584, 29380, 30174,
30964, 31752, 32536, 33316, 34093, 34867, 35637, 36403, 37166, 37925, 38680, 39432, 40180, 40924, 41665, 42402, 43134, 43863, 44588, 45308, 46024, 46735, 48838 ]

class ADS1118(SensorBase):
    def __init__(self, config):
        SensorBase.__init__(self, config, "ADS1118", spi_mode=1)
    def C_to_uV(self, C):
        if not -30 <= C <= 1200:
            self.fault("C out of range. Expected value between -30 and 1200, got {}".format(C))

        idx = bisect(DEGREES, C)

        uV = UVOLTS[idx-1] + ((C - DEGREES[idx-1]) * (UVOLTS[idx]-UVOLTS[idx-1]) / (DEGREES[idx]-DEGREES[idx-1]));
        return uV

    def uV_to_C(self, uV):

        if not -1156 <= uV <= 48838:
            self.fault("uV out of range. Expected value between 0 and 48838, got {}".format(uV))

        idx = bisect(UVOLTS, uV)

        C = DEGREES[idx-1] + ((uV - UVOLTS[idx-1]) * (DEGREES[idx]-DEGREES[idx-1]) / (UVOLTS[idx]-UVOLTS[idx-1]));
        return C
        
    def calc_temp(self, adc, fault):
        ch = ( adc & 0xFFFF0000 ) >> 16
        it = ( adc & 0x0000FFFF )

        if it & 0x2000: #1<<13
            # self.fault("ADS1118 Negative internal temperature, got {}".format(it))
            raw_it = ( (~(it>>2)-1) & 0x1FFF) * 0.03125 * -1 #0.03125 - Temperature resolution 
        else:
            raw_it = (it>>2) * 0.03125 #0.03125 - Temperature resolution 

        # if ch & 0x8000: #1<<15
        #     self.fault("ADS1118 Negative sensor temperature, got {}".format(ch))

        raw_ch = ch * 256000 / 32768 # (FSR 0.256 * 1000000)/(1<<15)

        return self.uV_to_C(raw_ch + self.C_to_uV(raw_it));

    def calc_adc(self, temp):
        if temp <= 20:
            return 0
        
        it = 2560; #(20*C / 0.03125) << 2 = 0x0A00
        raw_ch = self.C_to_uV(temp-20)
        ch = raw_ch * 32768 / 256000

        return ((int(ch)<<16|it))

######################################################################
# Sensor registration
######################################################################

Sensors = {
    "MAX6675": MAX6675,
    "MAX31855": MAX31855,
    "MAX31856": MAX31856,
    "MAX31865": MAX31865,
    "ADS1118": ADS1118,
}

def load_config(config):
    # Register sensors
    pheaters = config.get_printer().load_object(config, "heaters")
    for name, klass in Sensors.items():
        pheaters.add_sensor_factory(name, klass)
