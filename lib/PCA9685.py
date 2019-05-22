# coding: utf-8
import time

class PCA9685(object):
    # PCA9685 Default I2C address
    PCA9685_ADDRESS = 0x40

    # Value of servlo
    MODE1 = 0x00
    MODE2 = 0x01
    OSC_CLOCK = 25000000.0

    LED0_ON_L = 0x06
    LED0_ON_H = 0x07
    LED0_OFF_L = 0x08
    LED0_OFF_H = 0x09

    # PCA9685に電源を入れたときに、全部の値を初期化する際に用いる
    ALL_LED_ON_L = 0xFA
    ALL_LED_ON_H = 0xFB
    ALL_LED_OFF_L = 0xFC
    ALL_LED_OFF_H = 0xFD

    PRE_SCALE = 0xFE

    # MODE1 Bit
    RESTART = 0x80
    SLEEP = 0x10
    ALLCALL = 0x01

    # MODE2 Bit
    OUTDRV = 0x04

    # PWMを50Hzに設定
    PWM_HZ = 50

    WAIT_TIME = 0.005 # wait at least 500 microseconds

    ''' PCA9685 Registers
    # Register definitions
    MODE1 = 0x00 # Mode register 1
    MODE2 = 0x01 # Mode register 2
    SUBADR1 = 0x02 # I2C-bus subaddress 1
    SUBADR2 = 0x03 # I2C-bus subaddress 2
    SUBADR3 = 0x04 # I2C-bus subaddress 3
    ALLCALLADR = 0x05 # LED All Call I2C-bus address
    LED0_ON_L = 0x06 # LED0 output and brightness control byte 0
    LED0_ON_H = 0x07 # LED0 output and brightness control byte 1
    LED0_OFF_L = 0x08 # LED0 output and brightness control byte 2
    LED0_OFF_H = 0x09 # LED0 output and brightness control byte 3
    LED1_ON_L = 0x0A # LED1 output and brightness control byte 0
    LED1_ON_H = 0x0B # LED1 output and brightness control byte 1
    LED1_OFF_L = 0x0C # LED1 output and brightness control byte 2
    LED1_OFF_H = 0x0D # LED1 output and brightness control byte 3
    LED2_ON_L = 0x0E # LED2 output and brightness control byte 0
    LED2_ON_H = 0x0F # LED2 output and brightness control byte 1
    LED2_OFF_L = 0x10 # LED2 output and brightness control byte 2
    LED2_OFF_H = 0x11 # LED2 output and brightness control byte 3
    LED3_ON_L = 0x12 # LED3 output and brightness control byte 0
    LED3_ON_H = 0x13 # LED3 output and brightness control byte 1
    LED3_OFF_L = 0x14 # LED3 output and brightness control byte 2
    LED3_OFF_H = 0x15 # LED3 output and brightness control byte 3
    LED4_ON_L = 0x16 # LED4 output and brightness control byte 0
    LED4_ON_H = 0x17 # LED4 output and brightness control byte 1
    LED4_OFF_L = 0x18 # LED4 output and brightness control byte 2
    LED4_OFF_H = 0x19 # LED4 output and brightness control byte 3
    LED5_ON_L = 0x1A # LED5 output and brightness control byte 0
    LED5_ON_H = 0x1B # LED5 output and brightness control byte 1
    LED5_OFF_L = 0x1C # LED5 output and brightness control byte 2
    LED5_OFF_H = 0x1D # LED5 output and brightness control byte 3
    LED6_ON_L = 0x1E # LED6 output and brightness control byte 0
    LED6_ON_H = 0x1F # LED6 output and brightness control byte 1
    LED6_OFF_L = 0x20 # LED6 output and brightness control byte 2
    LED6_OFF_H = 0x21 # LED6 output and brightness control byte 3
    LED7_ON_L = 0x22 # LED7 output and brightness control byte 0
    LED7_ON_H = 0x23 # LED7 output and brightness control byte 1
    LED7_OFF_L = 0x24 # LED7 output and brightness control byte 2
    LED7_OFF_H = 0x25 # LED7 output and brightness control byte 3
    LED8_ON_L = 0x26 # LED8 output and brightness control byte 0
    LED8_ON_H = 0x27 # LED8 output and brightness control byte 1
    LED8_OFF_L = 0x28 # LED8 output and brightness control byte 2
    LED8_OFF_H = 0x29 # LED8 output and brightness control byte 3
    LED9_ON_L = 0x2A # LED9 output and brightness control byte 0
    LED9_ON_H = 0x2B # LED9 output and brightness control byte 1
    LED9_OFF_L = 0x2C # LED9 output and brightness control byte 2
    LED9_OFF_H = 0x2D # LED9 output and brightness control byte 3
    LED10_ON_L = 0x2E # LED10 output and brightness control byte 0
    LED10_ON_H = 0x2F # LED10 output and brightness control byte 1
    LED10_OFF_L = 0x30 # LED10 output and brightness control byte 2
    LED10_OFF_H = 0x31 # LED10 output and brightness control byte 3
    LED11_ON_L = 0x32 # LED11 output and brightness control byte 0
    LED11_ON_H = 0x33 # LED11 output and brightness control byte 1
    LED11_OFF_L = 0x34 # LED11 output and brightness control byte 2
    LED11_OFF_H = 0x35 # LED11 output and brightness control byte 3
    LED12_ON_L = 0x36 # LED12 output and brightness control byte 0
    LED12_ON_H = 0x37 # LED12 output and brightness control byte 1
    LED12_OFF_L = 0x38 # LED12 output and brightness control byte 2
    LED12_OFF_H = 0x39 # LED12 output and brightness control byte 3
    LED13_ON_L = 0x3A # LED13 output and brightness control byte 0
    LED13_ON_H = 0x3B # LED13 output and brightness control byte 1
    LED13_OFF_L = 0x3C # LED13 output and brightness control byte 2
    LED13_OFF_H = 0x3D # LED13 output and brightness control byte 3
    LED14_ON_L = 0x3E # LED14 output and brightness control byte 0
    LED14_ON_H = 0x3F # LED14 output and brightness control byte 1
    LED14_OFF_L = 0x40 # LED14 output and brightness control byte 2
    LED14_OFF_H = 0x41 # LED14 output and brightness control byte 3
    LED15_ON_L = 0x42 # LED15 output and brightness control byte 0
    LED15_ON_H = 0x43 # LED15 output and brightness control byte 1
    LED15_OFF_L = 0x44 # LED15 output and brightness control byte 2
    LED15_OFF_H = 0x45 # LED15 output and brightness control byte 3

    ALL_LED_ON_L = 0xFA # load all the LEDn_ON registers, byte 0
    ALL_LED_ON_H = 0xFB # load all the LEDn_ON registers, byte 1
    ALL_LED_OFF_L = 0xFC # load all the LEDn_OFF registers, byte 0
    ALL_LED_OFF_H = 0xFD # load all the LEDn_OFF registers, byte 1
    PRE_SCALE = 0xFE # prescaler for PWM output frequency
    TestMode = 0xFF # defines the test mode to be entered
    '''

    ''' PCA9685 MODE1 bit
    RESTART = 0x10 # 7bit = 2^7 = 128
    EXTCLK = 0x40 # 6bit = 2^6 = 64
    AI = 0x20 # 5bit = 2^5 = 32
    SLEEP = 0x10 # 4bit = 2^4 = 16
    SUB1 = 0x08 # 3bit = 2^3 = 8
    SUB2 = 0x04 # 2bit = 2^2 = 4
    SUB3 = 0x02 # 1bit = 2^1 = 2
    ALLCALL = 0x01 # 0bit = 2^0 = 1
    '''

    ''' PCA9685 MODE2 bit
    INVRT = 0x10 # 4bit = 2^4 = 16
    OCH = 0x08 # 3bit
    OUTDRV = 0x04 # 2bit
    '''


    def __init__(self, bus, value=300, address=0x40):
        '''
        bus: bus番号
        value: PCA9685に書き込む初期サーボ位置
        '''
        self.bus = bus
        value = int(value)
        self.PCA9685_ADDRESS = address

        #mode1 = self.get_mode1()
        #print("before mode1:{}".format(mode1))

        # PCA9685 全channelの値を初期化する
        # 通電時、PCA9685の全channleの値は、サーボ稼働範囲外の4096になっているのでこれを適切な範囲に設定しておく
        self.bus.write_byte_data(self.PCA9685_ADDRESS, self.ALL_LED_ON_L, 0x00)
        self.bus.write_byte_data(self.PCA9685_ADDRESS, self.ALL_LED_ON_H, 0x00)
        self.bus.write_byte_data(self.PCA9685_ADDRESS, self.ALL_LED_OFF_L, (value & 0xFF))
        self.bus.write_byte_data(self.PCA9685_ADDRESS, self.ALL_LED_OFF_H, (value >> 8))

        self.bus.write_byte_data(self.PCA9685_ADDRESS, self.MODE2, self.OUTDRV)
        self.bus.write_byte_data(self.PCA9685_ADDRESS, self.MODE1, self.ALLCALL)
        time.sleep(self.WAIT_TIME)

        # スリープ状態なら解除する
        mode = self.bus.read_byte_data(self.PCA9685_ADDRESS, self.MODE1)
        mode = mode & ~self.SLEEP # SLEEPビットを除去する
        self.bus.write_byte_data(self.PCA9685_ADDRESS, self.MODE1, mode)
        time.sleep(self.WAIT_TIME)


        #frequency設定はinitを抜けてから行うこと。
        # ex.
        # pca9685 = PCA9685()
        # hz = 60
        # pca9685.set_hz(hz)

    def __del__(self):
        """
        分離したコードでサーボとモーターのどちらかを止めてもPCA9685がスリープに入らないようにするために、この処理はコメントアウトにする。
        mode = self.bus.read_byte_data(self.PCA9685_ADDRESS, self.MODE1)
        mode = mode | self.SLEEP # sleep
        # スリープにする
        self.bus.write_byte_data(self.PCA9685_ADDRESS, self.MODE1, mode)
        return
        """

    def calc_prescale(self, hz):
        '''
        hzをprescaleに変換した値を返す
        '''
        return int(round(self.OSC_CLOCK/4096/hz)-1)

    def calc_hz(self, prescale):
        '''
        prescaleをhzに変換した値を返す
        誤差が出てしまうので参考値程度に
        '''
        d = None
        if (prescale +1) % 2 == 0:
            d = 0.5
        else:
            d = 4.999999e-1

        hz_min = int(self.OSC_CLOCK/4096/(prescale+1+d))
        hz_max = int(self.OSC_CLOCK/4096/(prescale+1-d))
        return hz_min, hz_max

    def set_hz(self, hz):
        self.PWM_HZ = hz
        prescale=self.calc_prescale(hz)
        '''
        Hz設定。レジスタに書き込む値はprescaleの値となる
        prescale = calc_prescale(hz)
        '''
        oldmode = self.bus.read_byte_data(self.PCA9685_ADDRESS, self.MODE1)
        newmode = oldmode | self.SLEEP # sleep

        #print("prescale:{}".format(prescale))
        #print("oldmode:{}".format(oldmode))
        #print("newmode:{}".format(newmode))
        # スリープにする
        self.bus.write_byte_data(self.PCA9685_ADDRESS, self.MODE1, newmode)
        #周波数を設定
        self.bus.write_byte_data(self.PCA9685_ADDRESS, self.PRE_SCALE, prescale)
        #スリープを解除
        self.bus.write_byte_data(self.PCA9685_ADDRESS, self.MODE1, oldmode)
        time.sleep(self.WAIT_TIME)
        # リスタートする（必須）
        self.bus.write_byte_data(self.PCA9685_ADDRESS, self.MODE1, (oldmode | self.RESTART))

    def get_hz(self):
        '''
        設定されているprescale値を取得し、保持しているPWM_HZと一致するかどうかを計算し、
        一致している場合はPWM_HZを返す
        それ以外は推定HZ範囲の中間値を返す
        '''
        value = None
        prescale = self.bus.read_byte_data(self.PCA9685_ADDRESS, self.PRE_SCALE) # レジストリのprescale値
        _prescale = self.calc_prescale(self.PWM_HZ) # 保持しているhzからprescale値算出
        if _prescale == prescale:
            value = self.PWM_HZ
        else:
            values = self.calc_hz(prescale)
            value0 = self.calc_prescale(values[0])
            value1 = self.calc_prescale(values[1])

            value = int((value0 + value1)/2)

        return value

    def get_prescale(self):
        '''
        設定されているprescale値を取得する
        prescale値はset_freq(hz)で設定するが、引数となるhzを正確に逆算する方法を思いつかないため、
        実際にレジストリに書かれているprescale値を返す
        '''
        prescale = self.bus.read_byte_data(self.PCA9685_ADDRESS, self.PRE_SCALE)
        return prescale

    def get_channel_value(self, channel):
        '''
        1byteの値を読み込む
        channel: PCA9685に接続しているPWM番号
        return: PCA9685に設定されているパルス幅
        '''
        block0 = self.bus.read_byte_data(self.PCA9685_ADDRESS, self.LED0_OFF_L+channel*4) # 0-255
        block256 = self.bus.read_byte_data(self.PCA9685_ADDRESS, self.LED0_OFF_H+channel*4) # 桁上がり
        value = (block256<<8) + block0
        ''' or
        read_word_data()は2byteを返すが、1byteしか要らないので削る。
        PCA9685は1byteしかレジスタ値を持っていないが、i2cは1byteを繰り返して2byteを返す
        block0 = self.bus.read_word_data(self.PCA9685_ADDRESS, self.LED0_OFF_L+channel*4) & 0xFF # 0-255
        block256 = self.bus.read_word_data(self.PCA9685_ADDRESS, self.LED0_OFF_H+channel*4) & 0xFF # 桁上がり
        value = (block256<<8) + block0
        '''
        ''' or
        read_i2c_block_data()は指定したbyte数をバイト配列で返す。1byteしか要らない。16byteを指定しても同じ値が16個並ぶだけ。
        list_of_bytes = 1 # 1byteだけ取る
        block0 = self.bus.read_i2c_block_data(self.PCA9685_ADDRESS, self.LED0_OFF_L+channel*4, list_of_bytes) # 0-255
        block256 = self.bus.read_i2c_block_data(self.PCA9685_ADDRESS, self.LED0_OFF_H+channel*4, list_of_bytes) # 桁上がり
        value = (block256[0]<<8) + block0[0]
        '''
        return value

    def set_channel_value(self, channel, value):
        '''
        1byteの値を書き込む
        channel: PCA9685に接続しているPWM番号
        value: パルス幅
        '''
        value=int(value)

        self.bus.write_byte_data(self.PCA9685_ADDRESS, self.LED0_ON_L+channel*4, 0x00)
        self.bus.write_byte_data(self.PCA9685_ADDRESS, self.LED0_ON_H+channel*4, 0x00)
        self.bus.write_byte_data(self.PCA9685_ADDRESS, self.LED0_OFF_L+channel*4, (value & 0xFF))
        self.bus.write_byte_data(self.PCA9685_ADDRESS, self.LED0_OFF_H+channel*4, (value >> 8))
        ''' or 
        self.bus.write_i2c_block_data(self.PCA9685_ADDRESS, self.LED0_ON_L+channel*4, [0x00])
        self.bus.write_i2c_block_data(self.PCA9685_ADDRESS, self.LED0_ON_H+channel*4, [0x00])
        self.bus.write_i2c_block_data(self.PCA9685_ADDRESS, self.LED0_OFF_L+channel*4, [value & 0xFF])
        self.bus.write_i2c_block_data(self.PCA9685_ADDRESS, self.LED0_OFF_H+channel*4, [value >> 8])
        '''

    def get_mode1(self):
        '''
        MODE1のアドレス値を取得する
        '''
        mode1 = self.bus.read_byte_data(self.PCA9685_ADDRESS, self.MODE1)
        return mode1
