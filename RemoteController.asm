/*
 * Author: Si Kedong (skd2278@gmail.com)
 */

INDR EQU 0x100
TMR0 EQU 0x101
PCL EQU 0x102
STATUS EQU 0x103
    C   EQU 0
    DC  EQU 1
    Z   EQU 2
    PD  EQU 3
    TO  EQU 4
    GPR EQU 5
RAMS EQU 0x104
PORTA EQU 0x105
PORTB EQU 0x106
PORTC EQU 0x107
PAS EQU 0x115
PBS EQU 0x116
PCS EQU 0x117
PBW EQU 0x120
PAW EQU 0x121
MCR EQU 0x122
ODB EQU 0x123
IORA EQU 0x125
IORB EQU 0x126
IORC EQU 0x127
IOPA EQU 0x12A
IOPD EQU 0x12B
T1CON EQU 0x12C
IOPH EQU 0x12D
WDTS EQU 0x12E
    WDTE    EQU 5
INTE EQU 0x12F
    TR0IE   EQU 0
    EXIE    EQU 1
    SPIIE   EQU 2
    TR1IE   EQU 3
    ICIE    EQU 4
    WDTIE   EQU 7
PWM1D EQU 0x130
PWM2D EQU 0x131
PWM1P EQU 0x132
PWM2P EQU 0x133
PWMC EQU 0x134
    PWS0    EQU 0
    PWS1    EQU 1
    PWS2    EQU 2
    PW1E    EQU 6
    PW2E    EQU 7
PTMRC EQU 0x135
TMR1 EQU 0x13E
TR1CV EQU 0x13F
PAL EQU 0x14B
PAH EQU 0x14C
IOP2 EQU 0x15F
IOSK2 EQU 0x171
IOFS EQU 0x172
IOBUF2 EQU 0x173
IOSK1 EQU 0x174
IOBUF1 EQU 0x175
OSCC EQU 0x178
LDOC EQU 0x17B
LVRC EQU 0x17D
INTF EQU 0x17F
    TR0IF   EQU 0
    EXIF    EQU 1
    SPIIF   EQU 2
    TR1IF   EQU 3
    ICIF    EQU 4
    WDTIF   EQU 7

    
;Author: Si Kedong 
;Email: skd2278@gmail.com

;/*Stack*/
rSTACK_SP       EQU 0x20
rSTACK_STATUS   EQU 0x21
rSTACK_RAMS     EQU 0x22

;/*General registers*/
R0  EQU 0x24
R1  EQU 0x25
R2  EQU 0x26
R3  EQU 0x27

;/*LED*/
rO_LED              EQU PORTB
    LED_B           EQU 0
;/*Key registers*/
rKEY_STATE          EQU 0x38
rKEY_PRESS_TIME     EQU 0x39
rKEY_INTERVAL_CNT   EQU 0x3A 
rKEY_VALUE_L        EQU 0x3B
rKEY_VALUE_H        EQU 0x3C
rKEY_VAL_SAVED_L    EQU 0x2E
rKEY_VAL_SAVED_H    EQU 0x2F
    ;/*bit7: is_read_flag;*/
    ;/*bit6: release flag, 1: release*/
    ;/*bit5~0: how long pressed (unit: s);*/
    ;/*秒计数器，这个时间+KEY_LPRESS_TIME，即为按了多少秒*/
    KEY_READ_B      EQU 7
    KEY_RELEASE_B   EQU 6

KEY_VAL_S2_ON       EQU 0b00010001
KEY_VAL_S2_OFF      EQU 0b00100001
KEY_VAL_COLOR_INC   EQU 0b01000001
KEY_VAL_S3_OFF      EQU 0b10000001
KEY_VAL_S3_ON       EQU 0b00010010
KEY_VAL_S1_OFF      EQU 0b00100010
KEY_VAL_S4_OFF      EQU 0b01000010
KEY_VAL_LUMIN_DEC   EQU 0b10000010
KEY_VAL_COLOR_DEC   EQU 0b00010100
KEY_VAL_S1_ON       EQU 0b00100100
KEY_VAL_S4_ON       EQU 0b01000100
KEY_VAL_LUMIN_INC   EQU 0b10000100
KEY_VAL_MASTER_ON   EQU 0b01001000
KEY_VAL_MASTER_OFF  EQU 0b10001000  
    
    
;/*IIC registers*/  
rO_IIC_SCL      EQU PORTB   ;SCL写端口寄存器
rO_IIC_SDA      EQU PORTB   ;SDA写端口寄存器
rI_IIC_SDA      EQU PBS     ;SDA读端口寄存器
rI_IIC_SCL      EQU PBS     ;SCL读端口寄存器
    IIC_SCL_B   EQU 5       ;SCL端口Pin
    IIC_SDA_B   EQU 4       ;SDA端口Pin

;/*eeprom registers*/
rEEPROM_ADDR        EQU     0x28
rEEPROM_DATA        EQU     0x29
rEEPROM_DATA_LEN    EQU     0x2A
rEEPROM_PBUF        EQU     0x2B
    
;/*RF SPI registers*/
rO_SPI_SS       EQU PORTA
rO_SPI_CLK      EQU PORTA
rO_SPI_MOSI     EQU PORTA
rI_SPI_MISO     EQU PBS
    SPI_SS_B    EQU 4
    SPI_CLK_B   EQU 2
    SPI_MOSI_B  EQU 0
    SPI_MISO_B  EQU 7

    
;/*RF registers*/
rO_RF_RESET     EQU PORTB
    RF_RESET_B  EQU 6
rI_RF_PKT       EQU PAS
    RF_PKT_B    EQU 3
    
rTEST               EQU 0x37
    RF_TEST_ERROR_B EQU 6
    RF_INIT_OK_B    EQU 7
rRX_TIMER           EQU 0x3F

rRX_BUF         EQU 0x00    ;alias name
rRX_BUF0        EQU 0x00
rRX_BUF1        EQU 0x01
rRX_BUF2        EQU 0x02
rRX_BUF3        EQU 0x03
rRX_BUF4        EQU 0x04
rRX_BUF5        EQU 0x05
rRX_BUF6        EQU 0x06
rRX_BUF7        EQU 0x07

rROLLING_CODE1  EQU rRX_BUF1
rROLLING_CODE2  EQU rRX_BUF2
rROLLING_CODE3  EQU rRX_BUF3
rROLLING_CODE4  EQU rRX_BUF4
rRX_KEY_LOW     EQU rRX_BUF5
rRX_KEY_HIGH    EQU rRX_BUF6
rKEY_SAVED_VAL  EQU 0x3D



;/*预留8个字节*/
rTX_BUF         EQU 0x08
rTX_BUF0        EQU 0x08
rTX_BUF1        EQU 0x09
rTX_BUF2        EQU 0x0A
rTX_BUF3        EQU 0x0B
rTX_BUF4        EQU 0x0C
rTX_BUF5        EQU 0x0D
rTX_BUF6        EQU 0x0E
rTX_BUF7        EQU 0x0F
;/*预留8个字节*/

rFRAME_LOST_CNT     EQU 0x35
rLAMP_COMM_STATUS   EQU 0x36
    LOST_FRAME_B    EQU 0
    RECV_ERROR_B    EQU 1
    
rSPI_ADDR       EQU 0x30
rSPI_DATA_H     EQU 0x31
rSPI_DATA_L     EQU 0x32
rSPI_BUF        EQU 0x33
rRX_DATA_LEN    EQU 0x34    
    
    
MASK_KEY_COL        EQU 0xF0
MASK_KEY_ROW        EQU 0x0F

MASK_PORTA_AS_KEY   EQU 0xC0
MASK_PORTC_AS_KEY   EQU 0x03
;C3C2C1C0R3R2R1R0
ROW0    EQU 6   ;PA6
ROW1    EQU 7   ;PA7
ROW2    EQU 0   ;PC0
ROW3    EQU 1   ;PC1

COL0    EQU 4   ;PB4
COL1    EQU 5   ;PB5
COL2    EQU 3   ;PB3
COL3    EQU 1   ;PB1

;Key time
KEY_DEJITTER_TIME   EQU 2   ;(x10ms)
KEY_LPRESS_TIME     EQU 200 ;(x10ms)
KEY_LPRESS_1S       EQU 100 ;(x10ms)
KEY_INTERVAL_TIME   EQU 20  ;(x10ms)continus hit

;Key state
KEY_STATE_IDLE      EQU 0
KEY_STATE_PRESS     EQU 1
KEY_STATE_DEJITTER  EQU 2
KEY_STATE_SPRESS    EQU 3
KEY_STATE_DPRESS    EQU 4
KEY_STATE_RELEASE   EQU 5


NORMAL_DATA_LEN     EQU 7
SYNC_ADDR_1         EQU 36
SYNC_ADDR_2         EQU 37
SYNC_ADDR_3         EQU 38
SYNC_ADDR_4         EQU 39
RF_RESERVED_DATA    EQU 0xCC
CHANNEL_INDEX       EQU 1

ROLLING_CODE_ADDR_L EQU 0xFD
ROLLING_CODE_ADDR_H EQU 0x0F

STACK_TOP   EQU 0x80        ;满递减栈，SP指向最大内存+1字节
    
    ORG     0x0000
    JMP     start
    ORG     0x0FFF
    JMP     start
    ORG     0x0003
/*  JMP     lvd_handler
    ORG     0x0005
    JMP     eint_handler
    ORG     0x0007
    JMP     wake_up_handler
*/  
    ORG     0x0009
    JMP     tcc_overflow_handler

/*
    ORG     0x000B
    JMP     watch_dog_handler

    ORG     0x0011
    JMP     timer1_handler
    ORG     0x0013
    JMP     pwm_handler
*/
    ORG     0x0015


/*********************************************************************/
;RF频道
LAMP_CHANNEL_TABLE:
    ADD     PCL, A
    RETLW   @0x30
    RETLW   @0x30
    RETLW   @0x30
/*********************************************************************/
;SyncWord
ID_TABLE:
    ADD     PCL, A
    RETLW   @0x55
    RETLW   @0xAA
    
/*********************************************************************/
/*
64个bit允许错3个:    reg32:5800, reg40:4404
48个bit允许错2个:    reg32:5000, reg40:4403
32个bit允许错1个:    reg32:4800, reg40:4402
16个bit不允许错: reg32:4000, reg40:4401
*/
FRAME_TABLE:    ; other Register the default value equal recommended value
    ADD     PCL, A
;---------------------------------------------------------------------

    RETLW   @0x00   ; Reg0
    RETLW   @0x6F
    RETLW   @0xE0   ;

    RETLW   @0x01   ; Reg1
    RETLW   @0x56
    RETLW   @0x81

    RETLW   @0x02   ; Reg2
    RETLW   @0x66
    RETLW   @0x17

    RETLW   @0x04   ; Reg4
    RETLW   @0x9C
    RETLW   @0xC9

    RETLW   @0x05   ; Reg5
    RETLW   @0x66
    RETLW   @0x37

    RETLW   @0x07   ; Reg7
    RETLW   @0x00
    RETLW   @0x00   ; Use for setting RF freqency

    RETLW   @0x08   ; Reg8
    RETLW   @0x6C
    RETLW   @0x90

    RETLW   @0x09   ; Reg9
    RETLW   @0x48
    RETLW   @0x00   ; Sets TX power level

    RETLW   @0x0A   ; Reg10
    RETLW   @0x7F
    RETLW   @0xFD   ; Crystal osc. enable

    RETLW   @0x0B   ; Reg11
    RETLW   @0x00
    RETLW   @0x08   ; RSSI enable

    RETLW   @0x0C   ; Reg12
    RETLW   @0x00
    RETLW   @0x00

    RETLW   @0x0D   ; Reg13
    RETLW   @0x48
    RETLW   @0xBD

    RETLW   @0x16   ; Reg22
    RETLW   @0x00
    RETLW   @0xFF

    RETLW   @0x17   ; Reg23
    RETLW   @0x80
    RETLW   @0x05

    RETLW   @0x18   ; Reg24
    RETLW   @0x00
    RETLW   @0x67

    RETLW   @0x19   ; Reg25
    RETLW   @0x16
    RETLW   @0x59

    RETLW   @0x1A   ; Reg26
    RETLW   @0x19
    RETLW   @0xE0

    RETLW   @0x1B   ; Reg27
    RETLW   @0x13
    RETLW   @0x00   ; 

    RETLW   @0x20   ; Reg32
    RETLW   @0x40
    RETLW   @0x00   ; 

    RETLW   @0x21   ; Reg33
    RETLW   @0x3F
    RETLW   @0xC7   ; Configures packet sequencing

    RETLW   @0x22   ; Reg34
    RETLW   @0x20
    RETLW   @0x00   ; Configures packet sequencing

    RETLW   @0x23   ; Reg35
    RETLW   @0x04
    RETLW   @0x00   ; AutoACK  TX retries = 3 

    RETLW   @0x24   ; Reg36
    RETLW   @0x19
    RETLW   @0x85   ; Sync Word

    RETLW   @0x25   ; Reg37
    RETLW   @0x10
    RETLW   @0x21   ; Sync Word

    RETLW   @0x26   ; Reg38
    RETLW   @0x19
    RETLW   @0x85   ; Sync Word

    RETLW   @0x27   ; Reg39
    RETLW   @0x05
    RETLW   @0x21   ; Sync Word

    RETLW   @0x28   ; Reg40
    RETLW   @0x44
    RETLW   @0x01   ; Configure FIFO flag,sync treshold

    RETLW   @0x29   ; Reg41
    RETLW   @0xB8
    RETLW   @0x00   ; CRC on,SCRAMBLE off,1st byte is packet length

    RETLW   @0x2A   ; Reg42
    RETLW   @0xFD
    RETLW   @0xB0   ;

    RETLW   @0x2B   ; Reg43
    RETLW   @0x00
    RETLW   @0x0F   ; Configure scan RSSI
/*
    RETLW   @0x2C
    RETLW   @0x01
    RETLW   @0x00
    
    RETLW   @0x2D
    RETLW   @0x05
    RETLW   @0x52
*/  
    RETLW   @0x32   ; Reg50
    RETLW   @0x00
    RETLW   @0x00

    RETLW   @0xFF   
    
    
/*  
lvd_handler:
    JMP     $
eint_handler:
    JMP     $
wake_up_handler:
    JMP     $
*/  
tcc_overflow_handler:           ;该中断大约每10ms发生一次
    PUSH
    BCR     INTF, TR0IF         ;清中断
    ;CALL   key_scan            ;键盘扫描
    POP
    RETINT
/*  
watch_dog_handler:
    JMP     $
timer1_handler:
    JMP     $
pwm_handler:
    JMP     $
*/

    
    
clear_ram:
    MOV     A, @0x7F
    MOV     RAMS, A
_clear_ram_loop:
    CLR     INDR
    DRSZR   RAMS    
    JMP     _clear_ram_loop
    CLR     INDR
    RETURN
    
start:
    DISI                        ;关中断
    BCR     WDTS, WDTE          ;关闭开门狗
    CALL    clear_ram
    CALL    stack_init
    ;CALL   watch_dog_init      ;TODO:看门狗初始化
    CALL    key_init            ;按键初始化
    ;CALL   timer0_init_device  ;定时器0初始化
    CALL    rf_init_module      ;RF模块初始化
    ENI
_main_loop:
    WDTC
    MOV     A, @90
    CALL    delay_100us
    CALL    key_scan
    ;TODO: 定时喂狗
    CALL    read_key_value      ;读取按键
    MOV     A, rKEY_VAL_SAVED_L
    AND     A, @0xFF
    BTSC    STATUS, Z           ;判断键值是否为0
    JMP     _main_loop          ;为0，跳转继续读按键
    CALL    rf_transmit_data    ;不为0，发送数据
    CLR     rKEY_VAL_SAVED_H
    CLR     rKEY_VAL_SAVED_L
    JMP     _main_loop          ;继续循环
    
    JMP     $                   ;为防止程序跑飞
    
    
    
led_init_device:
    ;LED 指示灯 PB0 output
    BCR     IORB, 0
    BCR     IOPH, 0
    BCR     PORTA, 0
    RETURN

rf_init_device:
    ;Reset Pin PB6 output
    BCR     IORB, 6
    BCR     IOPH, 6
    ;PKT_FLAG PA3 input
    BSR     IORA, 3
    ;TODO
    CALL    spi_init_device     ;SPI I/O初始化
    RETURN 
    
timer1_init_device:
    CLR     TMR1
    MOV     A, @30          ;1KHZ
    MOV     TR1CV, A
    MOV     A, @0x04
    MOV     T1CON, A
    BSR     INTE, TR1IE
    RETURN
    
timer0_init_device:
    MOV     A, @0xA7
    MOV     MCR, A
    CLR     TMR0
    BSR     INTE, TR0IE
    RETURN
    
iic_init_device:
    ;SDA config PORTB->4
    BCR     IORB, 4     ;output
    BCR     IOPH, 4     ;pull high
    BSR     ODB, 4      ;open drain enable
    ;SCL config PORTB->5
    BCR     IORB, 5     ;output
    BCR     IOPH, 5     ;pull high
    BCR     ODB, 5      ;open drain disable
    RETURN
    
spi_init_device:
    ;SPI CLK config PORTA->2
    BCR     IORA, 2     ;output
    ;SPI SS config PORTA->4
    BCR     IORA, 4     ;output
    ;SPI MOSI config PORTA->0
    BCR     IORA, 0     ;output
    ;SPI MISO config PORTB->7
    BSR     IORB, 7     ;input
    ;SPI IIC_SELECT config PORTA->1
    BCR     IORA, 1     ;output
    BCR     PORTA, 1    ;output 0, select SPI mode
    RETURN
    
pwm_init_device:
    ;LED config White Pin
    BCR     IORB, 3     ;output
    BCR     IOPH, 3     ;pull high
    ;Yellow pin
    BCR     IORB, 1     ;output
    BCR     IOPH, 1     ;pull high
    RETURN
    
key_init_device:
    ;Row config PortB->4,5,3,1 input
    BSR     IORB, 4
    BCR     IOPH, 4
    BSR     IORB, 5
    BCR     IOPH, 5
    BSR     IORB, 3
    BCR     IOPH, 3
    BSR     IORB, 1
    BCR     IOPH, 1
    
    ;Col config PORTA->6,7 PORTC->0,1 output
    BCR     IORA, 6
    BCR     IOPA, 6
    BCR     IORA, 7
    BCR     IOPA, 7
    
    BCR     IORC, 0
    BCR     IORC, 1
    BCR     IOP2, 3
    RETURN




;uint8_t read_key_col(void) start
read_key_col: ;Map All Key Col Pin Value to Reg A
    ;R0存储C3C2C1C0_0000
    CALL    stack_push
    CLR     R0  ;clear R0
    MOV     A, PBS  ;Read PORTB
    MOV     R1, A
    BTSC    R1, COL0
    BSR     R0, 4
    BTSC    R1, COL1
    BSR     R0, 5
    BTSC    R1, COL2
    BSR     R0, 6
    BTSC    R1, COL3
    BSR     R0, 7
    MOV     A, R0   ;Return value<-A
    CALL    stack_pop
    RETURN
;uint8_t read_key_col(void) end
    
;void write_key_row(uint8_t key_val) start  
write_key_row: ;Map A to All key row pin
    CALL    stack_push
    MOV     R0, A                       ;A = key_val
    MOV     A, PORTA                    ;Read PORTA;这种读取PORTA的数据对吗？
    OR      A, @MASK_PORTA_AS_KEY
    MOV     R1, A                       ;Backup PORTA to R1
    BTSS    R0, 0
    BCR     R1, ROW0
    BTSS    R0, 1
    BCR     R1, ROW1
    MOV     A, R1
    MOV     PORTA, A                    ;Output to PORTA
    MOV     A, PORTC                    ;;这种读取PORTC的数据对吗？
    OR      A, @MASK_PORTC_AS_KEY
    MOV     R1, A                       ;Backup PORTC to R1
    BTSS    R0, 2
    BCR     R1, ROW2
    BTSS    R0, 3
    BCR     R1, ROW3
    MOV     A, R1
    MOV     PORTC, A                    ;Output to PORTC
    CALL    stack_pop
    RETURN
;void write_key_row(uint8_t key_val) end

    
;int key_read(void) start
;return: 0 means no key
;        >0 means keyval    
key_read:
    CALL    stack_push
    MOV     A, @MASK_KEY_COL    ;;C3C2C1C0R3R2R1R0 = 0b11110000
    CALL    write_key_row       ;矩阵键盘先所有行置0
    CALL    read_key_col        ;然后读列数据看是否有列为0
    AND     A, @MASK_KEY_COL    ;Get C3C2C1C0_0000
    XOR     A, @MASK_KEY_COL    ;
    BTSS    STATUS, Z           ;判断是否有按键按下
    JMP     _WHICH_KEY          ;有，跳转检测是哪个按键按下
    JMP     _KEY_READ_NO_KEY    ;无
_WHICH_KEY:
    BSR     rO_LED, LED_B       ;led on
    CLR     R1
    MOV     R0, A
    MOV     A, @4
    MOV     R2, A
_HOW_MANY_KEY:                  ;检测同时有几个按键按下，过滤掉多于一个按键按下的情况
    RLC     R0
    BTSC    STATUS, C
    INC     R1
    DRSZR   R2
    JMP     _HOW_MANY_KEY
    MOV     A, R1
    XOR     A, @0x01
    BTSS    STATUS, Z
    JMP     _KEY_READ_NO_KEY    ;No Key Return 0->filter multiple keys press
;SCAN ROW
    BSR     STATUS, C   
    MOV     A, @0xFE
    MOV     R0, A
    MOV     A, @4
    MOV     R1, A
_SCAN_ROWX:
    MOV     A, R0
    CALL    write_key_row
    CALL    read_key_col
    AND     A, @MASK_KEY_COL
    XOR     A, @MASK_KEY_COL
    BTSS    STATUS, Z
    JMP     _KEY_VAL_MAP;

    RLC     R0
    DRSZR   R1
    JMP     _SCAN_ROWX
    JMP     _KEY_READ_NO_KEY    ;RETLW  @0
_KEY_VAL_MAP:
    MOV     R1, A               ;col
    MOV     A, R0
    XOR     A, @0xFF
    MOV     R0, A               ;row
    OR      A, R1               ;Key value
_key_read_end:
    CALL    stack_pop   
    RETURN
_KEY_READ_NO_KEY:
    BCR     rO_LED, LED_B       ;led off
    MOV     A, @0
    JMP     _key_read_end
;int key_read(void) end

key_init:
    CALL    led_init_device         ;指示灯初始化
    CALL    key_init_device
    MOV     A, @KEY_STATE_IDLE
    MOV     rKEY_STATE, A
    CLR     rKEY_VALUE_L            ;清零按键低位值
    CLR     rKEY_VALUE_H            ;清零按键高位值    
    RETURN

;设备休眠的时候，外部中断唤醒后，应该处于这个状态
key_idle:
    MOV     A, @KEY_STATE_PRESS
    MOV     rKEY_STATE, A
    RETURN
    
;void key_press(void) start 
key_press:
    CLR     rKEY_PRESS_TIME         ;清零计时器
    CLR     rKEY_INTERVAL_CNT       ;清零连击间隔计数器
    
    CALL    key_read                ;读取键值
    AND     A, @0xFF                ;使Z位变化
    BTSC    STATUS, Z               ;检测是否为0
    JMP     _key_press_end          ;为零，跳转
    MOV     A, @KEY_STATE_DEJITTER  ;不为0，按下状态->消抖状态
    MOV     rKEY_STATE, A           ;更新状态机
    MOV     A, @KEY_DEJITTER_TIME   ;设置消抖时间
    MOV     rKEY_PRESS_TIME, A
_key_press_end:
    RETURN
;void key_press(void) end   

;void key_dejitter(void) start  
key_dejitter:
    CALL    key_read                ;读取键值
    AND     A, @0xFF                ;运算使Z位变化
    BTSS    STATUS, Z               ;判断是否为0
    JMP     _DEJITTER               ;不为0，跳转
    MOV     A, @KEY_STATE_IDLE      ;为零，说明按键抖动，消抖状态->空闲状态
    MOV     rKEY_STATE, A           ;更新状态机
    JMP     _key_dejitter_end       ;退出
_DEJITTER:
    DRSZR   rKEY_PRESS_TIME         ;检测消抖时间是否为0
    JMP     _key_dejitter_end       ;不为零退出
    MOV     A, @KEY_STATE_SPRESS    ;为0，消抖状态->单击状态
    MOV     rKEY_STATE, A           ;更新状态机
_key_dejitter_end:  
    RETURN
;void key_dejitter(void) end

;void key_spress(void) start
key_spress:
    CALL    stack_push
    CALL    key_read                ;读取键值
    MOV     R0, A                   ;暂存键值
    MOV     R0, R0                  ;自赋值
    BTSS    STATUS, Z               ;检测是否为0
    JMP     _KEY_SPRESS_NEXT_STATE  ;键值有效跳转
    MOV     A, @KEY_STATE_RELEASE   ;键值无效，单击状态->释放状态
    MOV     rKEY_STATE, A           ;更新状态机
    JMP     _key_spress_end         ;返回
_KEY_SPRESS_NEXT_STATE:
    MOV     A, @KEY_STATE_DPRESS    ;键值有效，单击状态->连击状态
    MOV     rKEY_STATE, A           ;更新状态机
    MOV     A, @KEY_LPRESS_1S       ;按键在一秒钟之后进入连击状态
    MOV     rKEY_PRESS_TIME, A      ;保存时间
    MOV     A, R0                   ;
    MOV     rKEY_VALUE_L, A         ;键值存入寄存器
    CLR     rKEY_VALUE_H            ;清除按键读取标志
    MOV     A, @KEY_LPRESS_1S / @KEY_INTERVAL_TIME ;触发连击时间与连击间隔时间的商
    AND     A, @0x7F                ;去除最高位为1的情况
    MOV     rKEY_INTERVAL_CNT, A    ;赋初值给连击间隔计数器，该计数器用于计算按键按下多少秒
                                    ;实际时间为(rKEY_INTERVAL_CNT) * KEY_INTERVAL_TIME
    ;MOV        rKEY_VALUE_H, A
_key_spress_end:
    CALL    stack_pop
    RETURN

key_dpress:
    CALL    stack_push
    CALL    key_read                ;读取键值
    MOV     R0, A                   ;暂存到R0
    MOV     R0, R0                  ;自赋值
    BTSS    STATUS, Z               ;检测键值是否为0
    JMP     _KEY_DPRESS_TIME_DEC    ;不为0，跳转
    MOV     A, @KEY_STATE_RELEASE   ;为0，连击状态->释放状态
    MOV     rKEY_STATE, A           ;更新状态机
    JMP     _key_dpress_end         ;退出
_KEY_DPRESS_TIME_DEC:
    DRSZR   rKEY_PRESS_TIME         ;递减按键计时器，判断是否为0
    JMP     _key_dpress_end         ;不为0，退出
    MOV     A, @KEY_INTERVAL_TIME   ;为0，重新赋值为连击间隔时间
    MOV     rKEY_PRESS_TIME, A
    MOV     A, R0                   ;读取暂存键值
    MOV     rKEY_VALUE_L, A         ;然后赋值到键值低位寄存器
    CLR     rKEY_VALUE_H            ;清除键值高位寄存器
    INC     rKEY_INTERVAL_CNT       ;增加间隔时间计数器
    BTSC    rKEY_INTERVAL_CNT, 6    ;判断第六位是否为0
    DEC     rKEY_INTERVAL_CNT       ;如果为1，减1，为了不与KEY_READ_B值相冲突
    MOV     A, rKEY_INTERVAL_CNT    ;间隔计数器赋值到键值高位寄存器
    MOV     rKEY_VALUE_H, A
_key_dpress_end:
    CALL    stack_pop
    RETURN
    
;void key_release(void) start
key_release:
    MOV     A, @KEY_STATE_IDLE              ;释放状态->空闲状态
    MOV     rKEY_STATE, A
    BCR     rKEY_VALUE_H, KEY_READ_B        ;release时也会发送按键的键码，所以清除已读标志
    BSR     rKEY_VALUE_H, KEY_RELEASE_B     ;release标志置一
    RETURN
;void key_release(void) end

;void key_scan(void) start
key_scan:
    MOV     A, rKEY_STATE
    XOR     A, @KEY_STATE_IDLE
    BTSC    STATUS, Z
    JMP     _0
    XOR     A, @KEY_STATE_IDLE 
    
    XOR     A, @KEY_STATE_PRESS
    BTSC    STATUS, Z
    JMP     _1
    XOR     A, @KEY_STATE_PRESS 
    
    XOR     A, @KEY_STATE_DEJITTER
    BTSC    STATUS, Z
    JMP     _2
    XOR     A, @KEY_STATE_DEJITTER 
    
    XOR     A, @KEY_STATE_SPRESS
    BTSC    STATUS, Z
    JMP     _3
    XOR     A, @KEY_STATE_SPRESS
    
    XOR     A, @KEY_STATE_DPRESS
    BTSC    STATUS, Z
    JMP     _4  
    XOR     A, @KEY_STATE_DPRESS
    
    XOR     A, @KEY_STATE_RELEASE
    BTSC    STATUS, Z
    JMP     _5  
    XOR     A, @KEY_STATE_RELEASE
    JMP     _key_scan_end
_0:
    CALL    key_idle
    JMP     _key_scan_end
_1:
    CALL    key_press
    JMP     _key_scan_end
_2:
    CALL    key_dejitter
    JMP     _key_scan_end
_3:
    CALL    key_spress
    JMP     _key_scan_end
_4:
    CALL    key_dpress
    JMP     _key_scan_end
_5:
    CALL    key_release
_key_scan_end:
    RETURN
;void key_scan(void) end

;int read_key_value(void) start
;A=rKEY_VALUE_L; read H from rKEY_VALUE_H
read_key_value:
    MOV     rKEY_VALUE_L, rKEY_VALUE_L
    BTSC    STATUS, Z
    JMP     _READ_KEY_VALUE_END
    BTSC    rKEY_VALUE_H, KEY_READ_B        ;如果不为0，判断是否已读
    JMP     _READ_KEY_VALUE_END             ;如果已读，键值清零
    MOV     A, rKEY_VALUE_L
    MOV     rKEY_VAL_SAVED_L, A
    MOV     A, rKEY_VALUE_H
    MOV     rKEY_VAL_SAVED_H, A
    BSR     rKEY_VALUE_H, KEY_READ_B        ;如果未读，置为已读
    JMP     _READ_KEY_VALUE_END
_READ_KEY_VALUE_END:
    RETURN
;int read_key_value(void) end




/*********************************************************************
*  Function: Write SPI Register
*  Input:    rSPI_ADDR, rSPI_DATA_H, rSPI_DATA_L
*  Output:   None
*  static void write_spi_reg(addr, data_high, data_low)
*********************************************************************/

write_spi_reg:
    BCR     rO_SPI_SS, SPI_SS_B     ;低电平片选
    
    MOV     A, rSPI_ADDR            ;送寄存器地址
    AND     A, @0x7F                ;写数据，最高位为0
    CALL    write_spi
    NOP
    MOV     A, rSPI_DATA_H          ;送高位数据
    CALL    write_spi
    NOP
    MOV     A, rSPI_DATA_L          ;送低位数据
    CALL    write_spi
    NOP
    
    BSR     rO_SPI_SS, SPI_SS_B     ;取消片选
    
    RETURN
    
/*********************************************************************
*  Function: Read SPI Register
*  Input:   rSPI_ADDR
*  Output:  rSPI_DATA_H,rSPI_DATA_L
*  static struct {data_high, data_low} read_spi_reg(addr)
*********************************************************************/


read_spi_reg:
    BCR     rO_SPI_SS, SPI_SS_B     ;低电平片选
    
    MOV     A, rSPI_ADDR            ;送寄存器地址
    OR      A, @0x80                ;读数据，地址最高位为1
    CALL    write_spi
    NOP
    CALL    read_spi                ;读取高位数据
    MOV     rSPI_DATA_H, A          
    NOP
    CALL    read_spi                ;读取地位数据
    MOV     rSPI_DATA_L, A          
    NOP
    
    BSR     rO_SPI_SS, SPI_SS_B     ;取消片选
    
    RETURN
    
/*********************************************************************
* Function: Write SPI communication subroutine Falling edge
* Input:    A
* Output:   None
* static void write_spi(A)
*********************************************************************/

write_spi:
    CALL    stack_push
    MOV     rSPI_BUF, A                 ;带发送值放入移位寄存器
    MOV     A, @8                       ;移位计数值
    MOV     R0, A
_WRITE_BIT:
    BSR     rO_SPI_CLK, SPI_CLK_B       ;时钟高电平
    
    RLC     rSPI_BUF                    ;最高位左移放入C
    BTSS    STATUS, C                   ;检测是否为1
    JMP     _OUTPUT_0                   ;不为1，跳转输出0
    BSR     rO_SPI_MOSI, SPI_MOSI_B     ;为1，输出1
    JMP     _WRITE_BIT_DEC              ;跳转去递减计数值
_OUTPUT_0:
    BCR     rO_SPI_MOSI, SPI_MOSI_B     ;输出0
    NOP
_WRITE_BIT_DEC:
    BCR     rO_SPI_CLK, SPI_CLK_B       ;时钟拉低
    
    NOP
    DRSZR   R0                          ;判断8为数据是否全部输出
    JMP     _WRITE_BIT                  ;不为0，接着发送下一位
    NOP
    BCR     rO_SPI_MOSI, SPI_MOSI_B     ;数据位置低
    
    CALL    stack_pop
    RETURN

/*********************************************************************
* Function: Read SPI communication subroutine Falling edge
* Input:    None
* Output:   A
* SPI读的时候，没必要发送任何数据，只要有时钟就可以，之所以有的芯片
* 必须要发送数据，是因为，这些芯片的SPI是自带的功能，而不是模拟的，
* 如果要产生时钟，就必须往SBUF中写点什么，才能启动时钟
* static char read_spi(void)
*********************************************************************/

read_spi:
    CALL    stack_push
    CLR     rSPI_BUF                    ;清零移位寄存器
    MOV     A, @8                       ;移位计数值
    MOV     R0, A
_READ_BIT:
    BSR     rO_SPI_CLK, SPI_CLK_B       ;上升沿从机输出
    NOP                                 ;每个nop约166ns
    NOP
    NOP
    NOP
    BCR     rO_SPI_CLK, SPI_CLK_B       ;下降沿主机采样
    
    BTSS    rI_SPI_MISO, SPI_MISO_B     ;检测输入值是否为0
    JMP     _INPUT_0                    ;为0，跳转
    BSR     STATUS, C                   ;不为零，设置C值
    JMP     _READ_BIT_RLC                       
_INPUT_0:       
    BCR     STATUS, C                   ;为0，清零C
_READ_BIT_RLC:  
    RLC     rSPI_BUF                    ;左移移位寄存器，C移入最低位
    DRSZR   R0                          ;判断8位是否全部读入
    JMP     _READ_BIT                   ;否，继续读
    MOV     A, rSPI_BUF                 ;是，存入A寄存器，返回
    CALL    stack_pop
    RETURN


/*********************************************************************
* Function: Reset RF module
* Input:    None
* Output:   None
* static void rf_reset_module(void);
*********************************************************************/  
rf_reset_module:
    BCR     rO_RF_RESET, RF_RESET_B     ;reset 引脚置低复位
    
    MOV     A, @20                      ;延时2ms
    CALL    delay_100us
    
    BSR     rO_RF_RESET, RF_RESET_B     ;reset 引脚置高
    
    MOV     A, @50                      ;延时5ms，等待晶体稳定
    CALL    delay_100us
    RETURN
    
/*********************************************************************
*  Function: RF Initialize SubRoutine
*  Input:    None
*  Output:   None
*  public void rf_init_module(void)
*********************************************************************/
rf_init_module:
    CALL    rf_init_device
    CALL    rf_reset_module
    
    BCR     rTEST, RF_TEST_ERROR_B
    CALL    init_frame_reg
    
    MOV     A, @1
    CALL    delay_100us
    
    CALL    rf_test_initial
    BTSC    rTEST, RF_TEST_ERROR_B
    JMP     rf_init_module
    CALL    rf_enter_idle_state
    BSR     rTEST, RF_INIT_OK_B
    CALL    rf_set_comm_syncword
    RETURN
    
/**********************************************************************/
;subroutine
;static void init_frame_reg(void)
init_frame_reg:
    CALL    stack_push
    CLR     R0                  ;清零查表指针
_set_frame_reg:     
    MOV     A, R0               
    CALL    FRAME_TABLE         ;查表获取寄存器地址
    MOV     rSPI_ADDR, A        ;存储获取的寄存器地址
    XOR     A, @0xFF            ;与表格末尾值异或运算
    BTSC    STATUS, Z           ;判断是否到表末尾
    JMP     _set_frame_reg_end  ;到末尾退出
    INC     R0                  ;
    MOV     A, R0
    CALL    FRAME_TABLE         ;查表下一个数据
    MOV     rSPI_DATA_H, A      ;下一数据是寄存器高位数据
    INC     R0
    MOV     A, R0
    CALL    FRAME_TABLE         ;继续查表下一个数据
    MOV     rSPI_DATA_L, A      ;这一数据为寄存器低位数据
    CALL    write_spi_reg       ;以这三个值为参数设置RF寄存器
    INC     R0
    JMP     _set_frame_reg      ;继续设置下一个寄存器
_set_frame_reg_end:
    CALL    stack_pop
    RETURN
    
/*********************************************************************
*  test RF Register, if read==write test pass
*********************************************************************/
;subroutine
;static void rf_test_initial(void)
rf_test_initial:
    CALL    stack_push
    CLR     R0
_CHECK_DIGIT_REG:
    MOV     A, R0
    CALL    FRAME_TABLE
    MOV     rSPI_ADDR, A
    XOR     A, @0xFF
    BTSC    STATUS, Z
    JMP     _rf_test_initial_end
    CALL    read_spi_reg
    INC     R0
    MOV     A, R0
    CALL    FRAME_TABLE
    XOR     A, rSPI_DATA_H
    BTSS    STATUS, Z
    JMP     _RF_INIT_ERROR
    INC     R0
    MOV     A, R0
    CALL    FRAME_TABLE
    XOR     A, rSPI_DATA_L
    BTSS    STATUS, Z
    JMP     _RF_INIT_ERROR
    INC     R0
    JMP     _CHECK_DIGIT_REG
_RF_INIT_ERROR:
    BSR     rTEST, RF_TEST_ERROR_B
_rf_test_initial_end:
    CALL    stack_pop
    RETURN
    
/*********************************************************************
*  Function: Rewrite Communicate SyncWord
*  Input:    Commun_SyncWord1, Commun_SyncWord2,
              Commun_SyncWord3, Commun_SyncWord4
*  Output:   None
*  本系统用16bit同步字，并且允许错误位为0，只需往REG36写入数据即可
*  void rf_set_comm_syncword(void)
*********************************************************************/
rf_set_comm_syncword:
    CALL    stack_push
    CLR     R0
    MOV     A, R0
    CALL    ID_TABLE
    MOV     rSPI_DATA_H, A
    INC     R0
    MOV     A, R0
    CALL    ID_TABLE
    MOV     rSPI_DATA_L, A
    MOV     A, @SYNC_ADDR_1 ;REG36
    MOV     rSPI_ADDR, A
    CALL    write_spi_reg
/*  
    INC     R0
    MOV     A, R0
    CALL    ID_TABLE
    MOV     rSPI_DATA_H, A
    INC     R0
    MOV     A, R0
    CALL    ID_TABLE
    MOV     rSPI_DATA_L, A
    MOV     A, @SYNC_ADDR_4
    MOV     rSPI_ADDR, A
    CALL    write_spi_reg   
*/
    CALL    stack_pop
    RETURN

/*********************************************************************
*  Function: Enter TX State
*  Input:    A=CHANNEL_INDEX
*  Output:   None
*  void rf_enter_tx_state(channel)
*********************************************************************/

rf_enter_tx_state:
    CALL    stack_push
    MOV     R0, A               ;R0保存CHANNEL_INDEX
    MOV     A, @0x07
    MOV     rSPI_ADDR, A        ;配置寄存器为REG7
    MOV     A, @0x01
    MOV     rSPI_DATA_H, A      ;BIT8=1->TX_EN使能
    
    MOV     A, R0               ;读取RF频道值
    CALL    LAMP_CHANNEL_TABLE

    AND     A, @0x7F            ;BIT7=0->RX_EN禁用，TX_EN与RX_EN不能同时为1
    MOV     rSPI_DATA_L, A
    CALL    write_spi_reg       ;写入寄存器配置
    CALL    stack_pop
    RETURN
    
/*********************************************************************
*  Function: Enter RX State
*  Input:    A=CHANNEL_INDEX
*  Output:   None
*  void rf_enter_rx_state(channel)
*********************************************************************/
rf_enter_rx_state:
    CALL    stack_push
    MOV     R0, A               ;R0保存CHANNEL_INDEX
    MOV     A, @0x07            ;
    MOV     rSPI_ADDR, A        ;配置寄存器为REG7
    CLR     rSPI_DATA_H         ;清除高位寄存器
    
    MOV     A, R0               ;
    CALL    LAMP_CHANNEL_TABLE  ;读取RF频道值
    
    OR      A, @0x80            ;BIT7=1->RX_EN使能，TX_EN与RX_EN不能同时为1
    MOV     rSPI_DATA_L, A      
    CALL    write_spi_reg       ;写入寄存器配置
    CALL    stack_pop
    RETURN 
    
/*********************************************************************
*  Function: RF Enter Idle State
*  Input:    None
*  Output:   None
*  void rf_enter_idle_state(void)
*********************************************************************/
rf_enter_idle_state:
    MOV     A, @0x07
    MOV     rSPI_ADDR, A
    CLR     rSPI_DATA_H
    CLR     rSPI_DATA_L         ;TODO: 修改通道
    CALL    write_spi_reg       ;REG7, RX_EN与TX_EN都为0，进入IDLE状态
    RETURN
    
/*********************************************************************
*  Function: RF Enter Sleep State
*  Input:    None
*  Output:   None
*  void rf_enter_sleep_state(void)
*********************************************************************/
rf_enter_sleep_state:
    MOV     A, @0x23
    MOV     rSPI_ADDR, A
    CALL    read_spi_reg        ;读取REG35
    MOV     A, @0x40
    OR      rSPI_DATA_H, A      ;BIT14置1，RF将进入睡眠模式
    CALL    write_spi_reg
    RETURN
    
/*********************************************************************
*  Function: RF Enter Power Down State
*  Input:    None
*  Output:   None
*  void rf_enter_power_down_state(void)
*********************************************************************/
rf_enter_power_down_state:
    MOV     A, @0x23
    MOV     rSPI_ADDR, A
    CALL    read_spi_reg        ;读取REG35
    MOV     A, @0x80
    OR      rSPI_DATA_H, A      ;BIT15置1，RF将进入节电模式
    CALL    write_spi_reg
    RETURN  
    
    
/*********************************************************************
*  Function: Read FIFO RAM
*  Input:    None
*  Output:   RX_BUF    , Receive_Error
*  Software SPI
*  char *rf_read_fifo_ram(void)
*  impact global regs
*  success: return (rx_buf+1)
*  fail: rLAMP_COMM_STATUS/rFRAME_LOST_CNT
*********************************************************************/
rf_read_fifo_ram:
    CALL    stack_push
    MOV     A, @48D
    MOV     rSPI_ADDR, A
    CALL    read_spi_reg                    ;读取REG48寄存器
    MOV     A, rSPI_DATA_H
    AND     A, @0xC0                        ;
    BTSS    STATUS, Z                       ;判断最高两位是否值为1
    JMP     _RECV_DATA_ERROR                ;有1，则表示数据包出错
    CLR     rFRAME_LOST_CNT                 ;清零帧丢失计数器
    BCR     rLAMP_COMM_STATUS, LOST_FRAME_B ;清零帧丢失标志位
    
    BCR     rO_SPI_SS, SPI_SS_B             ;片选
    
    MOV     A, @0x32                        ;
    OR      A, @0x80
    CALL    write_spi                       ;写REG50
    NOP
    CALL    read_spi                        ;读取包数据
    MOV     rRX_DATA_LEN, A                 ;包中第一字节为包长度
    XOR     A, @NORMAL_DATA_LEN             
    BTSS    STATUS, Z                       ;判断是否与发送的包长度相同
    JMP     _RECV_DATA_ERROR                ;不相同
    BCR     rLAMP_COMM_STATUS, RECV_ERROR_B ;清除接收错误标志位
    
    MOV     A, rRX_DATA_LEN                 ;rTX_DATA_LEN == rRX_BUF[0]
    MOV     R0, A                           ;
    MOV     rRX_BUF, A
    
    MOV     A, @rRX_BUF + @1
    MOV     RAMS, A                         ;间接寻址指标，相当于指针
    MOV     R1, A
_READ_FIFO_LOOP:
    CALL    read_spi
    MOV     A, R1
    MOV     RAMS, A                         ;跟写fifo一样，rams被改写了  
    MOV     INDR, A                         ;读取数据
    INC     R1                              ;INDR等价于RAMS
    DRSZR   R0                              ;是否全部读取
    JMP     _READ_FIFO_LOOP                 ;否，继续读
_DISABLE_RF_COMM:
    BSR     rO_SPI_SS, SPI_SS_B             ;是，取消片选
    CALL    stack_pop
    RETURN
    
_RECV_DATA_ERROR:
    BSR     rLAMP_COMM_STATUS, RECV_ERROR_B ;置错误标志位
    BSR     rLAMP_COMM_STATUS, LOST_FRAME_B ;置帧丢失标志位
    INC     rFRAME_LOST_CNT                 ;增加帧接收错误计数器
    JMP     _DISABLE_RF_COMM                ;停止传输
    
/*********************************************************************
*  Function: Write FIFO RAM
*  Input:    TX_BUF
*  Output:   None
*  Software SPI
*  void rf_write_fifo_ram(tx_buf)
*********************************************************************/
rf_write_fifo_ram:
    CALL    stack_push
    BCR     rO_SPI_SS, SPI_SS_B
    
    MOV     A, @0x32
    CALL    write_spi
    
    MOV     A, rTX_BUF          ;data length == rTX_BUF[0] + 1
    ADD     A, @1
    MOV     R0, A
    MOV     A, @rTX_BUF
    MOV     RAMS, A
    MOV     R1, A
_WRITE_FIFO_LOOP:
    MOV     A, INDR
    CALL    write_spi
    INC     R1
    MOV     A, R1
    MOV     RAMS, A ;因为wirte_spi中会改写RAMS，RAMS貌似不能读，不能存储到栈中
    DRSZR   R0
    JMP     _WRITE_FIFO_LOOP
    
    BSR     rO_SPI_SS, SPI_SS_B
    CALL    stack_pop
    RETURN
    
/*********************************************************************
*  Function: Reset TX/RX FIFO
*  Input:    None
*  Output:   None
*  void rf_reset_rx_fifo(void)
*  void rf_reset_tx_fifo(void)
*********************************************************************/

rf_reset_tx_fifo:
    CALL    stack_push
    MOV     A, @0x80
    MOV     R0, A
    MOV     A, @0x00
    MOV     R1, A
    JMP     _RF_RESET_WRITE_REG
rf_reset_rx_fifo:
    CALL    stack_push
    MOV     A, @0x00
    MOV     R0, A
    MOV     A, @0x80
    MOV     R1, A
    JMP     _RF_RESET_WRITE_REG
_RF_RESET_WRITE_REG:
    MOV     A, @0x34
    MOV     rSPI_ADDR, A
    MOV     A, R0
    MOV     rSPI_DATA_H, A
    MOV     A, R1
    MOV     rSPI_DATA_L, A
    CALL    write_spi_reg
    CALL    stack_pop
    RETURN


/*********************************************************************
*  Function: Read Rolling code from fixed address
*  Input:    Rolling code address 如果是固定的就不需要这个参数
*  Output:   TX BUF
*  Rolling code 类似与芯片的序列号，只是这个芯片没有序列号，需要通过
*  软件去写，具体写哪个位置，还不确定。这个code用来标识唯一的芯片，在
*  具体应用到遥控器中，用来标识某一遥控器，使得这一遥控器跟灯绑定以后，
*  在未解绑之前，别的遥控器不能操作灯，Rolling code 作为发送的一部分
*  与功能码一起发送给灯。
*  char *rf_read_rolling_code(void)
*  返回值存储在rTX_BUF中
*********************************************************************/
rf_read_rolling_code:
    CALL    stack_push
    MOV     A, @ROLLING_CODE_ADDR_L     ;Rolling code 在rom中的地址低位
    MOV     R0, A
    MOV     A, @ROLLING_CODE_ADDR_H     ;Rolling code 在rom中的地址高位
    MOV     R1, A
    MOV     A, @rTX_BUF + @1                
    MOV     RAMS, A                     ;指向缓存
    MOV     A, @2                       ;@N_ROLLING_CODE 双字节的code有几个            
    MOV     R2, A                       ;计数器<-多少双字节Rolling code
                                        ;这个code不会超过8个字节，所以没有
                                        ;判断是否会超出BUF的存储范围

_READ_ROLLING_CODE_LOOP:
    MOV     A, R0
    MOV     PAL, A                      ;
    MOV     A, R1
    MOV     PAH, A
    BCR     STATUS, C
    RLC     PAL                         ;PAL[76543210] 
                                        ;=R0[6543210C]
    RLC     PAH                         ;PAH[543210] 
                                        ;=R1[43210C] C<-R0[7]
    MOV     A, PAH
    AND     A, @0b00011111              ;
    MOV     PAH, A                      ;结果如上
    BCR     PAL, 0
    MOVC                                ;获取低字节
    MOV     INDR, A                     ;存入rTX_BUF
    INC     RAMS                        ;指向rTX_BUF下一个字节
    
    BSR     PAL, 0                      ;置最低位，准备取高位字节
    MOVC                                ;获取高字节
    MOV     INDR, A                     ;存入rTX_BUF
    INC     RAMS                        ;指向rTX_BUF下一个字节

    INC     R0                          ;Rolling code低地址+1
    BTSC    STATUS, Z                   ;判断是否溢出
    INC     R1                          ;如果有，高位地址+1
    
    DRSZR   R2                          ;Rolling code 是否读完
    JMP     _READ_ROLLING_CODE_LOOP     ;否，继续读
    CALL    stack_pop
    RETURN
    

/*********************************************************************
*  Function: Pack data
*  Input:    None
*  Output:   None
*  void rf_pack_tx_data(void)
*  Packed data format:  1Byte Pack length 
*                       + 4Bytes Rolling Code 
*                       + 1Byte KEY_VALUE_H
*                       + 1Byte KEY_VALUE_L 
*                       + 1Byte RESERVED DATA
*********************************************************************/  
rf_pack_tx_data:
    CALL    rf_read_rolling_code    ;rTX_BUF[1,2,3,4]
    
    MOV     A, @rTX_BUF + @5
    MOV     RAMS, A
    MOV     A, rKEY_VAL_SAVED_L
    MOV     INDR, A                 ;rTX_BUF[5] = rKEY_VAL_SAVED_H
    
    INC     RAMS
    MOV     A, rKEY_VAL_SAVED_H
    MOV     INDR, A                 ;rTX_BUF[6] = rKEY_VAL_SAVED_L
    
    INC     RAMS
    MOV     A, @RF_RESERVED_DATA
    MOV     INDR, A                 ;rTX_BUF[7] = @RF_RESERVED_DATA
    
    MOV     A, @7
    MOV     rTX_BUF, A              ;rTX_BUF[0] = 7 数据包的长度
    
    RETURN
    
/*********************************************************************
*  Function: rf send data
*  Input:    None
*  Output:   None
*  void rf_transmit_data(void)
*********************************************************************/  
rf_transmit_data:
    CALL    rf_reset_tx_fifo            ;Clear TX FIFO
    CALL    rf_pack_tx_data             ;打包待发送的数据
    CALL    rf_write_fifo_ram           ;数据写入FIFO中
    MOV     A, @CHANNEL_INDEX           ;设置RF频道
    CALL    rf_enter_tx_state           ;进入发送状态
_TX_WAIT_PKT:
    MOV     A, @2
    CALL    delay_100us
    BTSS    rI_RF_PKT, RF_PKT_B         ;发送完成？PKT高电平表示发送完成
    JMP     _TX_WAIT_PKT                ;否，继续等待
    CALL    rf_enter_idle_state         ;进入IDLE状态
    RETURN

/*********************************************************************
*  Function: rf receive data
*  Input:    None
*  Output:   None
*  flag rf_receive_data(void)
*  返回是否成功标志 C==1 fail C==0 success
*********************************************************************/
rf_start_rx_timer:
    MOV     A, @30
    MOV     rRX_TIMER, A
    RETURN 
rf_read_rx_timer:
    MOV     A, rRX_TIMER
    RETURN
    
rf_receive_data:
    CALL    stack_push
    CALL    rf_reset_rx_fifo            ;Clear RX FIFO
    MOV     A, @CHANNEL_INDEX           ;设置RF频道
    CALL    rf_enter_rx_state           ;进入接收状态
    CALL    rf_start_rx_timer           ;启动接收定时器
_RX_WAIT_PKT:
    MOV     A, @2
    CALL    delay_100us
    CALL    rf_read_rx_timer            ;读接收定时器
    MOV     R0, A                       ;
    MOV     R0, R0
    BTSC    STATUS, Z                   ;判断定时器是否为0
    JMP     _RECV_TIME_OUT              ;为0，跳到超时处理
    BTSS    rI_RF_PKT, RF_PKT_B         ;检测PKT引脚是否为高
    JMP     _RX_WAIT_PKT                ;低电平表示接受未完成，继续接收
    CALL    rf_read_fifo_ram            ;接收完成，读取FIFO数据
    BCR     STATUS, C                   ;C标志位清零
    BTSS    rLAMP_COMM_STATUS, RECV_ERROR_B ;判断数据是否正确
    JMP     _rf_receive_data_end        ;正确，跳转
_RECV_TIME_OUT:
    BSR     STATUS, C                   ;超时或数据有误置C位
_rf_receive_data_end:
    CALL    rf_enter_idle_state         ;进入空闲状态
    CALL    stack_pop
    RETURN






stack_init:
    MOV     A, @STACK_TOP   ;
    MOV     rSTACK_SP, A    ;初始化栈顶
    RETURN
    
stack_push:
    PUSH        
    MOV     A, STATUS
    MOV     rSTACK_STATUS, A
    
    MOV     A, RAMS
    MOV     rSTACK_RAMS, A
    
    MOV     A, rSTACK_SP
    MOV     RAMS, A
    
    DEC     rSTACK_SP
    DEC     RAMS
    MOV     A, rSTACK_RAMS
    MOV     INDR, A
    
    DEC     rSTACK_SP
    DEC     RAMS
    MOV     A, R0
    MOV     INDR, A

    DEC     rSTACK_SP
    DEC     RAMS
    MOV     A, R1
    MOV     INDR, A

    DEC     rSTACK_SP
    DEC     RAMS
    MOV     A, R2
    MOV     INDR, A
    
    MOV     A, rSTACK_STATUS
    MOV     STATUS, A
    POP     
    RETURN
    
stack_pop:
    PUSH                        ;PUSH A
    MOV     A, STATUS
    MOV     rSTACK_STATUS, A    
    
    MOV     A, rSTACK_SP
    MOV     RAMS, A
    
    MOV     A, INDR
    MOV     R2, A
    INC     rSTACK_SP
    
    INC     RAMS
    MOV     A, INDR
    MOV     R1, A
    INC     rSTACK_SP
    
    INC     RAMS
    MOV     A, INDR
    MOV     R0, A
    INC     rSTACK_SP
    
    INC     RAMS
    MOV     A, INDR
    MOV     RAMS, A
    INC     rSTACK_SP
    
    MOV     A, rSTACK_STATUS
    MOV     STATUS, A
    POP                             ;POP A
    RETURN  
    

delay_100us:                        ;delay 100us needs 1205 cycles in 12MHz
    call    stack_push
    MOV     R0, A
    MOV     R0, R0
    BTSC    STATUS, Z
    JMP     _delay_100us_end
_DELAY_100US_N_TIMES:
    MOV     A, @140                 ;2 cycles;Push&pop 占用了84个周期
    MOV     R1, A                   ;2 cycles
_DELAY_100US_LOOP:
    WDTC
    DRSZR   R1                      ;4 cycles
    JMP     _DELAY_100US_LOOP       ;4 cycles
    
    DRSZR   R0                      ;4 cycles
    JMP     _DELAY_100US_N_TIMES    
_delay_100us_end:
    CALL    stack_pop
    RETURN  


    
