#include "msp.h"
#include <math.h>
#include <driverlib.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

//#define TESTING

#define PI 3.14159265358979323846

// master clock frequency = 3 MHz
#define MCLK_FREQ 12000000

// number of samples to average for GPS reads
#define GPS_SAMPLES 3

// number of seconds between recalculations while en route and far from target (minimum distance calculation >= 20 meters)
#define EN_ROUTE_RECALCULATION_TIME_FAR 5

// number of seconds between recalculations while en route and close to target (minimum distance calculation < 20 meters)
#define EN_ROUTE_RECALCULATION_TIME_CLOSE 1

// number of repetitions over which to average for motor calibration routine
#define MOTOR_CALIBRATION_REPS 2

// divider for correction motor offset value
#define CORRECT_SPEED_DIVIDER 2

// multiplier for forward speed
#define FWD_SPEED_MULTIPLIER 3

// number of times to correct direction during start_fwd()
#define FWD_CORRECTIONS 5

// calibration time
#define CALIBRATION_TIME 10

// compass module register macros
#define COMPASS_ADDR 0x0D
#define COMPASS_X_LSB 0x00
#define COMPASS_STAT1 0x06
#define COMPASS_CTL1 0x09
#define COMPASS_CTL2 0x0A
#define COMPASS_SET_RESET 0x0B

// compass DRDY pin = P4.0
#define COMPASS_DRDY_PORT P4IN
#define COMPASS_DRDY_MASK BIT0

// ultrasonic sensor trig pin P6.2
#define TRIG_PORT GPIO_PORT_P6
#define TRIG_PIN GPIO_PIN2

// ultrasonic sensor echo pin P6.3
#define ECHO_PORT GPIO_PORT_P6
#define ECHO_PIN GPIO_PIN3

// state machine state macros
#define IDLE 0
#define GETTING_COORDS 1
#define ORIENTING 2
#define EN_ROUTE 3
#define AVOIDING_OBSTACLE 4
#define ARRIVED 5
#define SENDING_DATA 6
#define SETUP 7
#define RESET 8

/////////////////////////////// GLOBAL VARIABLES ////////////////////////////////

// rover's current state
// 0 = idle, 1 = getting coords, 2 = orienting, 3 = en route, 4 = avoiding obstacle, 5 = arrived, 6 = sending data, 7 = setup, 8 = reset
int state = RESET;

// rover boolean values
bool compass_calibrated = false;    // true if the compass has been calibrated
bool motors_calibrated = false;     // true if motor PWM has been calibrated
bool moving = false;                // true if rover is currently moving
bool force_stop = false;            // true if force stop command is received from controller
bool force_setup = false;           // true if force setup command is received from controller
bool isBlocked = false;             // true if there is an obstacle within 10 cm
bool connection_lost = false;
bool go_home = false;

// last coordinate data from GPS
float current_lat = 0;
float current_lon = 0;

// coordinates of target location
float target_lat = 0;
float target_lon = 0;

// home coordinates
float home_lat = 0;
float home_lon = 0;

// heading to target location
float target_heading = 0;

// compass XYZ read buffer
int XYZ_buf_index = 0;
uint8_t XYZ_buf[6];

// for receiving characters through peripherals
char RX;

// NACK interrupt on compass i2c module (EUSCIB0) will set this flag
bool compass_nack_received = false;

// true if compass i2c module (EUSCIB0) is waiting to receive a single byte
bool compass_receiving_single = false;

// compass variables
int16_t XYZ_readings[3];    // concatenated 16-bit magnetometer values for X, Y, Z, respectively
float heading = 0;          // last calculated heading, not necessarily up to date. Refreshed using get_compass_data()

// calibration offset values for compass
float compass_offset[3] = {0, 0, 0};

// calibration scale values for compass
float compass_scale[3] = {1, 1, 1};

// calibration offset values for motors
float motor_scale[2] = {1, 1};

// motor pwm value for turning in place
uint16_t motor_spin_pwm = 5000;

// timer count value for delay functions
uint16_t TA1_count = 0;

// timer count values for recalculating while en route
uint16_t TA2_ms = 0;
uint16_t TA2_seconds = 0;

// I2C configuration
const eUSCI_I2C_MasterConfig i2cConfig =
{
     EUSCI_B_I2C_CLOCKSOURCE_SMCLK,         // SMCLK clock source
     12000000,                              // SMCLK = 12 MHz
     EUSCI_B_I2C_SET_DATA_RATE_100KBPS,     // Desired i2c clock 100 kHz
     0,                                     // No byte counter threshold
     EUSCI_B_I2C_NO_AUTO_STOP               // No auto stop
};


// Timer A configuration for 1ms tick period
const Timer_A_UpModeConfig ta_config_1ms =
{
     TIMER_A_CLOCKSOURCE_SMCLK,             // SMCLK clock source (12 MHz)
     TIMER_A_CLOCKSOURCE_DIVIDER_1,         // divider = 1 -> TA1CLK == SMCLK
     12000,                                  // 1 ms tick period
     TIMER_A_TAIE_INTERRUPT_DISABLE,        // Disable Timer interrupt
     TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE ,   // Enable CCR0 interrupt
     TIMER_A_SKIP_CLEAR                     // Clear value
};

// Timer A configuration for 20ms tick period
const Timer_A_UpModeConfig ta_config_20ms =
{
     TIMER_A_CLOCKSOURCE_SMCLK,             // SMCLK clock source (12 MHz)
     TIMER_A_CLOCKSOURCE_DIVIDER_4,         // divider = 4 -> TA1CLK = SMCLK/4 = 3MHz
     60000,                                  // 20 ms tick period
     TIMER_A_TAIE_INTERRUPT_DISABLE,        // Disable Timer interrupt
     TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE ,   // Enable CCR0 interrupt
     TIMER_A_SKIP_CLEAR                     // Clear value
};

// Timer A configuration for 4 ticks per second
const Timer_A_UpModeConfig ta_config_4ps =
{
     TIMER_A_CLOCKSOURCE_SMCLK,             // SMCLK clock source (12 MHz)
     TIMER_A_CLOCKSOURCE_DIVIDER_48,        // divider = 48 -> TA1CLK == SMCLK/48 = 250kHz
     62500,                                 // 250,000 / 62500 = 4 ticks per second
     TIMER_A_TAIE_INTERRUPT_DISABLE,        // Disable Timer interrupt
     TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE ,   // Enable CCR0 interrupt
     TIMER_A_SKIP_CLEAR                     // Clear value
};

////////////////////////////// FUNCTION PROTOTYPES //////////////////////////////

/*******************************************
*
* NAME:    delay_s
*
* INPUT:   n (int)
*
* OUTPUT:  N/A
*
* PURPOSE: delay number of seconds specified
*          by int n
*
*******************************************/
void delay_s(int n);

/*******************************************
*
* NAME:    delay_ms
*
* INPUT:   n (int)
*
* OUTPUT:  N/A
*
* PURPOSE: delay number of ms specified
*          by int n
*
*******************************************/
void delay_ms(long int n);

/*******************************************
*
* NAME:    delay_us
*
* INPUT:   n (int)
*
* OUTPUT:  N/A
*
* PURPOSE: delay number of us specified
*          by int n
*
*******************************************/
void delay_us(long int n);

/*******************************************
*
* NAME:    UART1_XBEE_init
*
* INPUT:   N/A
*
* OUTPUT:  N/A
*
* PURPOSE: initialize UART module for XBEE
*
*******************************************/
void UART1_XBEE_init();

/*******************************************
*
* NAME:    UART1_XBEE_send_char
*
* INPUT:   char c
*
* OUTPUT:  N/A
*
* PURPOSE: transmit a single character through
*          UART used for XBEE
*
*******************************************/
void UART1_XBEE_send_char(char c);

/*******************************************
*
* NAME:    UART1_XBEE_receive_char
*
* INPUT:   N/A
*
* OUTPUT:  character received through UART
*
* PURPOSE: receive a single character through
*          UART used for XBEE
*
*******************************************/
char UART1_XBEE_receive_char();

/*******************************************
*
* NAME:    UART2_GPS_init
*
* INPUT:   N/A
*
* OUTPUT:  N/A
*
* PURPOSE: initialize UART communicating with
*          GPS module
*
*******************************************/
void UART2_GPS_init();

/*******************************************
*
* NAME:    UART2_GPS_send_char
*
* INPUT:   char c, the character to be transmitted
*
* OUTPUT:  N/A
*
* PURPOSE: transmit a character via UART
*
*******************************************/
void UART2_GPS_send_char(char c);

/*******************************************
*
* NAME:    UART2_GPS_receive_char
*
* INPUT:   N/A
*
* OUTPUT:  c, character received through UART
*
* PURPOSE: receive a single character through UART module
*
*******************************************/
char UART2_GPS_receive_char();

void UART2_GPS_set_10HZ();

/*******************************************
*
* NAME:    comp_strings
*
* INPUT:   two strings and the length to compare
*
* OUTPUT:  1 if match, 0 if non-match
*
* PURPOSE: compare two strings
*
*******************************************/
int comp_strings(char * a, char * b, int len);

/*******************************************
*
* NAME:    UART2_GPS_detect_str
*
* INPUT:   string to detect and its length
*
* OUTPUT:  1 if found, 0 if not found or
*          there was an error
*
* PURPOSE: detect a sequence of characters
*          in incoming UART data
*
*******************************************/
int UART2_GPS_detect_str(char * str, int len);

/*******************************************
*
* NAME:    UART2_GPS_ignore
*
* INPUT:   number of incoming UART characters
*          to ignore
*
* OUTPUT:  1 if there were no errors, 0 otherwise
*
* PURPOSE: ignore a given amount of incoming UART
*          characters
*
*******************************************/
int UART2_GPS_ignore(int n);

/*******************************************
*
* NAME:    receive_GPS_GLL
*
* INPUT:   N/A
*
* OUTPUT:  1 if successful, 0 if unsuccessful (receive error)
*
* PURPOSE: receive latitude and longitude data from GPS
*          module through UART RX and store in local variables,
*          and update current_lat and current_lon
*
*******************************************/
int receive_GPS_GLL();

/*******************************************
*
* NAME:    NMEA_lon_to_degrees
*
* INPUT:   longitude in NMEA format and direction
*
* OUTPUT:  longitude converted to floating point value
*
* PURPOSE: convert NMEA longitude to floating point
*
*******************************************/
float NMEA_lon_to_degrees(char lon[11], char dir);

/*******************************************
*
* NAME:    NMEA_lat_to_degrees
*
* INPUT:   latitude in NMEA format and direction
*
* OUTPUT:  latitude converted to floating point value
*
* PURPOSE: convert NMEA latitude to floating point
*
*******************************************/
float NMEA_lat_to_degrees(char lat[10], char dir);

float get_distance(float lat, float lon, float target_lat, float target_lon);

/*******************************************
*
* NAME:    distance_to_target
*
* INPUT:   longitude and latitude of target point
*
* OUTPUT:  distance in meters. If there is an error,
*          function returns -1
*
* PURPOSE: receive current location from GPS module and
*          calculate distance to target coordinate
*          using haversine formula:
*
*
*******************************************/
float distance_to_target();

/*******************************************
*
* NAME:    bearing_to_target
*
* INPUT:   N/A
*
* OUTPUT:  bearing in degrees. If there is an error,
*          function returns -1
*
* PURPOSE: receive current location from GPS module and
*          calculate bearing to target coordinate
*
*******************************************/
float bearing_to_target();

/*******************************************
*
* NAME:    TA0_config_PWM
*
* INPUT:   N/A
*
* OUTPUT:  N/A
*
* PURPOSE: configure TA0 as PWM output
*
*******************************************/
void TA0_config_PWM();

/*******************************************
*
* NAME:    set_TA0
*
* INPUT:   number of which TA0 PWM output to set
*          and value to set it to
*
* OUTPUT:  N/A
*
* PURPOSE: set the specified PWM output
*
*******************************************/
void set_TA0(int n, int val);

/*******************************************
*
* NAME:    stop_all_TA0
*
* INPUT:   N/A
*
* OUTPUT:  N/A
*
* PURPOSE: stop all PWM outputs on TA0
*
*******************************************/
void stop_all_TA0();

/*******************************************
*
* NAME:    UCB0_i2c_compass_write
*
* INPUT:   memory address of register to write to
*          and data to write
*
* OUTPUT:  N/A
*
* PURPOSE: write to one of the compass module's
*          registers
*
*******************************************/
void UCB0_i2c_compass_write(uint8_t memaddr, char data);

/*******************************************
*
* NAME:    UCB0_i2c_compass_firstWrite
*
* INPUT:   memory address of register to write to
*          and data to write
*
* OUTPUT:  N/A
*
* PURPOSE: Used on the first i2c write. Writes to one of the compass module's
*          registers
*
*******************************************/
void UCB0_i2c_compass_firstWrite(uint8_t memaddr, char data);

/*******************************************
*
* NAME:    UCB0_i2c_compass_init
*
* INPUT:   N/A
*
* OUTPUT:  N/A
*
* PURPOSE: initialize the UART module for the i2c
*          and set up the compass' configuration
*          registers
*
*******************************************/
void UCB0_i2c_compass_init();

/*******************************************
*
* NAME:    UCB0_i2c_compass_read
*
* INPUT:   memory address of register to read from
*
* OUTPUT:  the data received from the register
*
* PURPOSE: read from one of the compass module's
*          registers
*
*******************************************/
char UCB0_i2c_compass_read(uint8_t memaddr);

/*******************************************
*
* NAME:    compass_soft_reset
*
* INPUT:   N/A
*
* OUTPUT:  N/A
*
* PURPOSE: reset the compass if it becomes
*          stuck
*
*******************************************/
void compass_soft_reset();

/*******************************************
*
* NAME:    compass_read_XYZ
*
* INPUT:   N/A
*
* OUTPUT:  x, y, and z values stored in
*          global variable XYZ_readings
*
* PURPOSE: write to one of the compass module's
*          registers
*
*******************************************/
void compass_read_XYZ();

/*******************************************
*
* NAME:    get_compass_data
*
* INPUT:   N/A
*
* OUTPUT:  heading stored in global variable
*          heading
*
* PURPOSE: read compass data and convert to heading
*          in degrees
*
*******************************************/
int get_compass_data();

/*******************************************
*
* NAME:    clock_config
*
* INPUT:   N/A
*
* OUTPUT:  N/A
*
* PURPOSE: set up the MSP432's clocks:
*               SMCLK = MCLK = DCOCLK = 3MHz, ACLK = REFO
*
*******************************************/
void clock_config();

/*******************************************
*
* NAME:    peripheral_config
*
* INPUT:   N/A
*
* OUTPUT:  N/A
*
* PURPOSE: configure the peripherals used:
*          - UART for XBEE and GPS
*          - I2C for compass
*          - TA0 PWM for motors
*
*******************************************/
void peripheral_config();

/*******************************************
*
* NAME:    get_target_coordinates
*
* INPUT:   N/A
*
* OUTPUT:  N/A
*
* PURPOSE: set up the MSP432's clocks:
*               SMCLK = MCLK = DCOCLK = 3MHz, ACLK = REFO
*
*******************************************/
void get_target_coordinates();

void compass_calibrate();

/*******************************************
*
* NAME:    start_fwd
*
* INPUT:   direction to move in
*
* OUTPUT:  N/A
*
* PURPOSE: if there is not obstacle in the way,
*          gradually increase the speed of TA0
*          PWM's corresponding to forward motion
*          until desired speed is reached, correcting
*          direction periodically. If rover
*          becomes blocked before reaching target
*          speed, rover stops and function exits.
*
*******************************************/
void start_fwd(float dir, int quick);

/*******************************************
*
* NAME:    timed_stop
*
* INPUT:   number of milliseconds over which to
*          slow down to a stop
*
* OUTPUT:  N/A
*
* PURPOSE: gradually decrease the speed of TA0
*          PWM's corresponding to forward motion
*          until rover has stopped
*
*******************************************/
void timed_stop(long int ms);

float calculate_bearing(float lat_a, float lon_a, float lat_b, float lon_b);

float get_current_heading();

void turn_in_place(float degrees);

void correct_direction(float dir);

void face_direction(float dir);

float change_in_heading(float h1, float h2);

bool calibrate_motor_speed();

void calibrate_motors();

char get_drift_dir(float h1, float h2);

void TA1_config();

void TA2_config();

void TA3_config();

void T32_config();

void ultrasonic_sensor_init();

void adc_init();

////////////////////////////// INTERRUPT HANDLERS ///////////////////////////////

void EUSCIB0_IRQHandler(void) {
    uint_fast16_t status;

    status = MAP_I2C_getEnabledInterruptStatus(EUSCI_B0_BASE);
    MAP_I2C_clearInterruptFlag(EUSCI_B0_BASE, status);

    if (status & EUSCI_B_I2C_NAK_INTERRUPT) {
        //MAP_I2C_masterReceiveMultiByteStop(EUSCI_B0_BASE);
        //while (MAP_I2C_masterIsStopSent(EUSCI_B0_BASE) == EUSCI_B_I2C_SENDING_STOP);
        MAP_I2C_masterSendStart(EUSCI_B0_BASE);
        MAP_I2C_disableInterrupt(EUSCI_B0_BASE, EUSCI_B_I2C_RECEIVE_INTERRUPT0);
        status &= ~EUSCI_B_I2C_RECEIVE_INTERRUPT0;
        compass_nack_received = true;
    }
    if (status & EUSCI_B_I2C_RECEIVE_INTERRUPT0) {
        // reading XYZ from compass
        if (XYZ_buf_index == 5) {
            MAP_I2C_disableInterrupt(EUSCI_B0_BASE, EUSCI_B_I2C_RECEIVE_INTERRUPT0 + EUSCI_B_I2C_NAK_INTERRUPT);
            MAP_I2C_masterReceiveMultiByteStop(EUSCI_B0_BASE);
            XYZ_buf[XYZ_buf_index++] = MAP_I2C_masterReceiveMultiByteNext(EUSCI_B0_BASE);
            // ADDED
            while (!(EUSCI_B0->IFG & EUSCI_B_IFG_STPIFG));
        }
        else {
            XYZ_buf[XYZ_buf_index++] = MAP_I2C_masterReceiveMultiByteNext(EUSCI_B0_BASE);
        }
    }
}

void EUSCIA1_IRQHandler(void) {
    uint_fast16_t status;

    status = MAP_UART_getEnabledInterruptStatus(EUSCI_A1_BASE);
    MAP_UART_clearInterruptFlag(EUSCI_A1_BASE, status);

    // no commands will be acknowledged while in SETUP mode.
    // if A is received while in IDLE mode, listen for coordinates from controller.
    // if 0 is received while in IDLE mode, send sensor data to controller.
    // if S is received while not in IDLE or ORIENTING mode, stop rover and put in IDLE mode.
    // if D is received, stop rover and run setup procedure.
    // if G is received, continue route to last target location
    // if R is received and both compass and motors are calibrated, send a C, otherwise go to RESET mode.
    if (status & EUSCI_A_UART_RECEIVE_INTERRUPT) {
        // disable interrupts until ISR exits and clear receive interrupt flag
        MAP_UART_disableInterrupt(EUSCI_A1_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
        MAP_UART_clearInterruptFlag(EUSCI_A1_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);


#ifdef TESTING
        // FOR TESTING ONLY!

        if (EUSCI_A1->RXBUF == 'F') {
            state = 80;
        }
        else if (EUSCI_A1->RXBUF == 'S') {
            force_stop = true;
            state = 90;
        }
        else if (EUSCI_A1->RXBUF == 'R') {
            state = 20;
        }
        else if (EUSCI_A1->RXBUF == 'L') {
            state = 30;
        }
        else if (EUSCI_A1->RXBUF == 'D') {
            state = 10;
        }
        else if (EUSCI_A1->RXBUF == 'C') {
            state = ORIENTING;
        }
        else if (EUSCI_A1->RXBUF == 'n') {
            state = 100;
        }
        else if (EUSCI_A1->RXBUF == 'e') {
            state = 110;
        }
        else if (EUSCI_A1->RXBUF == 's') {
            state = 120;
        }
        else if (EUSCI_A1->RXBUF == 'w') {
            state = 130;
        }
        MAP_UART_enableInterrupt(EUSCI_A1_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);

#else

        if (EUSCI_A1->RXBUF == 'X') {
            MAP_Timer32_setCount(TIMER32_0_BASE, 468750);
            MAP_Timer32_startTimer(TIMER32_0_BASE, 0);
            UART1_XBEE_send_char('X');
            if (connection_lost) connection_lost = false;
        }
        if (state != SETUP) {
            if (EUSCI_A1->RXBUF == 'D') {
                if (moving) {
                    timed_stop(2000);
                }
                if (state != IDLE) {
                    force_setup = true;
                }
                state = SETUP;
            }
            else if (state == IDLE) {
                if (EUSCI_A1->RXBUF == 'A') {
                    state = GETTING_COORDS;
                }
                else if (EUSCI_A1->RXBUF == '0') {
                    state = SENDING_DATA;
                }
                else if (EUSCI_A1->RXBUF == 'G') {
                    state = EN_ROUTE;
                }
                else if (EUSCI_A1->RXBUF == 'C') {
                    state = 100;
                }
                else if (EUSCI_A1->RXBUF == 'F') {
                    state = 80;
                }
                else if (EUSCI_A1->RXBUF == 'H') {
                    target_lat = home_lat;
                    target_lon = home_lon;
                    force_stop = true;
                    go_home = true;
                }
            }
            else {
                if (EUSCI_A1->RXBUF == 'S') {
                    force_stop = true;
                    state = IDLE;
                }
            }
        }
        MAP_UART_enableInterrupt(EUSCI_A1_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
#endif
    }

}

void TA1_0_IRQHandler(void) {

    MAP_Timer_A_clearCaptureCompareInterrupt(TIMER_A1_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0);

    TA1_count++;
}

void TA2_0_IRQHandler(void) {

    MAP_Timer_A_clearCaptureCompareInterrupt(TIMER_A2_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0);

    TA2_ms++;
    if (TA2_ms == 1000) {
        TA2_ms = 0;
        TA2_seconds++;
    }
}

void TA3_0_IRQHandler(void) {

    MAP_Interrupt_disableMaster();
    MAP_Timer_A_clearCaptureCompareInterrupt(TIMER_A3_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0);

    // change TA3 divider to 4 for trigger pulse and echo measurement
    MAP_Timer_A_configureUpMode(TIMER_A3_BASE, &ta_config_20ms);
    TA3R = 0;
    MAP_Timer_A_startCounter(TIMER_A3_BASE, TIMER_A_UP_MODE);

    int x;

    // send 10us pulse on TRIG pin
    P6->OUT |= TRIG_PIN;
    TA3R = 0;
    while (TA3R < 30); // 10us
    P6->OUT &= ~TRIG_PIN;

    // time the echo pulse length
    while (!(P6->IN & ECHO_PIN));
    TA3R = 0;
    while ( (P6->IN & ECHO_PIN) && (TA3R < 7200) );
    x = TA3R;

    // calculate distance and determine whether there is an obstacle within range
    if (x/3/58 < 40) {
        if (moving) {
            stop_all_TA0();
        }
        if (!isBlocked) {
            isBlocked = true;
        }
        if (state == EN_ROUTE || state == 80) {
            UART1_XBEE_send_char('N');
            //state = AVOIDING_OBSTACLE;
        }
    }
    else {
        if (isBlocked) {
            isBlocked = false;
            if (state == EN_ROUTE || state == 80) {
                //UART1_XBEE_send_char('Y');
            }
        }
    }

    // change TA3 divider back to 48
    MAP_Timer_A_configureUpMode(TIMER_A3_BASE, &ta_config_4ps);
    MAP_Timer_A_startCounter(TIMER_A3_BASE, TIMER_A_UP_MODE);
    MAP_Interrupt_enableMaster();

    MAP_Timer_A_clearCaptureCompareInterrupt(TIMER_A3_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0);
    TA3R = 0;
}

void T32_INT1_IRQHandler(void) {
    MAP_Timer32_clearInterruptFlag(TIMER32_0_BASE);

    target_lat = home_lat;
    target_lon = home_lon;

    force_stop = true;
    connection_lost = true;

    MAP_Timer32_haltTimer(TIMER32_0_BASE);
}

////////////////////////////// FUNCTION DEFINITIONS /////////////////////////////

void delay_s(int n) {
    //UART1_XBEE_send_char('S');
//    UART1_XBEE_send_char('D');


    int i;
    //delay 1 second n times
    for (i = n; i > 0; i--) {
        if (force_stop || force_setup) break;
        delay_ms(1000);
    }

//    UART1_XBEE_send_char('X');

}

void delay_ms(long int n) {
//    UART1_XBEE_send_char('m');
//    UART1_XBEE_send_char('D');


    TA1_count = 0;
    TA1R = 0;
    MAP_Timer_A_startCounter(TIMER_A1_BASE, TIMER_A_UP_MODE);
    while (TA1_count < n && !force_stop && !force_setup);
    MAP_Timer_A_stopTimer(TIMER_A1_BASE);

//    UART1_XBEE_send_char('X');

}

void delay_us(long int n) {
    TA1R = 0;
    MAP_Timer_A_startCounter(TIMER_A1_BASE, TIMER_A_UP_MODE);
    while ((MAP_Timer_A_getCounterValue(TIMER_A3_BASE) < 12*n) && !force_stop && !force_setup);
    MAP_Timer_A_stopTimer(TIMER_A1_BASE);
}

void UART1_XBEE_init() {
    EUSCI_A1->CTLW0 = UCSWRST; // reset mode for configuration
    EUSCI_A1->CTLW0 |= UCSSEL__SMCLK; // select smclk as clock source

    EUSCI_A1->MCTLW |= UCOS16; // set OS16 bit for oversampling mode
    EUSCI_A1->BRW = 78; // 12,000,000/9600/16 = 78.125
    EUSCI_A1->MCTLW |= (2 << 4) & 0x00F0; // set UCBRF0, 0.125*16 = 2
    EUSCI_A1->MCTLW |= 0x0000; // set UCBRS0 (value taken from table 22-5 in user manual)

    P2SEL0 |= BIT2 + BIT3; // P2.3 configured as TX and P2.2 configured as RX

    EUSCI_A1->CTLW0 &= ~UCSWRST; // clear reset mode bit to enable receiver
}

void UART1_XBEE_send_char(char c) {
    while ( (EUSCI_A1->IFG & UCTXIFG) == 0 ); // wait for transmit buffer to be ready
    EUSCI_A1->TXBUF = c;
}

void UART1_XBEE_send_heading() {
    int tmp, h;
    h = (int)heading;
    tmp = h / 100;
    UART1_XBEE_send_char('0' + tmp);
    h -= 100*tmp;
    tmp = h/10;
    UART1_XBEE_send_char('0' + tmp);
    h -= 10*tmp;
    tmp = h;
    UART1_XBEE_send_char('0' + tmp);
}

char UART1_XBEE_receive_char() {
    while ( !(EUSCI_A1->IFG & UCRXIFG) ); // wait for character to be received
    // if there is a receive error, return 0
    if (EUSCI_A1->STATW & EUSCI_A_STATW_RXERR) {
        return 0;
    }
    char c = EUSCI_A1->RXBUF; // read character from receive buffer
    return c;
}

int UART1_XBEE_detect_str(char * str, int len) {
    unsigned int i;
    char tmp; // to store received characters
    char window[10]; // store the last len characters read
    // wait for first letter in string to be detected, then start filling window
    while (1) {
        tmp = UART1_XBEE_receive_char();
        if (!tmp) return 0; // receive error, exit
        if (tmp == str[0]) break; // first character detected, break
    }
    // fill window
    window[0] = tmp;
    for (i = 1; i < len; i++) {
        tmp = UART1_XBEE_receive_char();
        if (!tmp) return 0; // receive error, exit
        window[i] = tmp;
    }
    long int n = 10000;
    while (1) {
        if ( comp_strings(window, str, len) ) {
            return 1; // full match, exit
        }
        else {
            if (n == 0) return 0; // timeout, exit
            tmp = UART1_XBEE_receive_char();
            if (!tmp) return 0; // receive error, exit
            // shift window
            for (i = 0; i < len-1; i++) {
                window[i] = window[i+1];
            }
            window[len-1] = tmp;
        }
        n--;
    }
}

void UART2_GPS_init() {
    EUSCI_A2->CTLW0 = UCSWRST; // reset mode for configuration
    EUSCI_A2->CTLW0 |= UCSSEL__SMCLK; // select smclk as clock source

    EUSCI_A2->MCTLW |= UCOS16; // set OS16 bit for oversampling mode
    EUSCI_A2->BRW = 78; // 12,000,000/9600/16 = 78.125
    EUSCI_A2->MCTLW |= (2 << 4) & 0x00F0; // set UCBRF0, 0.125*16 = 2
    EUSCI_A2->MCTLW |= 0x0000; // set UCBRS0 (value taken from table 22-5 in user manual)

    P3SEL0 |= BIT2 + BIT3; // P3.3 configured as TX and P3.2 configured as RX

    MAP_GPIO_setAsInputPin(GPIO_PORT_P6, GPIO_PIN0);    // 6.0 is PPS input
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P6, GPIO_PIN1);   // 6.1 is EN output

    P6->OUT |= BIT1;    // enable GPS module

    EUSCI_A2->CTLW0 &= ~UCSWRST; // clear reset mode bit to enable receiver

    UART2_GPS_set_10HZ();
}

void UART2_GPS_send_char(char c) {
    while ( (UCA2IFG & UCTXIFG) == 0 ); // wait for transmit buffer to be ready
    UCA2TXBUF = c;
}

char UART2_GPS_receive_char() {
    while ( !MAP_UART_getInterruptStatus(EUSCI_A2_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT) ); // wait for character to be received
    // if there is a receive error, return 0
    if (EUSCI_A2->STATW & EUSCI_A_STATW_RXERR) {
        return 0;
    }
    char c = EUSCI_A2->RXBUF; // read character from receive buffer
    return c;
}

int comp_strings(char * a, char * b, int len) {
    unsigned int i;
    for (i = 0; i < len; i++) {
        if ( a[i] != b[i] ) return 0;
    }
    return 1;
}

int UART2_GPS_detect_str(char * str, int len) {
    unsigned int i;
    char tmp; // to store received characters
    char window[10]; // store the last len characters read
    // wait for first letter in string to be detected, then start filling window
    while (1) {
        tmp = UART2_GPS_receive_char();
        if (!tmp) return 0; // receive error, exit
        if (tmp == str[0]) break; // first character detected, break
    }
    // fill window
    window[0] = tmp;
    for (i = 1; i < len; i++) {
        tmp = UART2_GPS_receive_char();
        if (!tmp) return 0; // receive error, exit
        window[i] = tmp;
    }
    long int n = 10000;
    while (1) {
        if ( comp_strings(window, str, len) ) {
            return 1; // full match, exit
        }
        else {
            if (n == 0) return 0; // timeout, exit
            tmp = UART2_GPS_receive_char();
            if (!tmp) return 0; // receive error, exit
            // shift window
            for (i = 0; i < len-1; i++) {
                window[i] = window[i+1];
            }
            window[len-1] = tmp;
        }
        n--;
    }
}

int UART2_GPS_ignore(int n) {
    unsigned int i;
    char tmp;
    for (i = n; i > 0; i--) {
        tmp = UART2_GPS_receive_char();
        if (!tmp) return 0; // receive error, exit
    }
    return 1;
}

void UART2_GPS_set_10HZ() {
    // B5 62 06 08 06 00 64 00 01 00 01 00 7A 12 B5 62 06 08 00 00 0E 30
    UART2_GPS_send_char(0xB5);
    UART2_GPS_send_char(0x62);
    UART2_GPS_send_char(0x06);
    UART2_GPS_send_char(0x08);
    UART2_GPS_send_char(0x06);
    UART2_GPS_send_char(0x00);
    UART2_GPS_send_char(0x64);
    UART2_GPS_send_char(0x00);
    UART2_GPS_send_char(0x01);
    UART2_GPS_send_char(0x00);
    UART2_GPS_send_char(0x01);
    UART2_GPS_send_char(0x00);
    UART2_GPS_send_char(0x7A);
    UART2_GPS_send_char(0x12);
    UART2_GPS_send_char(0xB5);
    UART2_GPS_send_char(0x62);
    UART2_GPS_send_char(0x06);
    UART2_GPS_send_char(0x08);
    UART2_GPS_send_char(0x00);
    UART2_GPS_send_char(0x00);
    UART2_GPS_send_char(0x0E);
    UART2_GPS_send_char(0x30);
}

int receive_GPS_GLL() {

//    UART1_XBEE_send_char('G');

    char NMEA_latitude[10], NMEA_longitude[11];
    char lat_dir, lon_dir, GPS_status;
    char tmp; // to store received character
    uint8_t i, samples = 0;

    current_lat = 0;
    current_lon = 0;

    MAP_Interrupt_disableMaster();          // NEW

    while (1) {
        if (samples == GPS_SAMPLES) break;
        tmp = EUSCI_A2->RXBUF; // read rx buffer to clear error flags

        // wait until "GLL" is found, meaning lat/lon data coming next
        if ( UART2_GPS_detect_str("GLL", 3) == 0 ) {
            return 0; // couldn't detect string, exit
        }
        // ignore the comma. Latitude in NMEA format will follow.
        if ( UART2_GPS_ignore(1) == 0 ) return 0; // error, exit
        // next 12 characters will be the latitude data (ddmm.mmmmm,D) <- D stands for direction (N,S)
        for (i = 10; i > 0; i--) {
            tmp = UART2_GPS_receive_char();
            if (!tmp) return 0; //error, exit
            NMEA_latitude[10-i] = tmp;
        }
        // ignore the comma
        if ( UART2_GPS_ignore(1) == 0 ) return 0; // error, exit
        // store the direction
        tmp = UART2_GPS_receive_char();
        if (!tmp) return 0; //error, exit
        lat_dir = tmp;
        // ignore the next character, which will be a comma preceding the longitude data in NMEA format
        if ( UART2_GPS_ignore(1) == 0 ) return 0; // error, exit
        // next 13 characters will be the longitude data (dddmm.mmmmm,D) <- D stands for direction (E,W)
        for (i = 11; i > 0; i--) {
            tmp = UART2_GPS_receive_char();
            if (!tmp) return 0; //error, exit
            NMEA_longitude[11-i] = tmp;
        }
        // ignore the comma
        if ( UART2_GPS_ignore(1) == 0 ) return 0; // error, exit
        // store the direction
        tmp = UART2_GPS_receive_char();
        if (!tmp) return 0; //error, exit
        lon_dir = tmp;
        // ignore the next 11 characters, which will be the time (,hhmmss.ss,). The status character will follow, indicating data valid or invalid (A = valid, V = invalid)
        if ( UART2_GPS_ignore(11) == 0 ) return 0; // error, exit
        //read the status character
        tmp = UART2_GPS_receive_char();
        if (!tmp) return 0; //error, exit
        GPS_status = tmp;
        if (GPS_status == 'A') {
            current_lat += NMEA_lat_to_degrees(NMEA_latitude, lat_dir);
            current_lon += NMEA_lon_to_degrees(NMEA_longitude, lon_dir);
            samples++;
        }
    }
    current_lat /= samples;
    current_lon /= samples;

    MAP_Interrupt_enableMaster();  // new
    UART1_XBEE_send_char('X');

    return 1;
}

float NMEA_lon_to_degrees(char lon[11], char dir) {
    float lon_fp;               // resulting fp value of longitude
    int lon_deg;                // degrees portion of longitude
    float lon_min;              // minutes portion of longitude
    unsigned int i;             // NMEA longitude index
    unsigned int k;             // k = weighting factor

    lon_deg = 100*(lon[0] - '0') + 10*(lon[1] - '0') + (lon[2] - '0');

    lon_min = 0;
    for (i = 3, k = 10; i < 5; i++, k /= 10) {
        lon_min += k*(lon[i] - '0');
    }
    i++;
    for (k = 10; i < 11; i++, k *= 10) {
        lon_min += (lon[i] - '0')/(float)k;
    }

    lon_fp = lon_deg + lon_min/60;

    // if longitude value out of range, return 0
    if ( (lon_fp < 0) || (lon_fp > 180) ) {
        return 0;
    }

    //if longitude direction is west, make longitude value negative
    if (dir == 'W') {
        lon_fp *= -1;
    }
    else {
        if (dir != 'E') {
            return 0; // longitude direction invalid, return 0
        }
    }

    return lon_fp;
}

float NMEA_lat_to_degrees(char lat[10], char dir) {
    float lat_fp;               // resulting fp value of latitude
    int lat_deg;                // degrees portion of latitude
    float lat_min;              // minutes portion of latitude
    unsigned int i;             // NMEA latitude index
    unsigned int k;             // k = weighting factor

    lat_deg = 10*(lat[0] - '0') + (lat[1] - '0');

    lat_min = 0;
    for (i = 2, k = 10; i < 4; i++, k /= 10) {
        lat_min += k*(lat[i] - '0');
    }
    i++;
    for (k = 10; i < 10; i++, k *= 10) {
        lat_min += (lat[i] - '0')/(float)k;
    }

    lat_fp = lat_deg + lat_min/60;

    // if latitude value out of range, return 0
    if ( (lat_fp < 0) || (lat_fp > 90) ) {
        return 0;
    }

    // if latitude direction is south, make latitude value negative
    if (dir == 'S') {
        lat_fp *= -1;
    }
    else {
        if (dir != 'N') {
            return 0; // latitude direction invalid, return 0
        }
    }

    return lat_fp;
}

float get_distance(float lat, float lon, float target_lat, float target_lon) {
    float earth_rad = 6371e3;

    lat *= PI/180;
    lon *= PI/180;
    target_lat *= PI/180;
    target_lon *= PI/180;

    float a, c, d;

    // a = sin^2(dlat/2) + cos(lat)*cos(t_lat)*sin^2(dlon/2)
    a = sin((target_lat - lat)/2)*sin((target_lat-lat)/2) + cos(lat)*cos(target_lat)*sin((target_lon-lon)/2)*sin((target_lon-lon)/2);

    c = 2*atan2(sqrt(a), sqrt(1-a));

    d = earth_rad * c;

    target_lat /= PI/180;
    target_lon /= PI/180;

    return d;
}

float distance_to_target() {
    receive_GPS_GLL();  // get current location
    return get_distance(current_lat, current_lon, target_lat, target_lon);
}

float bearing_to_target() {
    receive_GPS_GLL(); // get current location from GPS module
    return calculate_bearing(current_lat, current_lon, target_lat, target_lon);
}

void TA0_config_PWM() {
    P2OUT &= ~BIT4 & ~BIT5 & ~BIT6 & ~BIT7;  // initialize all PWM pins to low
    P2DIR |= BIT4 | BIT5 | BIT6 | BIT7;      // configure as outputs
    P2SEL0 |= BIT4 | BIT5 | BIT6 | BIT7;     // select TA0 function
    TA0CTL &= ~BIT5 & ~BIT4; // clear mode control bits
    TA0CTL |= TACLR; // clear Timer 0
    TA0CCR0 = 65535; // PWM period
    TA0CCTL1 = OUTMOD_7; // CCR1 set/reset
    TA0CCTL2 = OUTMOD_7; // CCR2 set/reset
    TA0CCTL3 = OUTMOD_7; // CCR3 set/reset
    TA0CCTL4 = OUTMOD_7; // CCR4 set/reset
    TA0CCR1 = 0; // initially not moving
    TA0CCR2 = 0; // initially not moving
    TA0CCR3 = 0; // initially not moving
    TA0CCR4 = 0; // initially not moving
    TA0CTL = TASSEL_2 + MC_1 + BIT7 + TACLR; // SMCLK, up mode, divider = 4, clear TAR
}

void set_TA0(int n, int val) {
    switch (n) {
        case 1:
            TA0CCR1 = val;
            break;
        case 2:
            TA0CCR2 = val;
            break;
        case 3:
            TA0CCR3 = val;
            break;
        case 4:
            TA0CCR4 = val;
            break;
        default:
            break;
    }
}

void stop_all_TA0() {
    int i;
    for (i = 1; i <= 4; i++) {
        set_TA0(i, 0);
    }
    moving = false;
}

void compass_write(uint8_t memaddr, uint8_t data) {
    EUSCI_B0->CTLW0 |= (EUSCI_B_CTLW0_TR | EUSCI_B_CTLW0_TXSTT);
    //Wait for trans flag
    while (!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG));
    EUSCI_B0->TXBUF = (uint8_t)memaddr; //Send Register to read addr
    while (!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG));
    EUSCI_B0->TXBUF = (uint8_t)data;
    while (!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG));
    EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TXSTP;
    while (!(EUSCI_B0->IFG & EUSCI_B_IFG_STPIFG));
    EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_SWRST;
    EUSCI_B0->CTLW0 &= ~EUSCI_B_CTLW0_SWRST;
}

void UCB0_i2c_compass_write(uint8_t memaddr, char data) {
    while (MAP_I2C_masterIsStopSent(EUSCI_B0_BASE) == EUSCI_B_I2C_SENDING_STOP);
    // enable NACK interrupt
    MAP_I2C_enableInterrupt(EUSCI_B0_BASE, EUSCI_B_I2C_NAK_INTERRUPT);
    // send start, slave address + R/W bit, and a garbage byte to flush the buffer
    if (!MAP_I2C_masterSendMultiByteStartWithTimeout(EUSCI_B0_BASE, memaddr, 1000)) {
        UCB0_i2c_compass_write(memaddr, data);
        return;
    }
    if (compass_nack_received) {
        compass_nack_received = false;
        UCB0_i2c_compass_write(memaddr, data);
        return;
    }
    if (!MAP_I2C_masterSendMultiByteNextWithTimeout(EUSCI_B0_BASE, memaddr, 1000)) {
        UCB0_i2c_compass_write(memaddr, data);
        return;
    }
    if (compass_nack_received) {
        compass_nack_received = false;
        UCB0_i2c_compass_write(memaddr, data);
        return;
    }
    // send data to be written
    if (!MAP_I2C_masterSendMultiByteFinishWithTimeout(EUSCI_B0_BASE, data, 1000)) {
        UCB0_i2c_compass_write(memaddr, data);
        return;
    }
    if (compass_nack_received) {
        compass_nack_received = false;
        UCB0_i2c_compass_write(memaddr, data);
        return;
    }
    // disable NACK interrupt
    MAP_I2C_disableInterrupt(EUSCI_B0_BASE, EUSCI_B_I2C_NAK_INTERRUPT);
}

void UCB0_i2c_compass_firstWrite(uint8_t memaddr, char data) {
    while (MAP_I2C_masterIsStopSent(EUSCI_B0_BASE) == EUSCI_B_I2C_SENDING_STOP);
    // enable NACK interrupt
    MAP_I2C_enableInterrupt(EUSCI_B0_BASE, EUSCI_B_I2C_NAK_INTERRUPT);
    // send start, slave address + R/W bit
    if (!MAP_I2C_masterSendMultiByteStartWithTimeout(EUSCI_B0_BASE, memaddr, 1000)) {
        UCB0_i2c_compass_write(memaddr, data);
        return;
    }
    if (compass_nack_received) {
        compass_nack_received = false;
        UCB0_i2c_compass_write(memaddr, data);
        return;
    }
    // send data to be written
    if (!MAP_I2C_masterSendMultiByteFinishWithTimeout(EUSCI_B0_BASE, data, 1000)) {
        UCB0_i2c_compass_write(memaddr, data);
        return;
    }
    if (compass_nack_received) {
        compass_nack_received = false;
        UCB0_i2c_compass_write(memaddr, data);
        return;
    }
    // disable NACK interrupt
    MAP_I2C_disableInterrupt(EUSCI_B0_BASE, EUSCI_B_I2C_NAK_INTERRUPT);
}

void compass_init() {
    MAP_I2C_initMaster(EUSCI_B0_BASE, &i2cConfig);          // master mode, i2c, synchronous, smclk, 100kHz
    MAP_I2C_setSlaveAddress(EUSCI_B0_BASE, COMPASS_ADDR);
    MAP_I2C_setMode(EUSCI_B0_BASE, EUSCI_B_I2C_TRANSMIT_MODE);
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P1, GPIO_PIN6 | GPIO_PIN7, GPIO_PRIMARY_MODULE_FUNCTION);
    MAP_I2C_enableModule(EUSCI_B0_BASE);                    // enable i2c module
    MAP_Interrupt_setPriority(INT_EUSCIB0, 0x01);           // set to high priority, only higher priority is XBEE receive
    MAP_I2C_clearInterruptFlag(EUSCI_B0_BASE, EUSCI_B_I2C_TRANSMIT_INTERRUPT0 + EUSCI_B_I2C_RECEIVE_INTERRUPT0);
    MAP_I2C_enableInterrupt(EUSCI_B0_BASE, EUSCI_B_I2C_STOP_INTERRUPT);
    MAP_Interrupt_enableInterrupt(INT_EUSCIB0);

    MAP_GPIO_setAsInputPin(GPIO_PORT_P4, GPIO_PIN0);    // DRDY pin as input,
    UCB0_i2c_compass_firstWrite(COMPASS_CTL2, 0xC1);         // soft reset, rolling ptr enabled, DRDY int enabled
    compass_write(COMPASS_SET_RESET, 0x01);    // recommended setting in datasheet
    compass_write(COMPASS_CTL1, 0x01);         // OSR = 512, FSR = 2 Gauss, ODR = 10 Hz, continuous mode

}

void UCB0_i2c_compass_init() {
    MAP_I2C_initMaster(EUSCI_B0_BASE, &i2cConfig);          // master mode, i2c, synchronous, smclk, 100kHz
    MAP_I2C_setSlaveAddress(EUSCI_B0_BASE, COMPASS_ADDR);
    MAP_I2C_setMode(EUSCI_B0_BASE, EUSCI_B_I2C_TRANSMIT_MODE);
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P1, GPIO_PIN6 | GPIO_PIN7, GPIO_PRIMARY_MODULE_FUNCTION);
    MAP_I2C_enableModule(EUSCI_B0_BASE);                    // enable i2c module
    MAP_I2C_clearInterruptFlag(EUSCI_B0_BASE, EUSCI_B_I2C_TRANSMIT_INTERRUPT0 + EUSCI_B_I2C_RECEIVE_INTERRUPT0);
    MAP_Interrupt_enableInterrupt(INT_EUSCIB0);

    MAP_GPIO_setAsInputPin(GPIO_PORT_P4, GPIO_PIN0);    // DRDY pin as input,
    UCB0_i2c_compass_firstWrite(COMPASS_CTL2, 0xC1);         // soft reset, rolling ptr enabled, DRDY int enabled
    UCB0_i2c_compass_write(COMPASS_SET_RESET, 0x01);    // recommended setting in datasheet
    UCB0_i2c_compass_write(COMPASS_CTL1, 0x01);         // OSR = 512, FSR = 2 Gauss, ODR = 10 Hz, continuous mode

}

char UCB0_i2c_compass_read(uint8_t memaddr) {
    // make sure last transaction completed
    while (MAP_I2C_masterIsStopSent(EUSCI_B0_BASE) == EUSCI_B_I2C_SENDING_STOP);
    MAP_I2C_masterSendMultiByteStart(EUSCI_B0_BASE, memaddr);
    MAP_I2C_masterSendMultiByteNext(EUSCI_B0_BASE, memaddr);
    // wait for transmission to complete
    while (!MAP_I2C_getInterruptStatus(EUSCI_B0_BASE, EUSCI_B_I2C_TRANSMIT_INTERRUPT0));

    // ADDED
    EUSCI_B0->STATW &= ~EUSCI_B_IFG_TXIFG;

    // send re-start and enable receive interrupt
    compass_receiving_single = true;
    MAP_I2C_masterReceiveStart(EUSCI_B0_BASE);

    //ADDED
    while((EUSCI_B0->CTLW0 & EUSCI_B_CTLW0_TXSTT));

    MAP_I2C_masterReceiveMultiByteStop(EUSCI_B0_BASE);

    MAP_I2C_enableInterrupt(EUSCI_B0_BASE, EUSCI_B_I2C_RECEIVE_INTERRUPT0);
    while (compass_receiving_single == 1);
    return RX;

}

void compass_soft_reset() {
    UCB0_i2c_compass_write(COMPASS_CTL2, 0xC1);         // soft reset, rolling ptr enabled, DRDY int enabled
    UCB0_i2c_compass_write(COMPASS_SET_RESET, 0x01);    // recommended setting in datasheet
    UCB0_i2c_compass_write(COMPASS_CTL1, 0x01);         // OSR = 512, FSR = 2 Gauss, ODR = 10 Hz, continuous mode
}

void compass_read_XYZ() {
    XYZ_buf_index = 0;
    // make sure last transaction completed
    while (MAP_I2C_masterIsStopSent(EUSCI_B0_BASE) == EUSCI_B_I2C_SENDING_STOP);
    // send start condition and register address of X_LSB
    MAP_I2C_masterSendMultiByteStart(EUSCI_B0_BASE, COMPASS_X_LSB);
    MAP_I2C_masterSendMultiByteNext(EUSCI_B0_BASE, COMPASS_X_LSB);
    // wait for transmission to complete
    while (!MAP_I2C_getInterruptStatus(EUSCI_B0_BASE, EUSCI_B_I2C_TRANSMIT_INTERRUPT0));
    // send re-start and enable receive interrupt
    XYZ_buf[0] = EUSCI_B0->RXBUF;
    MAP_I2C_enableInterrupt(EUSCI_B0_BASE, EUSCI_B_I2C_RECEIVE_INTERRUPT0);
    MAP_I2C_masterReceiveStart(EUSCI_B0_BASE);
    // wait for all compass output registers to be read

    while (XYZ_buf_index < 6);

    XYZ_readings[0] = ( ((int16_t)XYZ_buf[0]) & 0x00FF ) | ( (((int16_t)XYZ_buf[1]) << 8) & 0xFF00 );
    XYZ_readings[1] = ( ((int16_t)XYZ_buf[2]) & 0x00FF ) | ( (((int16_t)XYZ_buf[3]) << 8) & 0xFF00 );
    XYZ_readings[2] = ( ((int16_t)XYZ_buf[4]) & 0x00FF ) | ( (((int16_t)XYZ_buf[5]) << 8) & 0xFF00 );
}

int get_compass_data() {

//    UART1_XBEE_send_char('C');

    int i = 0;
    // if data not ready, reset the compass
    if ( !(COMPASS_DRDY_PORT & COMPASS_DRDY_MASK) ) {
        compass_read_XYZ();
    }
    // wait for compass data to be ready
    while ( !(COMPASS_DRDY_PORT & COMPASS_DRDY_MASK) );
    // read X, Y, and Z compass values into XYZ_readings
    compass_read_XYZ();
    // add offset values from calibration
    for (i = 0; i < 2; i++) {
        XYZ_readings[i] -= compass_offset[i];
        XYZ_readings[i] *= compass_scale[i];
    }
    // calculate heading from compass readings
    if (XYZ_readings[0] > 0) {
        heading = 90 - atan((float)XYZ_readings[1]/(float)XYZ_readings[0])*180/PI;
    }
    else if (XYZ_readings[0] < 0){
        heading = 270 - atan((float)XYZ_readings[1]/(float)XYZ_readings[0])*180/PI;
    }
    else {
        if (XYZ_readings[1] > 0) {
            heading = 0;
        }
        else {
            heading = 180;
        }
    }

    // account for declination
    heading *= -1;
    heading -= 5.9;
    heading = heading < 0 ? heading + 360 : heading;

    // S: x = -1935, y = 595
    // N: x = 67, y = 3202
    // S: x = -2787, y = 952
    // W: x = -147, y = 355

//    UART1_XBEE_send_char('X');

    return 1;
}

void clock_config() {
    CS->KEY = CS_KEY_VAL;                                       // Unlock CS module for register access
    CS->CTL0 = 0;                                               // Reset tuning parameters
    MAP_CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_12);         // Set DCO to 12MHz (nominal, center of 8-16MHz range)
    CS->CTL1 = CS_CTL1_SELA_2 |                                 // Select ACLK = REFO
               CS_CTL1_SELS_3 |                                 // SMCLK = DCO
               CS_CTL1_SELM_3;                                  // MCLK = DCO
    CS->KEY = 0;                                                // Lock CS module from unintended accesses
}

void peripheral_config() {
    UART2_GPS_init();
    UART1_XBEE_init();
    UCB0_i2c_compass_init();
    TA0_config_PWM();
    adc_init();
    ultrasonic_sensor_init();
    P6DIR |= BIT1;
    P6OUT |= BIT1;
}

void get_target_coordinates() {
    char target_latitude[9];
    char target_longitude[10];
    int i;

    MAP_Timer32_setCount(TIMER32_0_BASE, 100000000);

    MAP_Interrupt_disableMaster();

    // first get latitude
    for (i = 0; i < 9; i++) {
        UART1_XBEE_send_char('n');
        target_latitude[i] = UART1_XBEE_receive_char();

    }

    //then get longitude
    for (i = 0; i < 10; i++) {
        UART1_XBEE_send_char('n');
        target_longitude[i] = UART1_XBEE_receive_char();
    }

    target_lat = (float)atof(target_latitude);
    target_lon = (float)atof(target_longitude);

    MAP_Timer32_setCount(TIMER32_0_BASE, 468750);

    MAP_Interrupt_enableMaster();
    UART1_XBEE_send_char('X');  //NEW

}

void compass_calibrate() {
    uint16_t ii = 0, jj = 0, sample_count = 0;
    int16_t mag_max[3] = {-32767, -32767, -32767}, mag_min[3] = {32767, 32767, 32767};
    int32_t mag_scale[3] = {0, 0, 0};
    float avg_rad;

    // shoot for predefined number of seconds for calibration time
    sample_count = 10*CALIBRATION_TIME; // 10 Hz ODR
    stop_all_TA0();
    set_TA0(1, motor_spin_pwm);
    set_TA0(4, motor_spin_pwm);
    for (ii = 0; ii < sample_count; ii++) {
        get_compass_data();
        for (jj = 0; jj < 2; jj++) {
            if(XYZ_readings[jj] > mag_max[jj]) mag_max[jj] = XYZ_readings[jj];
            if(XYZ_readings[jj] < mag_min[jj]) mag_min[jj] = XYZ_readings[jj];
        }
        delay_ms(100);
    }
    stop_all_TA0();

    // Get hard iron correction
    compass_offset[0]  = (mag_max[0] + mag_min[0])/2;  // get average x mag bias in counts
    compass_offset[1]  = (mag_max[1] + mag_min[1])/2;  // get average y mag bias in counts
    //compass_offset[2]  = (mag_max[2] + mag_min[2])/2;  // get average z mag bias in counts

    // Get soft iron correction estimate
    mag_scale[0]  = (mag_max[0] - mag_min[0])/2;  // get average x axis max chord length in counts
    mag_scale[1]  = (mag_max[1] - mag_min[1])/2;  // get average y axis max chord length in counts
    //mag_scale[2]  = (mag_max[2] - mag_min[2])/2;  // get average z axis max chord length in counts

    avg_rad = mag_scale[0] + mag_scale[1];
    avg_rad /= 2.0;

    compass_scale[0] = avg_rad/((float)mag_scale[0]);
    compass_scale[1] = avg_rad/((float)mag_scale[1]);
    //compass_scale[2] = avg_rad/((float)mag_scale[2]);

    compass_calibrated = true;

}

void start_fwd(float dir, int quick) {
    int i;
    int correct_interval = FWD_SPEED_MULTIPLIER*motor_spin_pwm/FWD_CORRECTIONS;
    stop_all_TA0();
    moving = true;
    for (i = 3200; i <= FWD_SPEED_MULTIPLIER*motor_spin_pwm; i++) {
        set_TA0(1, i*motor_scale[0]);
        set_TA0(3, i*motor_scale[1]);
        if ( isBlocked || force_stop || force_setup) {
            stop_all_TA0();
            moving = false;
            break;
        }
        if (i % correct_interval == 0) {
            correct_direction(dir);
        }
        if (!quick) __delay_cycles(300);
    }
    if (TA0CCR1 != 0) moving = true;
    else moving = false;
}

void timed_stop(long int ms) {
    int dec = TA0CCR1/ms;
    int i = TA0CCR1;
    if (i > 0) {
        for (; i >= 0; i -= dec) {
            set_TA0(1, i);
            set_TA0(3, TA0CCR1);
            delay_ms(1);
            if (isBlocked || force_stop || force_setup) break;
        }
    }
    stop_all_TA0();
}

float calculate_bearing(float lat_a, float lon_a, float lat_b, float lon_b) {

    float x, y;
    float bearing;

    // convert all latitudes and longitutes to radians
    lat_a *= PI/180;
    lon_a *= PI/180;
    lat_b *= PI/180;
    lon_b *= PI/180;

    y = sin(lon_b - lon_a) * cos(lat_b);
    x = cos(lat_a)*sin(lat_b) - sin(lat_a)*cos(lat_b)*cos(lon_b - lon_a);

    bearing = atan2(y,x);
    bearing *= 180/PI; // convert to degrees

    bearing = bearing < 0 ? bearing + 360 : bearing;

    return bearing;
}

float get_current_heading() {
    get_compass_data();
    return heading;
}

void turn_in_place(float degrees) {
    float t_heading;
    t_heading = get_current_heading() + degrees;
    t_heading = t_heading < 0 ? t_heading + 360 : t_heading;
    face_direction(t_heading);
}

void correct_direction(float dir) {
    float heading_dif, tmp;
    float degrees = dir - get_current_heading();
    degrees = degrees < 0 ? degrees + 360 : degrees;
    if ( (degrees > 1 && degrees < 359) ) {
        heading_dif = change_in_heading(heading, dir);
        if (degrees < 180) {
            set_TA0(2, 0);
            set_TA0(1, TA0CCR1 + motor_spin_pwm/CORRECT_SPEED_DIVIDER);
            while (1) {
                if (isBlocked || force_stop || force_setup) break;
                else {
                    tmp = change_in_heading(get_current_heading(), dir);
                    if (tmp > heading_dif) break;
                    heading_dif = tmp;
                }
            }
            if (isBlocked || force_stop || force_setup) stop_all_TA0();
            else set_TA0(1, TA0CCR1 - motor_spin_pwm/CORRECT_SPEED_DIVIDER);
        }
        else {
            set_TA0(4, 0);
            set_TA0(3, TA0CCR3 + motor_spin_pwm/CORRECT_SPEED_DIVIDER);
            while (1) {
                if (isBlocked || force_stop || force_setup) break;
                else {
                    tmp = change_in_heading(get_current_heading(), dir);
                    if (tmp > heading_dif || change_in_heading(heading, dir) < 5) break;
                    heading_dif = tmp;
                }
            }
            if (isBlocked || force_stop || force_setup) stop_all_TA0();
            else set_TA0(3, TA0CCR3 - motor_spin_pwm/CORRECT_SPEED_DIVIDER);
        }
    }
}

void face_direction(float dir) {

//    UART1_XBEE_send_char('F');
//    UART1_XBEE_send_char('D');

    // no need to check for obstacles while turning
    MAP_Timer_A_disableCaptureCompareInterrupt(TIMER_A3_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0);

    float degrees, heading_dif, tmp;
    degrees = dir - get_current_heading();
    degrees = degrees < 0 ? degrees + 360 : degrees;
    stop_all_TA0();
    if ( (degrees > 5 && degrees < 354) ) {
        heading_dif = change_in_heading(heading, dir);
        if (degrees > 180) {
            set_TA0(2, motor_spin_pwm);
            set_TA0(3, motor_spin_pwm);
        }
        else {
            set_TA0(1, motor_spin_pwm);
            set_TA0(4, motor_spin_pwm);
        }

        while (1) {
            if (force_stop || force_setup) break;
            else {
                tmp = change_in_heading(get_current_heading(), dir);
                if (tmp > heading_dif || change_in_heading(heading, dir) < 5) break;
                heading_dif = tmp;
            }
        }

        stop_all_TA0();
        if ((change_in_heading(get_current_heading(), dir) > 10) && !force_stop && !force_setup) {
            delay_ms(10);
            face_direction(dir);
        }
    }

    // re-enable ultrasonic sensor
    MAP_Timer_A_enableCaptureCompareInterrupt(TIMER_A3_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0);

//    UART1_XBEE_send_char('X');

}

void go_direction(float dir) {
    face_direction(dir);
    start_fwd(dir, 0);
}

float change_in_heading(float h1, float h2) {
    float degrees;
    degrees = h2 - h1;
    degrees = degrees < 0 ? degrees + 360 : degrees;
    degrees = degrees > 180? abs(degrees - 360) : degrees;
    return degrees;
}

bool calibrate_motor_speed() {
    float tmp_heading, degrees_turned;
    int no_turn_count = 0;  // counts number of consecutive times change in heading is less than 5
    int i, reps;            // reps is the number of repetitions over which to average PWM calibration value
    motor_spin_pwm = 0;
    for (reps = 0; reps < MOTOR_CALIBRATION_REPS; reps++) {
        tmp_heading = get_current_heading();
        for (i = 3200; i < 65000; i++) {
            // for each repetition, change spin direction.
            if (reps % 2) {
                set_TA0(2, 0);
                set_TA0(3, 0);
                set_TA0(1, i);
                set_TA0(4, i);
            }
            else {
                set_TA0(1, 0);
                set_TA0(4, 0);
                set_TA0(2, i);
                set_TA0(3, i);
            }
            // periodically check whether the current PWM value is enough to turn the rover in place by 25 degrees in 250 ms (100 degrees / sec)
            if (i % 250 == 0) {
                // get current heading, then wait 250 ms and calculate the change in heading
                tmp_heading = get_current_heading();
                delay_ms(500);
                degrees_turned = change_in_heading(get_current_heading(), tmp_heading);
                if (degrees_turned > 35) break;
                // if the rover has turned less than 5 degrees 10 consecutive times while the PWM is above 10000, it is probably stuck or low on battery. Save the
                // current PWM value but return false to indicate the motors were not successfully calibrated.
                else if (i > 10000) {
                    if (degrees_turned < 5) {
                        no_turn_count++;
                        if (no_turn_count == 10) {
                            stop_all_TA0();
                            motor_spin_pwm = 5000;
                            motors_calibrated = false;
                            return false;
                        }
                    }
                    else {
                        no_turn_count = 0;
                    }
                }
            }
            //delay_us(5);
        }
        motor_spin_pwm += i;
        delay_ms(100);  // short delay between spin direction changes
    }
    stop_all_TA0();
    motor_spin_pwm /= MOTOR_CALIBRATION_REPS;
    motors_calibrated = true;
    return true;
}

void calibrate_motors() {
    float tmp;
    int motor_offset[2] = {0, 0};

    tmp = get_current_heading();

    start_fwd(tmp, 0);

    while (1) {
        delay_ms(500);
        get_compass_data();
        if (change_in_heading(tmp, heading) > 1) {
            if (get_drift_dir(tmp, heading) == 'R') {
                if (motor_offset[0] > 0) {
                    motor_offset[0] -= 250;
                    TA0CCR1 -= 250;
                }
                else {
                    motor_offset[1] += 250;
                    TA0CCR3 += 250;
                }
            }
            else {
                if (motor_offset[1] > 0) {
                    motor_offset[1] -= 250;
                    TA0CCR3 -= 250;
                }
                else {
                    motor_offset[0] += 250;
                    TA0CCR1 += 250;
                }
            }
        }
        else break;
        tmp = get_current_heading();
    }
    if (motor_offset[0] > motor_offset[1]) {
        motor_scale[0] = 1 + (float)((float)motor_offset[0] - (float)motor_offset[1]) / (float)12000;
    }
    else {
        motor_scale[1] = 1 + (float)((float)motor_offset[1] - (float)motor_offset[0]) / (float)12000;
    }
    timed_stop(3000);
}

char get_drift_dir(float h1, float h2) {
    float degrees;
    degrees = h2 - h1;
    degrees = degrees < 0 ? degrees + 360 : degrees;
    if (degrees > 180) return 'R';
    else return 'L';
}

void send_sensor_data() {
    uint16_t sensor_data[3];
    char sensor[3][5];
    int i, j;
    MAP_UART_disableInterrupt(EUSCI_A1_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);

    MAP_ADC14_enableConversion();
    MAP_ADC14_toggleConversionTrigger();
    while ( !(MAP_ADC14_getInterruptStatus() & ADC_INT2) );
    MAP_ADC14_getMultiSequenceResult(sensor_data);

    for (i = 0; i < 3; i++) {
        for (j = 0; j < 5; j++) {
            sensor[i][j] = sensor_data[i] % 10 + '0';
            sensor_data[i] /= 10;
        }
    }

    for (i = 0; i < 3; i++) {
       for (j = 4; j >= 0; j--) {
           while (UART1_XBEE_receive_char() != 'n');
           UART1_XBEE_send_char(sensor[i][j]);
           sensor_data[i] /= 10;
       }
   }


    MAP_UART_clearInterruptFlag(EUSCI_A1_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
    MAP_UART_enableInterrupt(EUSCI_A1_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
}

void TA1_config() {
    MAP_Timer_A_configureUpMode(TIMER_A1_BASE, &ta_config_1ms);
    MAP_Interrupt_setPriority(INT_TA1_0, 0x05);
    MAP_Timer_A_enableCaptureCompareInterrupt(TIMER_A1_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0);
    MAP_Interrupt_enableInterrupt(INT_TA1_0);
    MAP_Timer_A_stopTimer(TIMER_A1_BASE);
}

void TA2_config() {
    MAP_Timer_A_configureUpMode(TIMER_A2_BASE, &ta_config_1ms);
    MAP_Interrupt_setPriority(INT_TA2_0, 0x05);
    MAP_Timer_A_enableCaptureCompareInterrupt(TIMER_A2_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0);
    MAP_Interrupt_enableInterrupt(INT_TA2_0);
    MAP_Timer_A_stopTimer(TIMER_A2_BASE);
}

void TA3_config() {
    MAP_Timer_A_configureUpMode(TIMER_A3_BASE, &ta_config_4ps);
    MAP_Interrupt_setPriority(INT_TA3_0, 0x0F);
    MAP_Timer_A_enableCaptureCompareInterrupt(TIMER_A3_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0);
    MAP_Interrupt_enableInterrupt(INT_TA3_0);
    MAP_Timer_A_stopTimer(TIMER_A3_BASE);
}

void T32_config() {
    MAP_Timer32_initModule(TIMER32_0_BASE, TIMER32_PRESCALER_256, TIMER32_32BIT, TIMER32_PERIODIC_MODE);
    MAP_Timer32_setCount(TIMER32_0_BASE, 468750);
}

void adc_init() {
    MAP_REF_A_setReferenceVoltage(REF_A_VREF2_5V);
    MAP_REF_A_enableReferenceVoltage();

    MAP_ADC14_enableModule();
    MAP_ADC14_initModule(ADC_CLOCKSOURCE_MCLK, ADC_PREDIVIDER_1, ADC_DIVIDER_1, 0);

    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P5, GPIO_PIN1 | GPIO_PIN3 | GPIO_PIN5, GPIO_TERTIARY_MODULE_FUNCTION);

    MAP_ADC14_configureMultiSequenceMode(ADC_MEM0, ADC_MEM2, false);

    MAP_ADC14_configureConversionMemory(ADC_MEM0, ADC_VREFPOS_INTBUF_VREFNEG_VSS, ADC_INPUT_A0, false);
    MAP_ADC14_configureConversionMemory(ADC_MEM1, ADC_VREFPOS_INTBUF_VREFNEG_VSS, ADC_INPUT_A2, false);
    MAP_ADC14_configureConversionMemory(ADC_MEM2, ADC_VREFPOS_INTBUF_VREFNEG_VSS, ADC_INPUT_A4, false);

    MAP_ADC14_enableSampleTimer(ADC_AUTOMATIC_ITERATION);
}

void ultrasonic_sensor_init() {
    MAP_GPIO_setAsInputPin(ECHO_PORT, ECHO_PIN);
    MAP_GPIO_setAsOutputPin(TRIG_PORT, TRIG_PIN);
}

void send_coords() {
    char lat[9], lon[10];
    float f_tmp;
    int i_tmp;
    int i;

    f_tmp = current_lat;

    if (f_tmp < 0) {
        lat[0] = '-';
        f_tmp *= -1;
    }
    else {
        lat[0] = '0';
    }

    // get integer portion of latitude
    i_tmp = (int)f_tmp;

    // get fractional portion of latitude
    f_tmp -= i_tmp;

    for (i = 2; i > 0; i--) {
        lat[i] = i_tmp % 10 + '0';
        i_tmp /= 10;
    }

    lat[3] = '.';

    for (i = 4; i < 9; i++) {
        lat[i] = (int)(f_tmp*10) + '0';
        f_tmp *= 10;
        f_tmp -= (int)f_tmp;
    }

    f_tmp = current_lon;

    if (f_tmp < 0) {
        lon[0] = '-';
        f_tmp *= -1;
    }
    else {
        lon[0] = '0';
    }

    // get integer portion of latitude
    i_tmp = (int)f_tmp;

    // get fractional portion of latitude
    f_tmp -= i_tmp;

    for (i = 3; i > 0; i--) {
        lon[i] = i_tmp % 10 + '0';
        i_tmp /= 10;
    }

    lon[4] = '.';

    for (i = 5; i < 10; i++) {
        lon[i] = (int)(f_tmp*10) + '0';
        f_tmp *= 10;
        f_tmp -= (int)f_tmp;
    }

    MAP_Interrupt_disableMaster();

    // send P to tell the controller to listen for coordinates
    UART1_XBEE_send_char('P');

    // send latitude and longitude
    for (i = 0; i < 9; i++) {
        while (UART1_XBEE_receive_char() != 'n');
        UART1_XBEE_send_char(lat[i]);
    }
    for (i = 0; i < 10; i++) {
        while (UART1_XBEE_receive_char() != 'n');
        UART1_XBEE_send_char(lon[i]);
    }

    MAP_Interrupt_enableMaster();
    //UART1_XBEE_send_char('X');  // new
}

////////////////////////////////////// MAIN //////////////////////////////////////

void main(void)
{
    // initialize variables to be used for calculations when a target location is received.
    float distance_to_target = 0, new_target_heading = 0, avoidance_heading = 0;
    int close_to_target = false;
    int recalculation_time = 0;

    // initial state of rover is RESET, where it waits to be calibrated.
    state = RESET;

    MAP_WDT_A_holdTimer();      // Disable watchdog timer
    clock_config();             // SMCLK = MCLK = DCOCLK = 3MHz, ACLK = REFO
    TA1_config();               // configure TA1 for delay functions
    TA2_config();               // configure TA2 to count time between recalculations while en route
    TA3_config();               // configure TA3 to check for obstacle every 500 ms
    T32_config();               // configure T32 to return home if no X received from controller after 10 seconds

    peripheral_config();        // configure peripherals for GPS, XBee, compass, and motor PWM's



#ifndef TESTING

    // enable interrupts on EUSCI_A1 so commands can be received from controller.
    MAP_Interrupt_setPriority(INT_EUSCIA1, 0x00);
    MAP_UART_clearInterruptFlag(EUSCI_A1_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
    MAP_UART_enableInterrupt(EUSCI_A1_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
    MAP_Interrupt_enableInterrupt(INT_EUSCIA1);

    /* ROVER STATE MACHINE */

    receive_GPS_GLL();
    home_lat = current_lat;
    home_lon = current_lon;

    while (1) {
        /*********************************************************************************************
        *
        * STATE:        RESET
        *
        * PURPOSE:      Notify controller that rover is in reset mode by sending an 'R' every two seconds
        *               until calibration command ('D') is received from controller.
        *
        * NEXT STATE:   SETUP
        *
        **********************************************************************************************/
        if (state == RESET) {
            delay_s(2);
            UART1_XBEE_send_char('R');
            force_setup = false;
            force_stop = false;
        }
        /*********************************************************************************************
        *
        * STATE:        SETUP
        *
        * PURPOSE:      Attempt to calibrate motors. If successful, notify the remote device of success
        *               by sending a 'Y', then calibrate the compass. If unsuccessful, notify the remote
        *               device of failure by sending an 'N'.
        *
        * NEXT STATE:   IDLE
        *
        **********************************************************************************************/
        if (state == SETUP) {
            MAP_Timer32_haltTimer(TIMER32_0_BASE);  // NEW
            MAP_Timer32_disableInterrupt(TIMER32_0_BASE);   // NEW

            //disable UART1 interrupts during SETUP since commands cannot be received in this state
            MAP_UART_disableInterrupt(EUSCI_A1_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);

            calibrate_motor_speed();
            delay_s(1);
            compass_calibrate();
            delay_s(1);
            face_direction(0);
            UART1_XBEE_send_char('Y');

            // enable interrupts on EUSCI_A1 so commands can be received from controller
            MAP_UART_clearInterruptFlag(EUSCI_A1_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
            MAP_UART_enableInterrupt(EUSCI_A1_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);

            MAP_Interrupt_enableInterrupt(INT_T32_INT1);    //NEW
            MAP_Timer32_enableInterrupt(TIMER32_0_BASE);   // NEW
            MAP_Timer32_setCount(TIMER32_0_BASE, 468750);    // new
            MAP_Timer32_startTimer(TIMER32_0_BASE, 0);  // NEW

            state = IDLE;
        }
        /*********************************************************************************************
        *
        * STATE:        IDLE
        *
        * PURPOSE:      Wait for a command from controller.
        *
        * NEXT STATE:   'A' received ---> GETTING_COORDS
        *               '0' received ---> SENDING_DATA
        *               'D' received ---> SETUP
        *               'G' received ---> EN_ROUTE
        *               'S' received ---> IDLE
        *
        **********************************************************************************************/
        if (state == IDLE) {
            while (state == IDLE){
                if (connection_lost || go_home) {
                    state = ORIENTING;
                    break;
                }
            }
            force_stop = false;
            force_setup = false;
            go_home = false;
        }
        /*********************************************************************************************
        *
        * STATE:        GETTING_COORDS
        *
        * PURPOSE:      Receive target coordinates from controller.
        *
        * NEXT STATE:   force_stop signal ---> IDLE
        *               force_setup signal --> SETUP
        *               otherwise -----------> ORIENTING
        *
        **********************************************************************************************/
        if (state == GETTING_COORDS) {
            get_target_coordinates();
            if (force_stop) {
                stop_all_TA0();
                state = IDLE;
            }
            else if (force_setup) {
                stop_all_TA0();
                state = SETUP;
            }
            else {
                state = ORIENTING;
            }
            force_stop = false;
            force_setup = false;
        }
        /*********************************************************************************************
        *
        * STATE:        ORIENTING
        *
        * PURPOSE:      Calculate distance and bearing to target. If the rover is not within range goal,
        *               turn to face the target.
        *
        * NEXT STATE:   force_stop signal ---> IDLE
        *               force_setup signal --> SETUP
        *               within range --------> ARRIVED
        *               otherwise -----------> EN_ROUTE
        *
        **********************************************************************************************/
        if (state == ORIENTING) {
            receive_GPS_GLL();
            if (!connection_lost) send_coords();
            distance_to_target = get_distance(current_lat, current_lon, target_lat, target_lon);
            target_heading = calculate_bearing(current_lat, current_lon, target_lat, target_lon);
            new_target_heading = bearing_to_target();
            // If the distance calculation is within 5 meters or a second bearing calculation is not within 90 degrees
            // of the first, we are already within range. Skip EN_ROUTE and go straight to ARRIVED.
            if ( (distance_to_target < 5) || (change_in_heading(target_heading, new_target_heading) > 90) ) {
                state = ARRIVED;
            }
            // If not within range, turn to face the target.
            else {
                face_direction(target_heading);
                state = EN_ROUTE;
            }
            if (force_stop) {
                stop_all_TA0();
                state = IDLE;
            }
            else if (force_setup) {
                stop_all_TA0();
                state = SETUP;
            }
            force_stop = false;
            force_setup = false;
        }
        /*********************************************************************************************
        *
        * STATE:        EN_ROUTE
        *
        * PURPOSE:      Rover is on its way to the target location. Recalculate distance and heading to target periodically,
        *               determined by recalculation_time, in seconds. If calculated distance to target is within 5 meters or
        *               the newly calculated target heading changes with respect to the previous calculation by 90 degrees or
        *               more (meaning we are too close to the the target to make reliable calculations), we are within our
        *               range goal and go to the next state. If we have not gotten a distance calculation within 20 meters,
        *               we assume we are still far away and continue to update the target heading. Once we receive a read within
        *               20 meters, we stop updating the target heading and wait for a distance calculation within 5 meters or a
        *               change in target heading of 90 degrees or more, then go to the next state.
        *
        * NEXT STATE:   force_stop signal ---> IDLE
        *               force_setup signal --> SETUP
        *               otherwise -----------> ARRIVED
        *
        **********************************************************************************************/
        if (state == EN_ROUTE) {
            // enable obstacle detection
            TA3R = 0;
            MAP_Timer_A_startCounter(TIMER_A3_BASE, TIMER_A_UP_MODE);
            MAP_Timer_A_enableCaptureCompareInterrupt(TIMER_A3_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0);

            close_to_target = distance_to_target < 20 ? true : false;
            recalculation_time = close_to_target ? EN_ROUTE_RECALCULATION_TIME_CLOSE : EN_ROUTE_RECALCULATION_TIME_FAR;
            TA2_ms = 0;
            TA2_seconds = 0;
            MAP_Timer_A_startCounter(TIMER_A2_BASE, TIMER_A_UP_MODE);
            while (1) {
                if (TA2_seconds >= recalculation_time) {
                    receive_GPS_GLL();
                    if (!connection_lost) send_coords();
                    distance_to_target = get_distance(current_lat, current_lon, target_lat, target_lon);
                    new_target_heading = calculate_bearing(current_lat, current_lon, target_lat, target_lon);
                    if (distance_to_target < 5 || change_in_heading(target_heading, new_target_heading) > 90) {
                        state = ARRIVED;
                        break;
                    }
                    if ( !close_to_target && (distance_to_target < 20) ) {
                        close_to_target = true;
                        recalculation_time = EN_ROUTE_RECALCULATION_TIME_CLOSE;
                    }
                    if (!close_to_target) {
                        target_heading = new_target_heading;
                    }
                    TA2_seconds = 0;
                }
                if ( !isBlocked ) {
                    if (moving) {
                        correct_direction(target_heading);
                    }
                    else {
                        UART1_XBEE_send_char('Y');
                        go_direction(target_heading);
                    }
                }
                if (isBlocked || force_stop || force_setup) break;
            }
            MAP_Timer_A_stopTimer(TIMER_A2_BASE);
            if (force_stop) {
               stop_all_TA0();
               state = IDLE;
            }
            else if (force_setup) {
                stop_all_TA0();
                state = SETUP;
            }
            else if (isBlocked) {
                stop_all_TA0();
                state = AVOIDING_OBSTACLE;
            }
            force_stop = false;
            force_setup = false;

            // disable obstacle detection
            MAP_Timer_A_disableCaptureCompareInterrupt(TIMER_A3_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0);

        }
        /*********************************************************************************************
        *
        * STATE:        AVOIDING_OBSTACLE
        *
        * PURPOSE:      Rover encountered an obstacle while en route and is trying to maneuver around it.
        *
        * NEXT STATE:   force_stop signal ---> IDLE
        *               force_setup signal --> SETUP
        *               otherwise -----------> EN_ROUTE
        *
        **********************************************************************************************/
        if (state == AVOIDING_OBSTACLE) {
            // enable obstacle detection
            TA3R = 0;
            MAP_Timer_A_startCounter(TIMER_A3_BASE, TIMER_A_UP_MODE);
            MAP_Timer_A_enableCaptureCompareInterrupt(TIMER_A3_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0);

            while (1) {
                delay_ms(10);
                if (force_stop || force_setup) {
                    break;
                }
                if (isBlocked) {
//                    UART1_XBEE_send_char('0');
                    turn_in_place(-90);
//                    UART1_XBEE_send_char('1');
                    delay_ms(10);
                    if (force_stop || force_setup) {
                        break;
                    }
                    if (isBlocked) {
                        turn_in_place(90);
                        turn_in_place(90);
//                        UART1_XBEE_send_char('2');
                        delay_ms(10);
                        if (force_stop || force_setup) {
                            break;
                        }
                        if (isBlocked) {
                            UART1_XBEE_send_char('F'); // notify controller that obstacle avoidance failed
                            state = IDLE;
                            break;
                        }
                        else {
                            avoidance_heading = get_current_heading();
                            start_fwd(avoidance_heading, 1);
//                            UART1_XBEE_send_char('3');
                            timed_stop(500);
                            face_direction(avoidance_heading);
                            delay_ms(10);
                            turn_in_place(-90);
//                            UART1_XBEE_send_char('4');
                            delay_ms(10);
                            if (!isBlocked) state = EN_ROUTE;
                            break;
                        }
                    }
                    else {
                        avoidance_heading = get_current_heading();
                        start_fwd(avoidance_heading, 1);
                        timed_stop(500);
                        face_direction(avoidance_heading);
//                        UART1_XBEE_send_char('a');
                        delay_ms(500);
                        turn_in_place(90);
//                        UART1_XBEE_send_char('b');
                        delay_ms(100);
                        if (!isBlocked) state = EN_ROUTE;
                        break;
                    }
                }
                else {
                    state = EN_ROUTE;
                    break;
                }
            }
            if (force_stop) {
                stop_all_TA0();
                state = IDLE;
            }
            else if (force_setup) {
                stop_all_TA0();
                state = SETUP;
            }
            force_stop = 0;
            force_setup = 0;

            // disable obstacle detection
            MAP_Timer_A_disableCaptureCompareInterrupt(TIMER_A3_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0);
        }
        /*********************************************************************************************
        *
        * STATE:        ARRIVED
        *
        * PURPOSE:      Rover has arrived at its destination. Notify the controller by sending an
        *               'A' and stop the rover.
        *
        * NEXT STATE:   force_stop signal ---> IDLE
        *               force_setup signal --> SETUP
        *               otherwise -----------> IDLE
        *
        **********************************************************************************************/
        if (state == ARRIVED) {
            UART1_XBEE_send_char('A');
            timed_stop(3000);
            state = IDLE;
            if (force_stop) {
               stop_all_TA0();
               state = IDLE;
            }
            else if (force_setup) {
                stop_all_TA0();
                state = SETUP;
            }
            force_stop = false;
            force_setup = false;
        }
        /*********************************************************************************************
        *
        * STATE:        SENDING_DATA
        *
        * PURPOSE:      Controller has requested sensor data. Get sensor data and send to the controller.
        *
        * NEXT STATE:   IDLE
        *
        **********************************************************************************************/
        if (state == SENDING_DATA) {
            stop_all_TA0();
            UART1_XBEE_send_char('R');
            send_sensor_data();
            force_stop = false;
            force_setup = false;
            state = IDLE;
        }
        if (state == 80) {
            // enable obstacle detection
            TA3R = 0;
            MAP_Timer_A_startCounter(TIMER_A3_BASE, TIMER_A_UP_MODE);
            MAP_Timer_A_enableCaptureCompareInterrupt(TIMER_A3_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0);

            start_fwd(target_heading, 0);
            while(state == 80) {
                if (force_stop || force_setup || isBlocked) {
                    break;
                }
                else if ( !isBlocked ) {
                    if (moving) {
                        correct_direction(target_heading);
                    }
                    else {
                        go_direction(target_heading);
                    }
                }
            }
            if (force_stop) {
                timed_stop(1000);
                state = IDLE;
            }
            else if (force_setup) {
                timed_stop(1000);
                state = SETUP;
            }
            else if (isBlocked) {
                stop_all_TA0();
                state = AVOIDING_OBSTACLE;
            }
            force_stop = false;
            force_setup = false;

            // disable obstacle detection
            MAP_Timer_A_disableCaptureCompareInterrupt(TIMER_A3_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0);
        }
        if (state == 100) {
            stop_all_TA0();
            get_compass_data();
            UART1_XBEE_send_heading();
            state = IDLE;
            force_stop = false;
            force_setup = false;
        }
    }

#else
    ///////////////////////////////// FORWARD MOTION TEST PROGRAM /////////////////////////////////////

    state = IDLE;

    // enable interrupts on EUSCI_A1 so commands can be received from controller.
    MAP_UART_clearInterruptFlag(EUSCI_A1_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
    MAP_UART_enableInterrupt(EUSCI_A1_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
    MAP_Interrupt_enableInterrupt(INT_EUSCIA1);
    TA3R = 0;
    MAP_Timer_A_startCounter(TIMER_A3_BASE, TIMER_A_UP_MODE);
    MAP_Timer_A_enableCaptureCompareInterrupt(TIMER_A3_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0);

    while(1) {
        if (state == ORIENTING) {
            get_compass_data();
            UART1_XBEE_send_heading();
            state = 0;
            force_stop = false;
            force_setup = false;
        }
        if (state == 10) {
            calibrate_motor_speed();
            delay_s(1);
            compass_calibrate();
            delay_s(1);
            face_direction(0);
            state = IDLE;
            force_stop = false;
            force_setup = false;
        }
        if (state == 20) {
            turn_in_place(-10);
            state = 0;
            force_stop = false;
            force_setup = false;
        }
        if (state == 30) {
            turn_in_place(10);
            state = 0;
            force_stop = false;
            force_setup = false;
        }
        if (state == 80) {
            start_fwd(get_current_heading(), 0);
            while(state == 80) {
                if (force_stop || force_setup) break;
                correct_direction(heading);
            }
            force_stop = false;
            force_setup = false;
            if (state != 90) state = IDLE;
        }
        if (state == 90) {
            timed_stop(1500);
            force_stop = false;
            force_setup = false;
            state = 0;
        }
        if (state == 100) {
            go_direction(0);
            force_stop = false;
            force_setup = false;
            state = 0;
        }
        if (state == 110) {
            go_direction(270);
            force_stop = false;
            force_setup = false;
            state = 0;
        }
        if (state == 120) {
            go_direction(180);
            force_stop = false;
            force_setup = false;
            state = 0;
        }
        if (state == 130) {
            go_direction(90);
            force_stop = false;
            force_setup = false;
            state = 0;
        }
    }
#endif
}
