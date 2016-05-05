#include <msp430g2553.h>
#include <stdlib.h>

// Servo API provided through
//              http://gushh.net/blog/msp430-servo/

// Connect the servo SIGNAL wire to P2.4 through a 1K resistor.

#define MCU_CLOCK                       1000000
#define PWM_FREQUENCY           46              // In Hertz, ideally 50Hz.

#define SERVO_STEPS                     180             // Maximum amount of steps in degrees (180 is common)
#define SERVO_MIN                       650             // The minimum duty cycle for this servo
#define SERVO_MAX                       2700    // The maximum duty cycle

// Delay 20 cycles after setting the PWM to rotate the motor
// so it can turn
#define DELAY 20

// When rotating the servo, I found that it rotates quickly enough that the other sensor
// detects the hand, so I added a buffer delay to switching rotation direction.
#define BUFF_DELAY 200

// This is the threshold ADC value a sensor must output for the system to bias a rotation.
// This threshold cancels out noise the IR sensors might be detecting, and combined with the 'cone'
// around each detector, allows each sensor to only detect an object directly in front of it within
// a few centimeters
#define LOW_THRESH 600

// I also found that each of the three sensors detects an object in front of the PCB 'head', but the highest
// ADC reading is from the sensor that object is closest to. This difference threshold is the amount one sensor
// reading has to be different from the others in order to rotate the servo in that direction.
#define DIFF_THRESH 50

// Helper function prototypes
int max(int d1, int d2, int d3);
void find_direction();

unsigned int PWM_Period         = (MCU_CLOCK / PWM_FREQUENCY);  // PWM Period
unsigned int PWM_Duty           = 0;                                                    // %

// The PWM values for the servo
unsigned int servo_lut[ SERVO_STEPS+1 ];

// Variables to record and change the system state
int count = 0;
int go_left = 0;
int left_count = 0;
int go_right = 0;
int right_count = 0;

// Variables to control and record ADC readings
int which = 1;
int distance1, distance2, distance3;
int results[3];

int main(void) {
        unsigned int servo_stepval, servo_stepnow;
        unsigned int i;

        if (CALBC1_1MHZ == 0xFF || CALDCO_1MHZ == 0xff)
        while(1); // Erased calibration data? Trap!

        BCSCTL1 = CALBC1_1MHZ; // Set the DCO to 1 MHz
        DCOCTL = CALDCO_1MHZ; // And load calibration data

        // Calculate the step value and define the current step, defaults to minimum.
        servo_stepval   = ( (SERVO_MAX - SERVO_MIN) / SERVO_STEPS );
        servo_stepnow   = SERVO_MIN;

        // Initialize the PWM values
        for (i = 0; i <= SERVO_STEPS; i++) {
                servo_stepnow += servo_stepval;
                servo_lut[i] = servo_stepnow;
        }

        distance1 = 0;
        distance2 = 0;
        distance3 = 0;

        // Setup the PWM, etc.
        WDTCTL = WDTPW + WDTHOLD;     // Kill watchdog timer

    // Motor Control Configuration using Timer A1
        // Uses SMCLK so that LPM1 can be entered
        TA1CCTL2 = OUTMOD_7;            // TA1CCR2 reset/set
        TA1CTL = TASSEL_2 + MC_1;     // SMCLK, upmode
        TA1CCR0 = PWM_Period-1;        // PWM Period
        TA1CCR2 = PWM_Duty;            // TA1CCR2 PWM Duty Cycle
        P2DIR   |= BIT4;               // P2.4 = output
        P2SEL   |= BIT4;               // P2.4 = TA1 output

    // IR Reading Configuration using ADC10 (A1-IR1, A2-IR2, A4-IR3)
    ADC10CTL1 = INCH_1 + ADC10DIV_3;            // Source ADC from A1 with a clock divider of 3.
    ADC10CTL0 = REF2_5V + ADC10SHT_2 + REFON + ADC10ON + ADC10IE; // Reference with 2.5 V, enable ADC interrupts, turn ADC on

        i = 0;
        __bis_SR_register(GIE);
        // Main loop
        // The IR readings must be synchronous with the servo rotations as we do not want the servo
        // rotating too much upon a reading, nor do we want the servo to be rotating when we're in the
        // middle of reading new IR sensor data.
        while (1) {
                // Read IR1
                ADC10CTL0 &= ~ENC;                                              // block reads
                while (ADC10CTL1 & BUSY);               // Wait if ADC10 core is active
                which = 1;
                ADC10CTL1 = INCH_1 + ADC10DIV_3;        // IR1 is connected to A1
        ADC10CTL0 |= ENC + ADC10SC;                             // enable the read
        __bis_SR_register(LPM1_bits);

        // Read IR2
        ADC10CTL0 &= ~ENC;                                              // block reads
        while (ADC10CTL1 & BUSY);               // Wait if ADC10 core is active
        which = 2;
                ADC10CTL1 = INCH_2 + ADC10DIV_3;        // IR2 is connected to A2
        ADC10CTL0 |= ENC + ADC10SC;                             // enable the read
        __bis_SR_register(LPM1_bits);

        // Read IR3
        ADC10CTL0 &= ~ENC;                                              // block reads
        while (ADC10CTL1 & BUSY);               // Wait if ADC10 core is active
        which = 3;
                ADC10CTL1 = INCH_4 + ADC10DIV_3;        // IR3 is connected to A3
        ADC10CTL0 |= ENC + ADC10SC;                             // enable the read
        __bis_SR_register(LPM1_bits);

        // Determine the direction to turn based on current readings
        find_direction();
        if (go_left) {
                // Turn LEFT (clockwise rotation)
                // Rotate to 65° position
                TA1CCR2 = servo_lut[65];
                __delay_cycles(DELAY);

        } else if (go_right) {
                // Turn RIGHT (counter-clockwise rotation)
                // Rotate to 95° position
                TA1CCR2 = servo_lut[95];
                __delay_cycles(DELAY);

        } else {
                // Don't move
                TA1CCR2 = 0;
        }
        }
        return 0;
}

// Poll the IR sensors upon interrupt, recording the distance reading on each
#pragma vector=ADC10_VECTOR
__interrupt void ADC10_ISR (void) {
        switch (which) {
                case 1:
                        distance1 = ADC10MEM;
                        break;
                case 2:
                        distance2 = ADC10MEM;
                        break;
                case 3:
                        distance3 = ADC10MEM;
                        break;
        }

        __bic_SR_register_on_exit(LPM1_bits);
}


// Based on the IR readings, find which direction we should turn the servo
// Accounts for IR 'noise', two sensors reading a same distance, and switching directions
//      too abruptly
void find_direction() {
        int maxd = max(distance1, distance2, distance3);
        if (maxd <= LOW_THRESH) {
                // Do nothing... below the threshold
                go_left = 0;
                go_right = 0;
                left_count = 0;
                right_count = 0;
                return;
        }

        if (maxd == distance1) {
                // Left sensor has max value
                if (distance1 > (distance2 + DIFF_THRESH) && distance1 > (distance3 + DIFF_THRESH)) {
                        // IR1 has high enough readings, change state
                        left_count++;
                        if (go_right) {
                                // If we're going right, don't switch directions immediately...
                                // STOP going right
                                go_left = 0;
                                go_right = 0;
                                right_count = 0;
                        } else {
                                // Only rotate left if we've passed the buffer delay
                                if (left_count >= BUFF_DELAY) {
                                        go_left = 1;
                                        go_right = 0;
                                }
                        }
                }
        } else if (maxd == distance2) {
                // Go nowhere -- middle sensor has max value
                // Reset all state
                go_left = 0;
                go_right = 0;
                right_count = 0;
                left_count = 0;
        } else if (maxd == distance3) {
                // Right sensor has max value
                if (distance3 > (distance2 + DIFF_THRESH) && distance3 > (distance1 + DIFF_THRESH)) {
                        // IR3 has high enough readings, change state
                        right_count++;
                        if (go_left) {
                                // If we're going left, don't switch directions immediately...
                                // STOP going left
                                go_left = 0;
                                go_right = 0;
                                left_count = 0;
                        } else {
                                // Only rotate right if we've passed the buffer delay
                                if (right_count >= BUFF_DELAY) {
                                        go_left = 0;
                                        go_right = 1;
                                }
                        }
                }
        }
}

// Find the max of three values
// Helper function for find_direction()
int max(int d1, int d2, int d3) {
        if (d1 >= d2 && d1 >= d3)
                return d1;
        else if (d2 >= d1 && d2 >= d3)
                return d2;
        else if (d3 >= d1 && d3 >= d2)
                return d3;
}
