#include <stdio.h>
#include <string.h>
#include "io.hpp"
#include "utilities.h"
#include "adc0.h"
#include "lpc_pwm.hpp"
#include "vd_commons.h"
//#include <math.h>

#define ENABLE_DEBUG            0
#define printf(fmt, ...)        printf("[%3d] "fmt, __LINE__, ##__VA_ARGS__);

#define ZONE_CONVERT(d)         (d / 100)
#define ZONE_OUT_OF_RANGE(d)    (ZONE_CONVERT(d) < 3)
#define ZONE_TOO_FAR(d)         (ZONE_CONVERT(d) < 5  && ZONE_CONVERT(d) >= 3)
#define ZONE_FAR(d)             (ZONE_CONVERT(d) < 8  && ZONE_CONVERT(d) >= 5)
#define ZONE_IN_RANGE(d)        (ZONE_CONVERT(d) >= 8 && ZONE_CONVERT(d) <= 15)
#define ZONE_CLOSE(d)           (ZONE_CONVERT(d) > 15)

static const int QLEN = 30;
static const int LOGLEN = 600;
static const int VD_LEFT_ERROR = 15;
static const int VD_RIGHT_ERROR = 0;
static const int VD_THRESHOLD = 600;

static struct {
        int leftValue;
        int middleValue;
        int rightValue;
        //char leftValid:1;
        //char middleValid:1;
        //char rightValid:1;
} sensor;

/* logging related variables */
static int rec[LOGLEN][6];
static int ri = 0;
static int pi = 0;
static char pEnable = 0;
static char sEnable = 0;

static int paused = 1; // when VD starts, it should start in paused mode
static int startBT = 0;

/* state machine related variables */
static enum {
    VD_ALARM,
    VD_STOP,
    VD_FWD,
    VD_REV,
    VD_FWD_LEFT,
    VD_FWD_RIGHT,
    VD_REV_LEFT,
    VD_REV_RIGHT,
    VD_TURN
} vdState;

static enum {
    VD_HAULT,
    VD_SLOW = 35,
    VD_MEDIUM = 50,
    VD_FAST = 70
} vdSpeed;

static int targetDist;
static int lastTarget;

/* motor drivers */
static PWM pwmLeftFWD(PWM::pwm2, 1000); // P2.1
static PWM pwmLeftREV(PWM::pwm3, 1000); // P2.2
static PWM pwmRightFWD(PWM::pwm4, 1000); // P2.3
static PWM pwmRightREV(PWM::pwm5, 1000); // P2.4

static void vdCheckButtons(void)
{
    if(SW.getSwitchValues()) {
        LE.setAll(SW.getSwitchValues());

        if(SW.getSwitch(1)) {
            printf("Onboard Switch 1 Pressed\n");
            int i;
            /* print only 50 logs at a time, next 50 will be printed when button is pressed again */
            for(i = 0; i < 50; i++) {
                printf("<%4d:%4d :: %4d:%4d :: %4d:%4d>\n", rec[pi][0], rec[pi][1], rec[pi][2], rec[pi][3], rec[pi][4], rec[pi][5]);
                pi++;
            }
            if(pi >= LOGLEN) pi = 0;
        }
        if(SW.getSwitch(2)) {
            printf("Onboard Switch 2 Pressed\n");
            pEnable = !pEnable;
        }
        if(SW.getSwitch(4)) {
            printf("Onboard Switch 4 Pressed\n");
            if(paused && !ZONE_IN_RANGE(sensor.middleValue)) {
                printf("Object not in range, cannot resume.\n");
            }
            else {
                paused = !paused;
            }

            if(paused) {
                printf("VD Paused\n");
            }
            else {
                printf("VD Resumed\n");
                vdState = VD_STOP;
                vdSpeed = VD_HAULT;
                targetDist = 1100;
            }
        }
        delay_ms(300); // switch debouncing
        LE.setAll(0);
    }
}

static void vdNormalizeSensorValues(void)
{
    static int leftQueue[QLEN], middleQueue[QLEN], rightQueue[QLEN];
    static int qTail;
    int leftSorted[QLEN], middleSorted[QLEN], rightSorted[QLEN];
    int i, j, leftBig, middleBig, rightBig, temp;


    /* save the current sensor value at the end of circular queue */
    leftQueue[qTail] = adc0_get_reading(4);
    middleQueue[qTail] = adc0_get_reading(5);
    rightQueue[qTail] = adc0_get_reading(3);

    /* copy queue to temporary array to perform sort */
    //memcpy(leftSorted, leftQueue, sizeof(leftSorted));
    //memcpy(middleSorted, middleQueue, sizeof(middleSorted));
    //memcpy(rightSorted, rightQueue, sizeof(rightSorted));
    /* instead of running 3 memcpy, copy in one for loop */
    for(i = 0; i < QLEN; i++) {
        leftSorted[i] = leftQueue[i];
        middleSorted[i] = middleQueue[i];
        rightSorted[i] = rightQueue[i];
    }

    /* optimized bubble sort */
    for(i = 0; i < QLEN; i++) {
       for(j = leftBig = middleBig = rightBig = 0; j < (QLEN - i); j++) {
          if(leftSorted[j] > leftSorted[leftBig]) {
             leftBig = j;
          }
          if(middleSorted[j] > middleSorted[middleBig]) {
             middleBig = j;
          }
          if(rightSorted[j] > rightSorted[rightBig]) {
             rightBig = j;
          }
       }

       temp = leftSorted[leftBig];
       leftSorted[leftBig] = leftSorted[(QLEN - 1) - i];
       leftSorted[(QLEN - 1) - i] = temp;

       temp = middleSorted[middleBig];
       middleSorted[middleBig] = middleSorted[(QLEN - 1) - i];
       middleSorted[(QLEN - 1) - i] = temp;

       temp = rightSorted[rightBig];
       rightSorted[rightBig] = rightSorted[(QLEN - 1) - i];
       rightSorted[(QLEN - 1) - i] = temp;
    }

#if 0
    if((leftSorted[4] > (leftSorted[QLEN / 2] - THRESHOLD)) && (leftSorted[QLEN - 5] < (leftSorted[QLEN / 2] + THRESHOLD))) {
        sensor.leftValid = 1;
        sensor.leftValue = leftSorted[QLEN / 2];
    }

    if((middleSorted[4] > (middleSorted[QLEN / 2] - THRESHOLD)) && (middleSorted[QLEN - 5] < (middleSorted[QLEN / 2] + THRESHOLD))) {
        sensor.middleValid = 1;
        sensor.middleValue = middleSorted[QLEN / 2];
    }

    if((rightSorted[4] > (rightSorted[QLEN / 2] - THRESHOLD)) && (rightSorted[QLEN - 5] < (rightSorted[QLEN / 2] + THRESHOLD))) {
        sensor.rightValid = 1;
        sensor.rightValue = rightSorted[QLEN / 2];
    }
#else
    //sensor.leftValid = 1;
    //sensor.middleValid = 1;
    //sensor.rightValid = 1;
    sensor.leftValue = leftSorted[QLEN / 2];
    sensor.middleValue = middleSorted[QLEN / 2];
    sensor.rightValue = rightSorted[QLEN / 2];
#endif

#if ENABLE_DEBUG
    /* maintain log of past few values for debugging */
    /* actual values */
    rec[ri][1] = leftQueue[qTail];
    rec[ri][3] = middleQueue[qTail];
    rec[ri][5] = rightQueue[qTail];
    /* normalized values */
    rec[ri][0] = leftSorted[QLEN / 2];
    rec[ri][2] = middleSorted[QLEN / 2];
    rec[ri][4] = rightSorted[QLEN / 2];

    if(pEnable) {
        printf("%4d %4d %4d\n", sensor.leftValue, sensor.middleValue, sensor.rightValue);
    }

    ri++;
    if(ri >= LOGLEN) ri = 0;
#endif

    qTail++;
    if(qTail >= QLEN) {
        qTail = 0;
    }

    LD.setNumber(sensor.middleValue / 100); // only first two digits of sensor reading
}

static void vdReadSensor(void)
{
    static int alarmTarget;

    if(paused) {
        vdState = VD_STOP;
        return;
    }

    switch(vdState) {
        case VD_FWD:
            /*if((sensor.middleValue - lastTarget) > VD_THRESHOLD) {
                /* some obstacle has been detected, sound alarm *
                vdState = VD_ALARM;
                alarmTarget = lastTarget;
            }
            else*/ if(ZONE_FAR(sensor.middleValue)) {
                vdSpeed = VD_MEDIUM;
            }
            else if(ZONE_TOO_FAR(sensor.middleValue)) {
                vdSpeed = VD_FAST;
                if(ZONE_FAR(sensor.rightValue) || ZONE_TOO_FAR(sensor.rightValue)) {
                    vdState = VD_FWD_RIGHT;
                    lastTarget = sensor.rightValue;
                }
                else if(ZONE_FAR(sensor.leftValue) || ZONE_TOO_FAR(sensor.leftValue)) {
                    vdState = VD_FWD_LEFT;
                    lastTarget = sensor.leftValue;
                }
            }
            else if(ZONE_OUT_OF_RANGE(sensor.middleValue)) {
                 vdState = VD_TURN;
            }
            //else if(sensor.middleValue > targetDist) { /**/
            //    vdState = VD_STOP;
            //}
            else if(ZONE_IN_RANGE(sensor.middleValue)) {
                vdState = VD_STOP;
            }
            lastTarget = sensor.middleValue;
            break;

        case VD_REV:
            /*if((sensor.middleValue - lastTarget) > VD_THRESHOLD) {
                /* some obstacle has been detected, sound alarm *
                vdState = VD_ALARM;
                alarmTarget = lastTarget;
            }
            else*/ if(ZONE_CLOSE(sensor.middleValue)) {
                vdSpeed = VD_SLOW;
            }
            else if(sensor.middleValue < targetDist) { /**/
                vdState = VD_STOP;
            }
            //else if(ZONE_IN_RANGE(sensor.middleValue)) {
            //    vdState = VD_STOP;
            //}
            lastTarget = sensor.middleValue;
            break;

        case VD_TURN:
            if(ZONE_FAR(sensor.rightValue) || ZONE_TOO_FAR(sensor.rightValue)) {
                vdState = VD_FWD_RIGHT;
                lastTarget = sensor.rightValue;
            }
            else if(ZONE_FAR(sensor.leftValue) || ZONE_TOO_FAR(sensor.leftValue)) {
                vdState = VD_FWD_LEFT;
                lastTarget = sensor.leftValue;
            }
            else if(ZONE_IN_RANGE(sensor.rightValue) || ZONE_CLOSE(sensor.rightValue)) {
                vdState = VD_REV_LEFT;
                lastTarget = sensor.rightValue;
            }
            else if(ZONE_IN_RANGE(sensor.leftValue) || ZONE_CLOSE(sensor.leftValue)) {
                vdState = VD_REV_RIGHT;
                lastTarget = sensor.leftValue;
            }
            break;

        case VD_FWD_LEFT:
            if(ZONE_FAR(sensor.middleValue) || ZONE_IN_RANGE(sensor.middleValue) || ZONE_CLOSE(sensor.middleValue)) {
                vdState = VD_STOP;
            }
            lastTarget = sensor.middleValue;
            break;

        case VD_FWD_RIGHT:
            if(ZONE_FAR(sensor.middleValue) || ZONE_IN_RANGE(sensor.middleValue) || ZONE_CLOSE(sensor.middleValue)) {
                vdState = VD_STOP;
            }
            lastTarget = sensor.middleValue;
            break;

        case VD_REV_LEFT:
            if(ZONE_IN_RANGE(sensor.middleValue) || ZONE_CLOSE(sensor.middleValue)) {
                vdState = VD_STOP;
            }
            lastTarget = sensor.middleValue;
            break;

        case VD_REV_RIGHT:
            if(ZONE_IN_RANGE(sensor.middleValue) || ZONE_CLOSE(sensor.middleValue)) {
                vdState = VD_STOP;
            }
            lastTarget = sensor.middleValue;
            break;

        case VD_ALARM:
            if(alarmTarget - 200 < sensor.middleValue && alarmTarget + 200 > sensor.middleValue) {
                vdState = VD_STOP;
            }
            break;

        case VD_STOP:
        default:
            /*if((sensor.middleValue - lastTarget) > VD_THRESHOLD) {
                /* some obstacle has been detected, sound alarm *
                vdState = VD_ALARM;
                alarmTarget = lastTarget;
            }
            else*/ if(ZONE_FAR(sensor.middleValue) || ZONE_TOO_FAR(sensor.middleValue) || ZONE_OUT_OF_RANGE(sensor.middleValue)) {
                vdState = VD_FWD;
            }
            else if(ZONE_CLOSE(sensor.middleValue)) {
                vdState = VD_REV;
            }
            lastTarget = sensor.middleValue;
            break;
    }
}

static inline void vdMotorDrive(int leftFWD, int leftREV, int rightFWD, int rightREV)
{
    pwmLeftFWD.set((leftFWD)? (leftFWD + VD_LEFT_ERROR/* + VD_LEFT_ERROR / 2*/): VD_HAULT);
    pwmLeftREV.set((leftREV)? (leftREV + VD_LEFT_ERROR/* + VD_LEFT_ERROR*/): VD_HAULT);
    pwmRightFWD.set((rightFWD)? (rightFWD + VD_RIGHT_ERROR): VD_HAULT);
    pwmRightREV.set((rightREV)? (rightREV + VD_RIGHT_ERROR): VD_HAULT);
}

static void vdRunMotor(void)
{
    static int lastState = VD_STOP;

     if(paused) {
        vdMotorDrive(VD_HAULT, VD_HAULT, VD_HAULT, VD_HAULT);
        return;
     }

     if(vdState == lastState) {
          return;
     }

    switch(vdState) {
        case VD_FWD:
            vdMotorDrive(vdSpeed, VD_HAULT, vdSpeed, VD_HAULT);
            break;

        case VD_REV:
            vdMotorDrive(VD_HAULT, vdSpeed, VD_HAULT, vdSpeed);
            break;

        case VD_FWD_LEFT:
            vdMotorDrive(VD_SLOW, VD_HAULT, VD_FAST + VD_SLOW, VD_HAULT);
            break;

        case VD_FWD_RIGHT:
            vdMotorDrive(VD_FAST, VD_HAULT, VD_SLOW, VD_HAULT);
            break;

        case VD_REV_LEFT:
            vdMotorDrive(VD_HAULT, VD_HAULT, VD_HAULT, VD_MEDIUM);
            break;

        case VD_REV_RIGHT:
            vdMotorDrive(VD_HAULT, VD_MEDIUM, VD_HAULT, VD_HAULT);
            break;

        default:
            vdMotorDrive(VD_HAULT, VD_HAULT, VD_HAULT, VD_HAULT);
            break;
    }

    lastState = vdState;
}

static void vdIndicatorLED(void)
{
    if(paused) {
        LE.setAll(0);
        return;
    }

    switch(vdState) {
        case VD_FWD:
            LE.off(1);
            LE.on(2);
            LE.on(3);
            LE.off(4);
            break;

        case VD_REV:
            LE.on(1);
            LE.off(2);
            LE.off(3);
            LE.on(4);
            break;

        case VD_TURN:
            LE.on(1);
            LE.off(2);
            LE.off(3);
            LE.on(4);
            delay_ms(100);
            LE.off(1);
            LE.off(2);
            LE.off(3);
            LE.off(4);
            delay_ms(500);
            break;

        case VD_FWD_LEFT:
            LE.on(1);
            LE.on(2);
            LE.off(3);
            LE.off(4);
            break;

        case VD_FWD_RIGHT:
            LE.off(1);
            LE.off(2);
            LE.on(3);
            LE.on(4);
            break;

        case VD_REV_LEFT:
            LE.on(1);
            LE.off(2);
            LE.off(3);
            LE.off(4);
            break;

        case VD_REV_RIGHT:
            LE.off(1);
            LE.off(2);
            LE.off(3);
            LE.on(4);
            break;

        case VD_ALARM:
            LE.on(1);
            LE.on(2);
            LE.on(3);
            LE.on(4);
            delay_ms(100);
            LE.off(1);
            LE.off(2);
            LE.off(3);
            LE.off(4);
            delay_ms(100);
            break;

        case VD_STOP:
        default:
            LE.off(1);
            LE.on(2);
            LE.on(3);
            LE.off(4);
            delay_ms(500);
            LE.on(1);
            LE.off(2);
            LE.off(3);
            LE.on(4);
            delay_ms(500);
            break;
    }
}

static void vdBuzzer(void)
{
    const uint32_t buzzerBit = (1 << 23);

    if(paused) {
        LPC_GPIO1->FIOCLR = buzzerBit;
        return;
    }

    if(vdState == VD_ALARM) {
        LPC_GPIO1->FIOSET = buzzerBit;
        delay_ms(500);
        LPC_GPIO1->FIOCLR = buzzerBit;
        delay_ms(500);
    }
}

static void txbyte(char byte){
    printf("sending %c[%d]\n", byte, byte);
    LPC_UART2->THR = byte;
    while(! (LPC_UART2->LSR & (1 << 6)));
}

static void vdBluetoothRx(void)
{
    int rv;

    while(! (LPC_UART2->LSR & (1<<0)));
    rv = LPC_UART2->RBR;

    //printf("BT data received %d\n", rv);
    if(rv == 3) {
        if(!ZONE_IN_RANGE(sensor.middleValue)) {
            printf("Object not in range, cannot resume.\n");
        }
        else {
            startBT = 1;
            paused = 0;
        }
        //printf("BT started\n");
    }
    else {
        startBT = 0;
        paused = 1;
        //printf("BT stopped\n");
    }
}

static void vdBluetoothTx(void)
{
    int i;

    if(!startBT) {
        return;
    }
    //    sensor.leftValue = 3000;
    //    sensor.rightValue = 4000;
    //    sensor.middleValue = 5000;
    int left,middle,right;
    while(1){
        if(!startBT) {
            break;
        }
        txbyte('0');        // dummy byte
        left  = sensor.leftValue;
        middle = sensor.middleValue;
        right = sensor.rightValue;

        while(left) {
            txbyte((left % 10) + '0');
            left = left / 10;
        }
        txbyte('~');
        while(middle) {
            txbyte((middle % 10) + '0');
            middle = middle / 10;
        }
        txbyte('~');
        while(right) {
            txbyte((right % 10) + '0');
            right = right / 10;
        }
        txbyte('~');

        txbyte(vdState+'0');
        txbyte('\n');
        delay_ms(300);

    }
}

#if 0
#define pLINE() if(sEnable) printf("{%4d} ", __LINE__)

static const int QLEN = 30;
static const int VD_THRESHOLD = 200;
static const int LOGLEN = 600;
static const int VD_LEFT_ERROR = 10;
static const int VD_RIGHT_ERROR = 0;
static const int VD_TURN_ERROR = 30;

static const float COS25 = 0.90630778703664996324255265675432;

static int paused = 1; // when VD starts, it should start in paused mode

static struct {
        int leftValue;
        int middleValue;
        int rightValue;
        //char leftValid:1;
        //char middleValid:1;
        //char rightValid:1;
} sensor;

/* logging related variables */
static int rec[LOGLEN][6];
static int ri = 0;
static int pi = 0;
static char pEnable = 0;
static char sEnable = 0;

/* state machine related variables */
static enum {
    VD_ALARM,
    VD_STOP,
    VD_FWD,
    VD_REV,
    VD_FWD_LEFT,
    VD_FWD_RIGHT,
    VD_REV_LEFT,
    VD_REV_RIGHT,
    VD_TURN
} vdState;

static enum {
    VD_HAULT,
    VD_SLOW = 30,
    VD_MEDIUM = 40,
    VD_FAST = 50
} vdSpeed;

static int targetDist;
static int lastTarget;

/* motor drivers */
static PWM pwmLeftFWD(PWM::pwm2, 1000); // P2.1
static PWM pwmLeftREV(PWM::pwm3, 1000); // P2.2
static PWM pwmRightFWD(PWM::pwm4, 1000); // P2.3
static PWM pwmRightREV(PWM::pwm5, 1000); // P2.4

static const uint32_t ledBit = (1 << 23);

static void normalizeSensorValues(void)
{
    static int leftQueue[QLEN], middleQueue[QLEN], rightQueue[QLEN];
    static int qTail;
    int leftSorted[QLEN], middleSorted[QLEN], rightSorted[QLEN];
    int i, j, leftBig, middleBig, rightBig, temp;

    /* save the current sensor value at the end of circular queue */
    leftQueue[qTail] = adc0_get_reading(4) * COS25;
    middleQueue[qTail] = adc0_get_reading(5) * COS25;
    rightQueue[qTail] = adc0_get_reading(3) * COS25;

    /* copy queue to temporary array to perform sort */
    memcpy(leftSorted, leftQueue, sizeof(leftSorted));
    memcpy(middleSorted, middleQueue, sizeof(middleSorted));
    memcpy(rightSorted, rightQueue, sizeof(rightSorted));

    /* optimized bubble sort */
    for(i = 0; i < QLEN; i++) {
       for(j = leftBig = middleBig = rightBig = 0; j < (QLEN - i); j++) {
          if(leftSorted[j] > leftSorted[leftBig]) {
             leftBig = j;
          }
          if(middleSorted[j] > middleSorted[middleBig]) {
             middleBig = j;
          }
          if(rightSorted[j] > rightSorted[rightBig]) {
             rightBig = j;
          }
       }

       temp = leftSorted[leftBig];
       leftSorted[leftBig] = leftSorted[(QLEN - 1) - i];
       leftSorted[(QLEN - 1) - i] = temp;

       temp = middleSorted[middleBig];
       middleSorted[middleBig] = middleSorted[(QLEN - 1) - i];
       middleSorted[(QLEN - 1) - i] = temp;

       temp = rightSorted[rightBig];
       rightSorted[rightBig] = rightSorted[(QLEN - 1) - i];
       rightSorted[(QLEN - 1) - i] = temp;
    }

#if 0
    if((leftSorted[4] > (leftSorted[QLEN / 2] - THRESHOLD)) && (leftSorted[QLEN - 5] < (leftSorted[QLEN / 2] + THRESHOLD))) {
        sensor.leftValid = 1;
        sensor.leftValue = leftSorted[QLEN / 2];
    }

    if((middleSorted[4] > (middleSorted[QLEN / 2] - THRESHOLD)) && (middleSorted[QLEN - 5] < (middleSorted[QLEN / 2] + THRESHOLD))) {
        sensor.middleValid = 1;
        sensor.middleValue = middleSorted[QLEN / 2];
    }

    if((rightSorted[4] > (rightSorted[QLEN / 2] - THRESHOLD)) && (rightSorted[QLEN - 5] < (rightSorted[QLEN / 2] + THRESHOLD))) {
        sensor.rightValid = 1;
        sensor.rightValue = rightSorted[QLEN / 2];
    }
#else
    //sensor.leftValid = 1;
    //sensor.middleValid = 1;
    //sensor.rightValid = 1;
    sensor.leftValue = leftSorted[QLEN / 2];
    sensor.middleValue = middleSorted[QLEN / 2];
    sensor.rightValue = rightSorted[QLEN / 2];
#endif

    /* maintain log of past few values for debugging */
    /* actual values */
    rec[ri][1] = leftQueue[qTail];
    rec[ri][3] = middleQueue[qTail];
    rec[ri][5] = rightQueue[qTail];
    /* normalized values */
    rec[ri][0] = leftSorted[QLEN / 2];
    rec[ri][2] = middleSorted[QLEN / 2];
    rec[ri][4] = rightSorted[QLEN / 2];

    if(pEnable) {
        printf("%4d %4d %4d\n", sensor.leftValue, sensor.middleValue, sensor.rightValue);
    }

    ri++;
    if(ri >= LOGLEN) ri = 0;

    qTail++;
    if(qTail >= QLEN) {
        qTail = 0;
    }

    //LD.setNumber(sensor.middleValue/100);
    //LD.setNumber((3000-sensor.middleValue)/60); // inches
    LD.setNumber(((3000 - sensor.middleValue) / 50) * COS25);
}

static void vdCheckButtons(void)
{
    if(SW.getSwitchValues()) {
        if(SW.getSwitch(4)) {
            printf("\n");
            printf("\n");
            printf("Onboard Switch 4 Pressed\n");
            paused = !paused;
            if(paused) {
                printf("VD Paused\n");
            }
            else {
                printf("VD Resumed\n");
                vdState = VD_STOP;
                vdSpeed = VD_HAULT;
                targetDist = sensor.middleValue;
                lastTarget = targetDist;
                if(targetDist < 1000 || targetDist > 2000) {
                    printf("[%4d] Target Distance should be within 1000-2000 when starting!\n", targetDist);
                    printf("VD Paused\n");
                    paused = !paused;
                }
            }
            printf("\n");
            printf("\n");
        }
        if(SW.getSwitch(3)) {
            printf("Onboard Switch 3 Pressed\n");
            sEnable = !sEnable;
        }
        if(SW.getSwitch(2)) {
            printf("Onboard Switch 2 Pressed\n");
            pEnable = !pEnable;
        }
        if(SW.getSwitch(1)) {
            printf("Onboard Switch 1 Pressed\n");
            int i;
            /* print only 50 logs at a time, next 50 will be printed when button is pressed again */
            for(i = 0; i < 50; i++) {
                printf("<%4d:%4d :: %4d:%4d :: %4d:%4d>\n", rec[pi][0], rec[pi][1], rec[pi][2], rec[pi][3], rec[pi][4], rec[pi][5]);
                pi++;
            }
            if(pi >= LOGLEN) pi = 0;
        }

        delay_ms(300); // switch debouncing
    }
}

static void vdReadSensor(void)
{
    normalizeSensorValues();

    if(paused) {
        return;
    }

    if(sEnable) {
        printf("%4d ", lastTarget);
    }
    switch(vdState) {
        case VD_FWD:
            if((sensor.middleValue - lastTarget) > VD_THRESHOLD * 5) {
                /* some obstacle has been detected, sound alarm */
                vdState = VD_ALARM;
                pLINE();
            }
#if 1
            else if(sensor.leftValue < 500 && sensor.middleValue < 500 && sensor.rightValue < 500) {
                vdState = VD_TURN;
                pLINE();
            }
#endif
            else if((lastTarget - sensor.middleValue) > VD_THRESHOLD * 4 || sensor.middleValue < 500) {
                /* object has turned */
                if(sensor.rightValue > 500 && sensor.leftValue < 500) {
                    vdState = VD_FWD_RIGHT;
                    pLINE();
                }
                else if(sensor.leftValue > 500 && sensor.rightValue < 500) {
                    vdState = VD_FWD_LEFT;
                    pLINE();
                }
                else {
                    vdState = VD_TURN;
                    pLINE();
                }
            }
            else if((targetDist - sensor.middleValue) > VD_THRESHOLD * 3) {
                vdSpeed = VD_FAST;
                pLINE();
            }
            else if((targetDist - sensor.middleValue) > VD_THRESHOLD * 2) {
                vdSpeed = VD_MEDIUM;
                pLINE();
            }
            else if((targetDist - sensor.middleValue) > VD_THRESHOLD * 1) {
                vdSpeed = VD_SLOW;
                pLINE();
            }
            else {
                vdState = VD_STOP;
                pLINE();
            }
            lastTarget = sensor.middleValue;
            break;

        case VD_REV:
            if((sensor.middleValue - lastTarget) > VD_THRESHOLD * 5) {
                /* some obstacle has been detected, sound alarm */
                vdState = VD_ALARM;
                pLINE();
            }
            //else if((lastTarget - sensor.middleValue) > VD_THRESHOLD * 4) {
            else if((lastTarget - sensor.middleValue) > VD_THRESHOLD * 2) {
            //else if((lastTarget - sensor.middleValue) > 0) {
                /* object has turned */
                if(sensor.rightValue < 500 && sensor.leftValue > targetDist) {
                    vdState = VD_REV_RIGHT;
                    pLINE();
                }
                else if(sensor.leftValue < 500 && sensor.rightValue > targetDist) {
                    vdState = VD_REV_LEFT;
                    pLINE();
                }
                else {
                    vdState = VD_TURN;
                    pLINE();
                }
            }
            else if((sensor.middleValue - targetDist) > VD_THRESHOLD * 3) {
                vdSpeed = VD_FAST;
                pLINE();
            }
            else if((sensor.middleValue - targetDist) > VD_THRESHOLD * 2) {
                vdSpeed = VD_MEDIUM;
                pLINE();
            }
            else if((sensor.middleValue - targetDist) > VD_THRESHOLD * 1) {
                vdSpeed = VD_SLOW;
                pLINE();
            }
            else {
                vdState = VD_STOP;
                pLINE();
            }
            lastTarget = sensor.middleValue;
            break;

        case VD_FWD_LEFT:
        case VD_REV_RIGHT:
            if(sensor.middleValue > (targetDist - VD_THRESHOLD * 5) && sensor.middleValue < (targetDist + VD_THRESHOLD * 5)) {
                vdState = VD_STOP;
                lastTarget = sensor.middleValue;
                pLINE();
            }
            else {
                //if((lastTarget - sensor.leftValue) > VD_THRESHOLD * 3) {
                if((lastTarget - sensor.leftValue) > VD_THRESHOLD * 3 || sensor.leftValue < 500) {

                    /* object has turned fast, increase speed */
                    vdSpeed = VD_FAST;
                    pLINE();
                }
                else {
                    /* take left */
                    vdSpeed = VD_MEDIUM;
                    pLINE();
                }
                lastTarget = sensor.leftValue;
            }
            break;

        case VD_FWD_RIGHT:
        case VD_REV_LEFT:
            if(sensor.middleValue > (targetDist - VD_THRESHOLD * 5) && sensor.middleValue < (targetDist + VD_THRESHOLD * 5)) {
                vdState = VD_STOP;
                lastTarget = sensor.middleValue;
                pLINE();
            }
            else {
                //if((lastTarget - sensor.rightValue) > VD_THRESHOLD * 3) {
                if((lastTarget - sensor.rightValue) > VD_THRESHOLD * 3 || sensor.rightValue < 500) {
                    /* object has turned fast, increase speed */
                    vdSpeed = VD_FAST;
                    pLINE();
                }
                else {
                    /* take right */
                    vdSpeed = VD_MEDIUM;
                    pLINE();
                }
                lastTarget = sensor.rightValue;
            }
            break;

        case VD_TURN:
            if(sensor.leftValue > (targetDist - VD_THRESHOLD * 3) && sensor.leftValue < (targetDist + VD_THRESHOLD * 3)) {
                if(sensor.leftValue < targetDist) {
                    vdState = VD_FWD_LEFT;
                    pLINE();
                }
                else {
                    vdState = VD_REV_RIGHT;
                    pLINE();
                }
                lastTarget = sensor.leftValue;
            }
            else if(sensor.rightValue > (targetDist - VD_THRESHOLD * 3) && sensor.rightValue < (targetDist + VD_THRESHOLD * 3)) {
                if(sensor.rightValue < targetDist) {
                    vdState = VD_FWD_RIGHT;
                    pLINE();
                }
                else {
                    vdState = VD_REV_LEFT;
                    pLINE();
                }
                lastTarget = sensor.rightValue;
            }
            else if(sensor.rightValue < 500 && sensor.leftValue > targetDist) {
                vdState = VD_REV_RIGHT;
                pLINE();
                lastTarget = sensor.leftValue;
            }
            else if(sensor.leftValue < 500 && sensor.rightValue > targetDist) {
                vdState = VD_REV_LEFT;
                pLINE();
                lastTarget = sensor.rightValue;
            }
            else if(sensor.middleValue < 500 && sensor.leftValue > 500) {
                vdState = VD_FWD_LEFT;
                pLINE();
                lastTarget = sensor.leftValue;
            }
            else if(sensor.middleValue < 500 && sensor.rightValue > 500) {
                vdState = VD_FWD_RIGHT;
                pLINE();
                lastTarget = sensor.rightValue;
            }
            else if(sensor.middleValue > (targetDist - VD_THRESHOLD * 3) && sensor.middleValue < (targetDist + VD_THRESHOLD * 3)) {
                vdState = VD_STOP;
                lastTarget = sensor.middleValue;
                pLINE();
            }
            else if(sensor.middleValue > 500 && sensor.leftValue < 500 && sensor.rightValue < 500) {
                vdState = VD_STOP;
                lastTarget = sensor.middleValue;
                pLINE();
            }
            else {
                pLINE();
            }
            break;

        case VD_ALARM:
            vdSpeed = VD_HAULT;
            if((lastTarget - sensor.middleValue) > VD_THRESHOLD * 4 && sensor.middleValue > 1000) {
                vdState = VD_STOP;
                pLINE();
            }
            else {
                pLINE();
            }
            lastTarget = sensor.middleValue;
            break;

        case VD_STOP:
        default:
            vdSpeed = VD_HAULT;
            if(sensor.middleValue > (targetDist - VD_THRESHOLD) && sensor.middleValue < (targetDist + VD_THRESHOLD)) {
                /* do nothing */
                pLINE();
            }
            else if(sensor.middleValue < (targetDist - VD_THRESHOLD)) {
                /* object is moving forward */
                if((lastTarget - sensor.middleValue) > VD_THRESHOLD * 4) {
                    /* object has turned */
                    vdState = VD_TURN;
                    pLINE();
                }
                else {
                    vdState = VD_FWD;
                    pLINE();
                }
            }
            else if(sensor.middleValue > (targetDist + VD_THRESHOLD)) {
                /* object is moving closer */
                /* check if any obstacle has appeared */
                if((sensor.middleValue - lastTarget) > VD_THRESHOLD * 5) {
                    /* sound alarm */
                    vdState = VD_ALARM;
                    pLINE();
                }
                else {
                    vdState = VD_REV;
                    pLINE();
                }
            }
            else {
                pLINE();
            }
            lastTarget = sensor.middleValue;
            break;
    }

    /* print log */
    if(sEnable) {
        char str[8] = {0};

        switch(vdSpeed) {
            case VD_FAST:   str[6] = '+';
            case VD_MEDIUM: str[5] = '+';
            case VD_SLOW:   str[4] = '+';
        }

        str[0] = str[2] = str[3] = ' ';
        switch(vdState) {
            case VD_FWD:        str[1] = '^'; break;
            case VD_REV:        str[1] = 'v'; break;
            case VD_FWD_LEFT:   str[1] = '^'; str[0] = '<'; break;
            case VD_FWD_RIGHT:  str[1] = '^'; str[2] = '>'; break;
            case VD_REV_LEFT:   str[1] = 'v'; str[0] = '<'; break;
            case VD_REV_RIGHT:  str[1] = 'v'; str[2] = '>'; break;
            case VD_TURN:       str[1] = '#'; break;
            case VD_ALARM:      str[1] = '@'; break;
            default:            str[1] = '.'; str[2] = 0; break;
        }
        printf("[%4d] %4d %4d %4d %s\n", targetDist, sensor.leftValue, sensor.middleValue, sensor.rightValue, str);
    }
}

static void vdRunMotor(void)
{
    if(paused) {
        pwmLeftFWD.set(VD_HAULT);
        pwmLeftREV.set(VD_HAULT);
        pwmRightFWD.set(VD_HAULT);
        pwmRightREV.set(VD_HAULT);
        return;
     }

    switch(vdState) {
        case VD_FWD:
            pwmLeftFWD.set(vdSpeed + VD_LEFT_ERROR);
            pwmLeftREV.set(VD_HAULT);
            pwmRightFWD.set(vdSpeed + VD_RIGHT_ERROR);
            pwmRightREV.set(VD_HAULT);
            LE.off(1);
            LE.on(2);
            LE.on(3);
            LE.off(4);
            break;

        case VD_REV:
            pwmLeftFWD.set(VD_HAULT);
            pwmLeftREV.set(vdSpeed + VD_LEFT_ERROR);
            pwmRightFWD.set(VD_HAULT);
            pwmRightREV.set(vdSpeed + VD_RIGHT_ERROR);
            LE.on(1);
            LE.off(2);
            LE.off(3);
            LE.on(4);
            break;

        case VD_FWD_LEFT:
            if(vdSpeed == VD_FAST) {
                pwmLeftFWD.set(VD_SLOW + VD_LEFT_ERROR + VD_TURN_ERROR);
                pwmLeftREV.set(VD_HAULT);
                pwmRightFWD.set(vdSpeed + VD_RIGHT_ERROR + VD_TURN_ERROR);
                pwmRightREV.set(VD_HAULT);
            }
            else {
                pwmLeftFWD.set(VD_HAULT);
                pwmLeftREV.set(VD_HAULT);
                pwmRightFWD.set(vdSpeed + VD_RIGHT_ERROR + VD_TURN_ERROR);
                pwmRightREV.set(VD_HAULT);
            }
            LE.off(1);
            LE.on(2);
            LE.off(3);
            LE.off(4);
            break;

        case VD_FWD_RIGHT:
            if(vdSpeed == VD_FAST) {
                pwmLeftFWD.set(vdSpeed + VD_LEFT_ERROR + VD_TURN_ERROR);
                pwmLeftREV.set(VD_HAULT);
                pwmRightFWD.set(VD_SLOW + VD_RIGHT_ERROR);
                pwmRightREV.set(VD_HAULT);
            }
            else {
                pwmLeftFWD.set(vdSpeed + VD_LEFT_ERROR + VD_TURN_ERROR);
                pwmLeftREV.set(VD_HAULT);
                pwmRightFWD.set(VD_HAULT);
                pwmRightREV.set(VD_HAULT);
            }
            LE.off(1);
            LE.off(2);
            LE.on(3);
            LE.off(4);
            break;

        case VD_REV_LEFT:
            if(vdSpeed == VD_FAST) {
                pwmLeftFWD.set(VD_HAULT);
                pwmLeftREV.set(VD_SLOW + VD_LEFT_ERROR + VD_TURN_ERROR);
                pwmRightFWD.set(VD_HAULT);
                pwmRightREV.set(vdSpeed + VD_RIGHT_ERROR + VD_TURN_ERROR);
            }
            else {
                pwmLeftFWD.set(VD_HAULT);
                pwmLeftREV.set(VD_HAULT);
                pwmRightFWD.set(VD_HAULT);
                pwmRightREV.set(vdSpeed + VD_RIGHT_ERROR + VD_TURN_ERROR);
            }
            LE.on(1);
            LE.off(2);
            LE.off(3);
            LE.off(4);
            break;

        case VD_REV_RIGHT:
            if(vdSpeed == VD_FAST) {
                pwmLeftFWD.set(VD_HAULT);
                pwmLeftREV.set(vdSpeed + VD_LEFT_ERROR + VD_TURN_ERROR);
                pwmRightFWD.set(VD_HAULT);
                pwmRightREV.set(VD_SLOW + VD_RIGHT_ERROR + VD_TURN_ERROR);
            }
            else {
                pwmLeftFWD.set(VD_HAULT);
                pwmLeftREV.set(vdSpeed + VD_LEFT_ERROR + VD_TURN_ERROR);
                pwmRightFWD.set(VD_HAULT);
                pwmRightREV.set(VD_HAULT);
            }
            LE.off(1);
            LE.off(2);
            LE.off(3);
            LE.on(4);
            break;

        case VD_ALARM:
            pwmLeftFWD.set(VD_HAULT);
            pwmLeftREV.set(VD_HAULT);
            pwmRightFWD.set(VD_HAULT);
            pwmRightREV.set(VD_HAULT);
            LE.on(1);
            LE.on(2);
            LE.on(3);
            LE.on(4);
            break;

        case VD_STOP:
        default:
            pwmLeftFWD.set(VD_HAULT);
            pwmLeftREV.set(VD_HAULT);
            pwmRightFWD.set(VD_HAULT);
            pwmRightREV.set(VD_HAULT);
            LE.off(1);
            LE.off(2);
            LE.off(3);
            LE.off(4);
            break;
    }
}

static void vdBuzzer(void)
{
    if(vdState == VD_ALARM) {
        LPC_GPIO1->FIOSET = ledBit;
        delay_ms(500);
        LPC_GPIO1->FIOCLR = ledBit;
        delay_ms(500);
    }
    else {
        LPC_GPIO1->FIOCLR = ledBit;
    }
}
#endif
