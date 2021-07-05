/* This File consists code to calibrate ESC
Need USB power to run this funciton
*/
#include "quadcopter.h"

int sw2_press_count=0;
extern PwmOut ESC1; // ESC1 is Front Left 
extern PwmOut ESC2; // ESC2 is Front Right 
extern PwmOut ESC3; // ESC3 is Rear Left 
extern PwmOut ESC4; // ESC4 is Rear Right 
extern DigitalIn sw2;
extern DigitalIn sw3;
void esc_calibration(void) 
{
     //The PWMs all share the same period - if you change the period for one, you change it for all. 
    ESC1 = 0.00f; // Setting the min duty cycle for the ESC to appropriate percentage
    ESC2 = 0.00f; // Setting the min duty cycle for the ESC to appropriate percentage
    ESC3 = 0.00f; // Setting the min duty cycle for the ESC to appropriate percentage
    ESC4 = 0.00f; // Setting the min duty cycle for the ESC to appropriate percentage 
    printf("\r\n Starting ESC Calibration\r\n  !!!! REMOVE THE PROPELLERS AND DISCONNECT THE BATTERY BEFORE PROCEDING FURTHER !!!!\r\n");        
    printf("\r\n Press SW2 to set max duty cycle for calibration \r\n");
    while(True)
    {
        if(sw2==0)
        {           
           if (sw2_press_count==1) // Setting min duty cycle
           {
               ESC1 = MIN_DUTY_CYCLE; // Setting the min duty cycle for the ESC for calibration
               ESC2 = MIN_DUTY_CYCLE; // Setting the min duty cycle for the ESC for calibration
               ESC3 = MIN_DUTY_CYCLE; // Setting the min duty cycle for the ESC for calibration
               ESC4 = MIN_DUTY_CYCLE; // Setting the min duty cycle for the ESC for calibration
               printf("\r\n Disconnect and Reconnect the battery \r\n");
               printf("\r\n End of Calibration \r\n");
               printf("\r\n Press button SW3 to test the motor min max range\r\n ");
               sw2_press_count = 2;
               ThisThread::sleep_for(std::chrono::seconds(2));
            }
            if (sw2_press_count==0) // Setting max duty cycle
            {
               ESC1 = MAX_DUTY_CYCLE; // Setting the max duty cycle for the ESC for calibration
               ESC2 = MAX_DUTY_CYCLE; // Setting the max duty cycle for the ESC for calibration
               ESC3 = MAX_DUTY_CYCLE; // Setting the max duty cycle for the ESC for calibration
               ESC4 = MAX_DUTY_CYCLE; // Setting the max duty cycle for the ESC for calibration
               printf("\r\n 1)Connect the battery\r\n 2)Press SW2 after 2 beeps from motor \r\n");
               sw2_press_count = 1;
               ThisThread::sleep_for(std::chrono::seconds(2));
            }
        }
        if(sw3==0)
        {
            for(float val = MIN_DUTY_CYCLE; val <= MAX_DUTY_CYCLE; val+=0.01f)
            {
                printf("\r\nThis the the duty cycle %f \r\n\r\n", val);
                ESC1 = val;
                ESC2 = val;
                ESC3 = val;
                ESC4 = val;
                ThisThread::sleep_for(std::chrono::seconds(1));
            }    
                
            for(float val = MAX_DUTY_CYCLE; val >= MIN_DUTY_CYCLE; val-=0.01f)
            {
                printf("\r\nThis the the duty cycle %f\r\n\r\n", val);
                ESC1 = val;
                ESC2 = val;
                ESC3 = val;
                ESC4 = val;
                ThisThread::sleep_for(std::chrono::seconds(1));
            }  
            printf("\r\n End of of min max range test\r\n ");
            break;
            
        }
    }
}

