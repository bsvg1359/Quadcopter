///* This File consists of the main code for the quadcopter
//*/
#include "quadcopter.h"
#include "FXAS21002.h"
#include "FXOS8700.h" 
#include "rtos.h"
#include "kalman.c"
#include "arm_math.h"

#define Rad2Dree 57.295779513082320876798154814105f
#define PID_ROLL_KP 0.0245f
#define PID_ROLL_KI 0.000175f
#define PID_ROLL_KD 0.0f
#define PID_PITCH_KP 0.0245f
#define PID_PITCH_KI 0.000175f
#define PID_PITCH_KD 0.0f
#define ROLL_SP PI/2
#define PITCH_SP PI/2

bool CALIBRATE_ESC = True;  // Make this variable "True" when when esc calibration is needed "False" otherwise
bool STOP_FLAG = True;
bool START_FLAG = False;

// Variable for getting sensor data
float accel_data[3] = {0}; 
float mag_data[3] = {0};   
float gyro_data[3]= {0};  

PwmOut ESC1(PTD1); // ESC1 is Front Left 
PwmOut ESC2(PTD3); // ESC2 is Front Right 
PwmOut ESC3(PTD2); // ESC3 is Rear Left 
PwmOut ESC4(PTD0); // ESC4 is Rear Right 
DigitalIn sw2(SW2);
DigitalIn sw3(SW3);
Serial pc(USBTX,USBRX); // Defining serial output to print in tera term
Serial bt_serial(PTC15, PTC14);

// Initialize pins for I2C communication for sensors. Set jumpers J6,J7 in FRDM-STBC-AGM01 board accordingly.
// The J6 and J7 jumoer pins must be set to SDA1 and SCL1 to read from FRDM-STBC-AGM01 insetad of onboard sensors.
FXOS8700 accel(D14,D15);
FXOS8700 mag(D14,D15);
FXAS21002 gyro(D14,D15);

// Kalman variables
float R;
kalman filter_pitch;
kalman filter_roll;
double angle[3]; // orientation 
float roll_error;
float pitch_error;
float roll;
float pitch;

// Timer Variables
Timer GlobalTime;
Timer ProgramTimer;
unsigned long timer;


void arm_pid_init_f32(
  arm_pid_instance_f32 * S,
  int32_t resetStateFlag)
{

  /* Derived coefficient A0 */
  S->A0 = S->Kp + S->Ki + S->Kd;

  /* Derived coefficient A1 */
  S->A1 = (-S->Kp) - ((float32_t) 2.0 * S->Kd);

  /* Derived coefficient A2 */
  S->A2 = S->Kd;

  /* Check whether state needs reset or not */
  if(resetStateFlag)
  {
    /* Clear the state buffer.  The size will be always 3 samples */
    memset(S->state, 0, 3u * sizeof(float32_t));
  }

}

void esc_start(void) 
{
    ESC1 = MIN_DUTY_CYCLE;
    ESC2 = MIN_DUTY_CYCLE;
    ESC3 = MIN_DUTY_CYCLE;
    ESC4 = MIN_DUTY_CYCLE;
    for (int i= 0; i< 4; i++)
    {
        ESC1 = ESC1 + 0.001f;
        ESC2 = ESC2 + 0.001f;
        ESC3 = ESC3 + 0.001f;
        ESC4 = ESC4 + 0.001f;
        wait_ms(1000);
    }
}

// Thread(s)
// Thread to print sensor data
void print_sensor_data(void const *argument) 
{
    while(1)
    {
    accel.acquire_accel_data_g(accel_data);
    printf("\r\nAccel X:%4.2f,\tAccel Y:%4.2f,\tAccel Z:%4.2f,\t\r\n",accel_data[0],accel_data[1],accel_data[2]);

    
    mag.acquire_mag_data_uT(mag_data);
    printf("Mag X:%4.2f,\tMag Y:%4.2f,\tMag Z:%4.2f,\t\r\n",mag_data[0],mag_data[1],mag_data[2]);

    
    gyro.acquire_gyro_data_dps(gyro_data);
    printf("Gyro X:%4.2f,\tGyro Y:%4.2f,\tGyro Z:%4.2f\r\n",gyro_data[0],gyro_data[1],gyro_data[2]);
    wait(1);
    }
}

// Thread to fetch the input controls for the quadcopter
void blue_control(void const *argument)
{
    //Code to control the quadcopter using bluetooth
    bt_serial.printf("***Initializing BLuetooth Control***");
    char ch = 'n'; 
    bt_serial.baud(9600);
    int count =0;
    while(True)
    {
        wait(1.0f);
        ch = bt_serial.getc();
        printf("This is the response %c\n", ch);
        
        switch(ch)
        {
            case 'w':
            {
                bt_serial.printf("Comming here %d", count);
                count = count + 1;
                START_FLAG = True;
                STOP_FLAG = False;
                // Throttle up 
                if (ESC1 + 0.001f < MAX_DUTY_CYCLE)
                {
                    ESC1 = ESC1 +  0.001f;
                    ESC2 = ESC2 +  0.001f;
                    ESC3 = ESC3 +  0.001f;
                    ESC4 = ESC4 +  0.001f;
                }
                else
                {
                    ESC1 = MAX_DUTY_CYCLE;
                    ESC2 = MAX_DUTY_CYCLE;
                    ESC3 = MAX_DUTY_CYCLE;
                    ESC4 = MAX_DUTY_CYCLE;
                        
                }
                bt_serial.printf("\r\nThrottle is currently at %3.1f%% \r\n",((ESC1- MIN_DUTY_CYCLE)/(MAX_DUTY_CYCLE - MIN_DUTY_CYCLE))*100);
                break;
            }
            case 's':
            {
                // Throttle down
                if (ESC1 - 0.005f > MIN_DUTY_CYCLE)
                {
                    ESC1 = ESC1 - 0.005f;
                    ESC2 = ESC2 - 0.005f;
                    ESC3 = ESC3 - 0.005f;
                    ESC4 = ESC4 - 0.005f;
                }
                else
                {
                    ESC1 = MIN_DUTY_CYCLE;
                    ESC2 = MIN_DUTY_CYCLE;
                    ESC3 = MIN_DUTY_CYCLE;
                    ESC4 = MIN_DUTY_CYCLE;
                        
                }
                bt_serial.printf("\r\nThrottle is currently at %3.1f%% \r\n",((ESC1- MIN_DUTY_CYCLE)/(MAX_DUTY_CYCLE - MIN_DUTY_CYCLE))*100);
                break;
                
            }
            case 'k':
            {
                STOP_FLAG = True;
                // Kill throttle 
                ESC1 = MIN_DUTY_CYCLE;
                ESC2 = MIN_DUTY_CYCLE;
                ESC3 = MIN_DUTY_CYCLE;
                ESC4 = MIN_DUTY_CYCLE;
                bt_serial.printf("\r\nThrottle is currently at %3.1f%% \r\n",((ESC1- MIN_DUTY_CYCLE)/(MAX_DUTY_CYCLE - MIN_DUTY_CYCLE))*100);
                break;
            }
            case 'r':
            {
                STOP_FLAG = False;
                break;    
            }
            default:
            {
                break;
            }
        }       
    }
}

// Function to apply the Pitch and Roll to ESC
float esc_control(float current, float pid, float rate)
{
    if ((current + pid+ rate) > MAX_DUTY_CYCLE) return MAX_DUTY_CYCLE;
    else if ((current + pid+ rate) < MIN_DUTY_CYCLE) return MIN_DUTY_CYCLE;
    else return current + pid+ rate;
}

int main() 
{   
    printf("Starting");
    Thread bc(blue_control);
    // Instantiating PID controller
    arm_pid_instance_f32 RPID;
    arm_pid_instance_f32 PPID;
    //Pitch
    PPID.Kp= PID_PITCH_KP/1000.0f; // Proporcional
    PPID.Ki= PID_PITCH_KI/1000.0f; // Integral
    PPID.Kd= PID_PITCH_KD/1000.0f; // Derivative
    //Roll
    RPID.Kp= PID_ROLL_KP/1000.0f; // Proporcional
    RPID.Ki= PID_ROLL_KI/1000.0f; // Integral
    RPID.Kd= PID_ROLL_KD/1000.0f; // Derivative
    arm_pid_init_f32(&RPID, 1);
    arm_pid_init_f32(&PPID, 1);
    
    // Initialize the Global Timer
    GlobalTime.start();
    
    // Instantiating Kalman filter
    kalman_init(&filter_pitch, R_matrix, Q_Gyro_matrix, Q_Accel_matrix);
    kalman_init(&filter_roll, R_matrix, Q_Gyro_matrix, Q_Accel_matrix);
    ESC1.period_ms(PWM_TIME_PERIOD);
    if (CALIBRATE_ESC)
    {
        esc_calibration(); 
    }
    // Initialize Duty Cycle to minimum
    ESC1 = MIN_DUTY_CYCLE;
    ESC2 = MIN_DUTY_CYCLE;
    ESC3 = MIN_DUTY_CYCLE;
    ESC4 = MIN_DUTY_CYCLE;
    
    // Initialize accelerometer, Magnetometer and Gyrosope 
    accel.accel_config();
    mag.mag_config();
    gyro.gyro_config();
    
    esc_start();
    
    ProgramTimer.start();
    timer = ProgramTimer.read_us();
    while(START_FLAG)
    {
        accel.acquire_accel_data_g(accel_data);
        mag.acquire_mag_data_uT(mag_data);
        gyro.acquire_gyro_data_dps(gyro_data);
        R = sqrt(std::pow(accel_data[0], 2) + std::pow(accel_data[1], 2) + std::pow(accel_data[2], 2));
        kalman_predict(&filter_pitch, gyro_data[0], (ProgramTimer.read_us() - timer));
        kalman_update(&filter_pitch, acos(accel_data[0]/R));
        kalman_predict(&filter_roll, gyro_data[1], (ProgramTimer.read_us() - timer));
        kalman_update(&filter_roll, acos(accel_data[1]/R));
        angle[0] = kalman_get_angle(&filter_pitch);
        angle[1] = kalman_get_angle(&filter_roll);
        
        if (angle[0] > PI) 
        {
            angle[0] = PI;
        }
        else 
        {
            if (angle[0] < 0) 
            {
                angle[0] = 0.0f;
            }
        }
        
        if (angle[1] > PI) 
        {
            angle[1] = PI;
        }
        else 
        {
            if (angle[1] < 0) 
            {
                angle[1] = 0.0f;
            }
        }
        
        pitch_error = angle[0] - PITCH_SP;
        roll_error = angle[1] - ROLL_SP;
        pitch = arm_pid_f32(&PPID, pitch_error);
        roll = arm_pid_f32(&RPID, roll_error);
        
        timer = ProgramTimer.read_us();
        
        if (!STOP_FLAG)
        {
            ESC1 = esc_control(ESC1, -pitch/2, 0.0f);
            ESC2 = esc_control(ESC2, -pitch/2, 0.0f);
            ESC1 = esc_control(ESC1, -roll/2, 0.0f);
            ESC3 = esc_control(ESC3, -roll/2, 0.0f);
            ESC2 = esc_control(ESC2, roll/2, 0.0f);
            ESC4 = esc_control(ESC4, roll/2, 0.0f);
            ESC3 = esc_control(ESC3, pitch/2, 0.0f);
            ESC4 = esc_control(ESC4, pitch/2, 0.0f);
        }
        wait(0.02f);
    }
    
}
 
 

