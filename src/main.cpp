#include "mbed.h"
 
// pes board pin map
 
#include "PESBoardPinMap.h"
 
// drivers


#include "DebounceIn.h" 
#include "DCMotor.h" 
#include "DigitalIn.h" 
#include "FastPWM.h" 
#include "Servo.h"
#include "SensorBar.h"
#include "LineFollower.h"
 
bool do_execute_main_task = false; // this variable will be toggled via the user button (blue button) and
                                   // decides whether to execute the main task or not
 
bool do_reset_all_once = false;    // this variable is used to reset certain variables and objects and
                                   // shows how you can run a code segment only once
// objects for user button (blue button) handling on nucleo board
DebounceIn user_button(BUTTON1);   // create DebounceIn to evaluate the user button
void toggle_do_execute_main_fcn(); // custom function which is getting executed when user
                                   // button gets pressed, definition below
                                   // mechanical button


                                   // main runs as an own thread

bool REMARK = false; //- remark flag for enigneering      

bool oncethrough = false;

int main() 
{ 

    //- definine constants:
    const float voltage_max = 12.0f; // maximum voltage of battery packs, adjust this to // 6.0f V if you only use one battery pack // motor M3                                       
    const float gear_ratio_ALL = 78.125f; // gear ratio 
    const float MOTOR_CONSTANT_ALL = 180.0f / 12.0f;  // motor constant [rpm/V]  // it is assumed that only one motor is available, there fore  // we use the pins from M1, so you can leave it connected to M1 
    const float parSpeedStDrive = 0.3f;
    const float parSpeedStFollow = 0.3f;
    const int printcycle = 1000; // 1 sec
    const int pulluptime = 2000; // 1 sec
    const int totaltimecheckstop = 30000; // 30 sec

    const float servoupPos = 0.45f;
    const float servoDownPos = 0.76f;



    //- define objects:
    DigitalIn mechanical_button(PC_6); // belegung siehe foto cyril
    DigitalIn mechanical_RopeDet(PC_5); // belegung siehe foto cyril
    DigitalOut enable_motors(PB_15); 
    
    //DC Motoren 
    DCMotor motor_right(PB_PWM_M3, PB_ENC_A_M3, PB_ENC_B_M3, gear_ratio_ALL, MOTOR_CONSTANT_ALL, voltage_max); 
    DCMotor motor_left(PB_PWM_M2, PB_ENC_A_M2, PB_ENC_B_M2, gear_ratio_ALL, MOTOR_CONSTANT_ALL, voltage_max); 
    DigitalOut user_led(LED1);
    DigitalOut led1(PB_9); // belegung siehe foto cyril

    //- parameterize objects:    
    mechanical_button.mode(PullUp);  
    mechanical_RopeDet.mode(PullUp);
    //- actors
    //motor_right.enableMotionPlanner(); // enable the motion planner for smooth movement 
    motor_right.setMaxAcceleration(motor_right.getMaxAcceleration() * 0.5f); // limit max. acceleration to half of the default acceleration 
    //motor_left.enableMotionPlanner(); // enable the motion planner for smooth movement 
    motor_left.setMaxAcceleration(motor_left.getMaxAcceleration() * 0.5f); // limit max. acceleration to half of the default acceleration   

    //- Line following:

    const float bar_dist = 0.13f; // distance from wheel axis to leds on sensor bar / array in meters
    SensorBar sensorBar(PB_9, PB_8, bar_dist);
    const float d_wheel = 0.04f; // wheel radius in meters
    const float b_wheel = 0.17f;          // wheelbase, distance from wheel to wheel in meter
    const float Kp = 1.0f * 3.0f;
    const float Kp_nl = 1.0f * 17.0f;

    LineFollower lineFollower(PB_9, PB_8, bar_dist, d_wheel, b_wheel, motor_right.getMaxPhysicalVelocity());
    lineFollower.setMaxWheelVelocityRPS(parSpeedStFollow);
    lineFollower.setRotationalVelocityGain(Kp, Kp_nl);


    //- Servo
    Servo servo1(PB_D0);
//- teached values:
    const float servo1_ang_min = 0.035f; // carefull, these values might differ from servo to servo
    const float servo1_ang_max = 0.120f;
    //-
    servo1.calibratePulseMinMax(servo1_ang_min, servo1_ang_max);
    servo1.setMaxAcceleration(0.3f);
    //- initialisze up position
    float servo_input = servoupPos;
// set up states for state machine
 
enum RobotStep { 
    ST_OFF,
    ST_INIT, 
    ST_FOLLOW, 
    ST_DRIVE, 
    ST_PULLUP, 
    ST_DROPDOWN
} robot_step = RobotStep::ST_OFF; // init step:


enum RobotSubStep { 
    SUB_NONE,
    SUB_START,
    SUB_INTERMED,
    SUB_END,
    SUB_LINE,
    SUB_PLATFORM,
    SUB_ROPE,
}robot_substep = RobotSubStep::SUB_NONE; // init step:
// 
    // attach button fall function address to user button object
 
    user_button.fall(&toggle_do_execute_main_fcn);
 
    // Timers
    Timer print_timer;
    Timer main_task_timer;   
    Timer tmrPullup;   // delay to pullup the linefollower
    Timer tmrtotalTime;   // delay to pullup the linefollower
    print_timer.start();
    main_task_timer.start();
    const int main_task_period_ms = 20;
 
    // this loop will run forever
 
    while (true) {
 
        main_task_timer.reset();
 
        if (do_execute_main_task) {
 
            led1 = 1;           

static bool prevRopeDet = mechanical_RopeDet.read(); //- init by sensor value
bool currentRopeDet = mechanical_RopeDet.read();
bool outFallingEdgeRope = (prevRopeDet == 1 && currentRopeDet == 0);
bool outRisingEdgeRope = (prevRopeDet == 0 && currentRopeDet == 1);
prevRopeDet = currentRopeDet; // store for next cycle

if (outFallingEdgeRope){
    printf("Falling Edge");
}
if (outRisingEdgeRope){
    printf("Rising Edge");
}

bool isInFinishRange = outRisingEdgeRope && (robot_step == ST_DRIVE) && (tmrtotalTime.read_ms() >= totaltimecheckstop); // ensure after rising edge

//- servo:
servo1.setPulseWidth(servo_input);
if (!servo1.isEnabled())
servo1.enable();

// *******************STATE MACHINE**********************************************
switch (robot_step) {
 
    case RobotStep::ST_OFF: {
        
        enable_motors = 0;    
        // Transition: 
        if (!isInFinishRange && !oncethrough){ // mechanical_button.read()) {
            robot_step = RobotStep::ST_INIT;
        }
        break;
    }
    
    case RobotStep::ST_INIT: {
        
        enable_motors = 1;
        if (!servo1.isEnabled())
        servo1.enable();
        servo_input = servoDownPos;
        // Transition: 
        if (true){ // old if (mechanical_button.read()) {
            printf("Transition to Step: STFollow\n");
            robot_step = RobotStep::ST_FOLLOW;
        }
        break;
    }
    
    case RobotStep::ST_FOLLOW: {
        // Some logic for finding target or condition

        //motors
        enable_motors = currentRopeDet == 1;  
        // linefollower to motor:
        motor_left.setVelocity(lineFollower.getLeftWheelVelocity()) ; 
        motor_right.setVelocity(lineFollower.getRightWheelVelocity());
        servo_input = servoDownPos;

        
        //linesensor

        // only update sensor bar angle if a led is triggered
        if (sensorBar.isAnyLedActive())
            
        // Transition: 
        if (outFallingEdgeRope) { 
            robot_step = RobotStep::ST_PULLUP;
            printf("Transition to Step: ST_PULLUP\n");
            printf("By Edge Detection");
        }
        break;
    }
    
    case RobotStep::ST_DRIVE: {
        // Driving logic or activation of drive motors
        //motors
        enable_motors = 1;  
        // linefollower to motor:
        motor_right.setVelocity(parSpeedStDrive); 
        motor_left.setVelocity(parSpeedStDrive);
        //- servo:
        servo_input = servoupPos;
        //- check substep:
        if(robot_substep == RobotSubStep::SUB_PLATFORM){
        }        
        // Transition: 
        if (isInFinishRange) { 
            robot_step = RobotStep::ST_OFF;
            printf("Transition to Step: StOff\n");
            printf("Finished!!! ");
            tmrtotalTime.reset();
            oncethrough = true;
        
            if(REMARK){
            robot_substep = RobotSubStep::SUB_INTERMED;
            printf("Transition to Substep: Platform\n");
        }
        }
        break;
    }
    
    case RobotStep::ST_PULLUP: {
        // Backward pull-up logic
        enable_motors = 0;   //-motoroff!
        servo_input = servoupPos;
        tmrPullup.start();
        // Transition: 
        if (tmrPullup.read_ms() >= pulluptime) { 
            robot_step = RobotStep::ST_DRIVE;
            printf("Transition to Step: StDrive\n");
            printf("Pulluptimerdone\n");  
            tmrPullup.reset();
            tmrtotalTime.start();
        }
        break;
    }
    
    case RobotStep::ST_DROPDOWN: {
        // Final action logic (e.g., dropping something down)
        enable_motors = 1;  
    
        // Transition: 
        if (REMARK) { 
            robot_step = RobotStep::ST_OFF;
        }
        break;
    }  
     
    default: {

        break; // do nothing
    }
    }
    }
    else {
        // the following code block gets executed only once
        if (do_reset_all_once) {
            do_reset_all_once = false;
            oncethrough = false;
            tmrtotalTime.reset();

            // reset variables and objects
            led1 = 0;
            enable_motors = 0;
            motor_left.setMotionPlanerPosition(0.0f);
            motor_left.setMotionPlanerVelocity(0.0f);
            //motor_left.enableMotionPlanner();
            motor_right.setMotionPlanerPosition(0.0f);
            motor_right.setMotionPlanerVelocity(0.0f);
            //motor_right.enableMotionPlanner();
            robot_step = RobotStep::ST_INIT;
            servo1.disable();
        }
    }
 

        // toggling the user led
        user_led = !user_led;

        // Print messages only if 1 second has elapsed since the last print
        if (print_timer.read_ms() >= printcycle) {
            printf("Step: ");
            switch (robot_step) {
                case RobotStep::ST_OFF:       printf("OFF\n"); break;
                case RobotStep::ST_INIT:      printf("INIT\n"); break;
                case RobotStep::ST_FOLLOW:      printf("FOLLOW\n"); break;
                case RobotStep::ST_DRIVE:     printf("DRIVE\n"); break;
                case RobotStep::ST_PULLUP:    printf("PULLUP\n"); break;
                case RobotStep::ST_DROPDOWN:   printf("DROPDOWNS\n"); break;
                default:                    printf("Unknown\n"); break;
            }
            printf("Substep: ");
            switch (robot_substep) {
                case RobotSubStep::SUB_NONE:       printf("NONE\n"); break;
                case RobotSubStep::SUB_START:      printf("START\n"); break;
                case RobotSubStep::SUB_INTERMED:   printf("INTERMED\n"); break;
                case RobotSubStep::SUB_END:        printf("END\n"); break;
                case RobotSubStep::SUB_LINE:       printf("LINE\n"); break;
                case RobotSubStep::SUB_PLATFORM:   printf("PLATFORM\n"); break;
                case RobotSubStep::SUB_ROPE:       printf("ROPE\n"); break;
                default:                         printf("Unknown Substep\n"); break;
            }

            //printf("DC Motor RIGHT Rotations: %f\n", motor_right.getRotation());
            //printf("DC Motor LEFT Rotations: %f\n", motor_left.getRotation());
            //printf("ANGLE: %f\n", angle);
            //printf("linefolowwer rigth: %f\n", lineFollower.getRightWheelVelocity());
            //printf("linefolowwer left: %f\n", lineFollower.getLeftWheelVelocity());
            printf("Servo Setpoint %f\n", servo_input);
            printf("Total Time: %lld", duration_cast<milliseconds>(tmrtotalTime.elapsed_time()).count());
            // Reset the print timer
            print_timer.reset();
        }
        // read timer and make the main thread sleep for the remaining time span (non blocking)
        int main_task_elapsed_time_ms = duration_cast<milliseconds>(main_task_timer.elapsed_time()).count();
        if (main_task_period_ms - main_task_elapsed_time_ms < 0)
            printf("Warning: Main task took longer than main_task_period_ms\n");
        else
            thread_sleep_for(main_task_period_ms - main_task_elapsed_time_ms); //- define cycle time by sleep
    }
}

void toggle_do_execute_main_fcn()
{
    // toggle do_execute_main_task if the button was pressed
    do_execute_main_task = !do_execute_main_task;
    // set do_reset_all_once to true if do_execute_main_task changed from false to true
    if (do_execute_main_task)
        do_reset_all_once = true;
} 