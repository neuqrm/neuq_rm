#ifndef FRIC_H
#define FRIC_H

#define Fric_UP 1400
#define Fric_DOWN 1300
#define Fric_OFF 1000

/***** Ħ���ֿ����� *****/
#define FRIC1_Speed Kinematics.fric1.target_angular
#define FRIC2_Speed Kinematics.fric2.target_angular

typedef signed char int8_t;
typedef signed short int int16_t;
typedef signed int int32_t;
typedef signed long long int64_t;

typedef unsigned char uint8_t;
typedef unsigned short int uint16_t;
typedef unsigned int uint32_t;
typedef unsigned long long uint64_t;
typedef unsigned char bool_t;
typedef float fp32;
typedef double fp64;
void auto_fire(void);
void fric_control_close(int fire1_speed,int fire2_speed);
extern void fric_PWM_configuration(void);
extern void fric_off(void);
extern void fric1_on(uint16_t cmd);
extern void fric2_on(uint16_t cmd);
#endif



