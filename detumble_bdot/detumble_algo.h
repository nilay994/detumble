/*
 * reference: Robert's algorithm doc v0.2
 *----------------------------------------------------Detumble initialization----------------------------------------------
 *--------------------------------------------------+-----------------------+------------------------+-----------+---------+
 * description                                      | variable              | initial value          | dimension | unit    |
 * ctrl loop freq preset                            | detumble_ctrl_freq    | 4                      | 1         | Hz      |
 * control loop period                              | detumble_period_Tc    | 250                    | 1         | ms      |
 * duty cycle of mtq                                | detumble_duty_mtq     | 0.6                    | 1         | none    |
 * max actuation period                             | detumble_period_Ta    | deltaTc (TBD?)         | 1         | sec     |
 * B-dot gain                                       | detumble_kw           | 1.1422*10^-6           | 1         | Am^2s/T |
 * max dipole moment x, y, z                        | detumble_max_moment   | 0.002                  | 1         | Am^2    |
 * torque rod polarity x, y, z {-1, 1}              | detumble_rod_polarity | 1                      | 1         | -       |
 * weight factor mtm1,2                             | detumble_mag1_weight  | 0.5                    | 1         | -       |
 * bias vector mtm1,2                               | detumble_x/y/zbias    | {0,0,0}/code o/p       | 3x1       | T       |
 * scaling/orientation                              | detumble_T1           | [1 0 0; 0 1 0; 0 0 -1] | 3x3       | -       |
 * tumble parameter (complementary) filter constant | detumble_alpha        | 0.01                   | 1         | -       |
 * tumble parameter threshold                       | detumble_pbar         | 2*10^-6                | 1         | T/s     |
 * detumble confirmation time                       | detumble_tconf        | 3600                   | 1         | sec     |
 * tumble parameter                                 | detuble_p_tumb        | {10^-5, 10^-5, 10^-5}  | 3x1       | T/s     |
 * counter                                          | detumble_count_tumb   | 0                      | 1         | -       |
 * desired magnetic dipole moment                   | detumble_m_desired    | (code o/p)             | 3x1       | Am2     |
 * corrected vector (mag1 and mag2)                 | detumble_b1/b2_curr   | (code o/p)             | 3x1       | T       |
 * current weighted vector                          | detumble_b_curr       | (code o/p)             | 3x1       | T       |
 * current normalized vector                        | detumble_bnorm_curr   | (code o/p)             | 3x1       | T       |
 * previous meas. vector                            | detumble_b_prev       | (code o/p)             | 3x1       | T       |
 * bdot vector                                      | detumble_b_dot        | (code o/p)             | 3x1       | T       |
 * normalized_bdot                                  | detumble_b_normdot    | (code o/p)             | 3x1       | T       |
 * hold period                                      | detumble_t_hold       | N/A                    | N/A       | sec     |
 * raw measurement vector (mtm1/2)                  | detumble_raw_mtm1/2   | (code o/p)             | 3x1       | T       |
 * vector of torque rods on times                   | detumble_t_on         | (code o/p)             | 3x1       | s       |
 * vector of torque rods on directions              | detumble_s_on {-1, 1} | (gnd station flip)     | 1         | -       |
 *--------------------------------------------------+-----------------------+------------------------+-----------+---------+
 *
 **/
#ifndef _DETUMBLE_ALGO_H
#define _DETUMBLE_ALGO_H

#ifdef __cplusplus
extern "C" {
#endif

#define LINE_BUFFER 500 // 200 char per line

typedef signed char bool;
#define true 1
#define false 0

// math macros
#define MIN(a,b) (((a)<(b))?(a):(b))
#define SGN(a)	 (((a)<(0))?(-1):(1))	

typedef struct {
	double x; 
	double y; 
	double z;
} vector_t;

void step3_biasCalc(double w1, vector_t b1_raw, vector_t b1_bias, double w2, vector_t b2_raw, vector_t b2_bias, vector_t* b_cur, vector_t* b_cur_norm);
void step4_bdotCalc(vector_t b_cur, vector_t b_cur_norm, vector_t b_prev, vector_t b_prev_norm, vector_t* b_dot, vector_t* b_dot_norm);
vector_t step5_tumbleParam(vector_t b_dot);
int step6_countUpdate(vector_t p_tumb);
void step7_actuationDecision(int c_tumble, vector_t b_cur, vector_t b_dot_norm, vector_t* t_on, vector_t* s_on);
vector_t step8_controlCalc(vector_t b_cur, vector_t b_dot_norm);
void step9_torqueActuate(vector_t m_des, vector_t m_pol, vector_t* t_on, vector_t* s_on);
double step10_hold();

void controlLoop(vector_t b1_raw, vector_t b2_raw, vector_t* s_on, vector_t* t_on, vector_t* p_tumb);

#ifdef __cplusplus
}
#endif

#endif