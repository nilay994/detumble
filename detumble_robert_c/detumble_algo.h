#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>


#define LINE_BUFFER 200 // 200 char per line

typedef int8_t bool;
#define true 1
#define false 0

// math macros
#define MIN(a,b) (((a)<(b))?(a):(b))
#define SGN(a)	 ((a)<(0)?(-1):(1))

typedef struct {
	float x; 
	float y; 
	float z;
} vector_t;


vector_t step3_bias(float w1, float w2, vector_t b1_raw, vector_t b2_raw, vector_t b1_bias, vector_t b2_bias); 
void step4_bdotCalc(vector_t b_cur, vector_t b_cur_norm, vector_t b_prev, vector_t b_prev_norm, vector_t* b_dot, vector_t* b_dot_norm);
vector_t step5_tumbleParam(vector_t b_dot);
void step6_countUpdate(vector_t p_tumb);
void step7_actuationDecision(int c_tumble, vector_t b_cur, vector_t b_dot_norm);
vector_t step8_controlCalc(vector_t b_cur, vector_t b_dot_norm);
vector_t step9_torqueActuate(vector_t m_des, vector_t m_pol);
float step10_hold();