#include "detumble_algo.h"
/*
TODO: Make provision for online tuning of params: 
maybe use python to update a settings file
check accuracy of float/ undersaturating points
*/


void main(int argc, char *argv[]) 
{
	if (argc < 2) {
		printf("ERR: supply arguments correctly \n");
		exit(0);
	}
	else {
		for (int i = 0; i <= argc; i++) {
			printf("%d: %s \n", i, argv[i]);
		}
	}
	FILE *csv_input;
	char buf[LINE_BUFFER];
	csv_input = fopen(argv[1], "r");
	int cnt = 0;
	char *token = 0;
	while(fgets(buf, sizeof(buf), csv_input)) {
		printf("line %d: ", cnt++);
		token = strtok(buf, ",");
		while(token) {
			printf("%s\t", token);
			token = strtok(NULL, ",");
		}
		printf("\n");
	}
	fclose(csv_input);
}

int8_t T1[3][3] = {1, 0, 0, 0, 1, 0, 0, 0, -1};
int8_t T2[3][3] = {1, 0, 0, 0, 1, 0, 0, 0, -1};
/*
 * Description: performs step3 of detumbling on ADCS - Sensor readings and fusion
 * input:  weights, raw and bias values of each sensor
 * output: normalized and fused magnetic field readings.
*/
vector_t step3_biasCalc(float w1, vector_t b1_raw, vector_t b1_bias, float w2, vector_t b2_raw, vector_t b2_bias)
{
	//vector_t b1_cur; b2_cur; b1_bias; b2_bias; b1_raw; b2_raw, b_cur;
	vector_t b1_cur, b2_cur, b_cur, b_cur_norm;
	/*equation 1a*/
	b1_cur.x = T1[0][0]*b1_raw.x + T1[0][1]*b1_raw.y + T1[0][2]*b1_raw.z + b1_bias.x; 
	b1_cur.y = T1[1][0]*b1_raw.x + T1[1][1]*b1_raw.y + T1[1][2]*b1_raw.z + b1_bias.y; 
	b1_cur.z = T1[2][0]*b1_raw.x + T1[2][1]*b1_raw.y + T1[2][2]*b1_raw.z + b1_bias.z; 

	/*equation 1b*/
	b2_cur.x = T2[0][0]*b2_raw.x + T2[0][1]*b2_raw.y + T2[0][2]*b2_raw.z + b2_bias.x; 
	b2_cur.y = T2[1][0]*b2_raw.x + T2[1][1]*b2_raw.y + T2[1][2]*b2_raw.z + b2_bias.y; 
	b2_cur.z = T2[2][0]*b2_raw.x + T2[2][1]*b2_raw.y + T2[2][2]*b2_raw.z + b2_bias.z; 

	/* equation 2 */
	b_cur.x = w1*b1_cur.x + w2*b2_cur.x;
	b_cur.y = w1*b1_cur.y + w2*b2_cur.y;
	b_cur.z = w1*b1_cur.z + w2*b2_cur.z;

	float temp_norm = sqrt(b_cur.x*b_cur.x + b_cur.y*b_cur.y + b_cur.z*b_cur.z);
	b_cur_norm.x = b_cur.x / temp_norm;
	b_cur_norm.y = b_cur.y / temp_norm;
	b_cur_norm.z = b_cur.z / temp_norm;

	return b_cur_norm;
}

float Tc = 0.25; // 4 Hz, 250 ms
/*
 * Description: performs step4 of the detumbling on ADCS - Bdot calculation
 * input: b_cur, b_prev, b_cur_norm, b_prev_norm
 * output: b_dot, b_dot_norm
 */
void step4_bdotCalc(vector_t b_cur, vector_t b_cur_norm, vector_t b_prev, vector_t b_prev_norm, 
					vector_t* b_dot, vector_t* b_dot_norm)
{
	static bool first_call = true;
	if (first_call == true) {
		b_dot->x      = 0;
		b_dot->y      = 0;
		b_dot->z      = 0;
		b_dot_norm->x = 0;
		b_dot_norm->y = 0;
		b_dot_norm->z = 0;
		first_call = false;
	} 
	// make sure Tc is 250 ms
	else {
		b_dot->x = (b_cur.x - b_prev.x)/Tc;
		b_dot->y = (b_cur.y - b_prev.y)/Tc;
		b_dot->z = (b_cur.z - b_prev.z)/Tc;

		b_dot_norm->x = (b_cur_norm.x - b_prev_norm.x)/Tc;
		b_dot_norm->y = (b_cur_norm.y - b_prev_norm.y)/Tc;
		b_dot_norm->z = (b_cur_norm.z - b_prev_norm.z)/Tc;
	}
}


float alpha = 0.01; 
/*
 * Description: performs step5 of the detumbling algorithm - tumble parameter update
 * input: alpha, p_tumb, b_dot
 * output: tumble parameter
 */
vector_t step5_tumbleParam(vector_t b_dot) 
{
	// TODO: check if it works, the static part
	static vector_t p_tumb = {0.00001, 0.00001, 0.00001};
	static vector_t b_dot_old; 
	
	if (b_dot_old.x != b_dot.x || b_dot_old.y != b_dot.y || b_dot_old.z != b_dot.z) {
		p_tumb.x = alpha*abs(b_dot.x) + (1-alpha)*p_tumb.x;
		p_tumb.y = alpha*abs(b_dot.y) + (1-alpha)*p_tumb.y;
		p_tumb.z = alpha*abs(b_dot.z) + (1-alpha)*p_tumb.z;
		b_dot_old = b_dot;
	}
	return p_tumb;
}

float p_bar = 0.000002;

/*
 * Description: performs step6 of the detumbling algorithm - counter update
 * input: p_tumb
 * output: c_tumble
 */
void step6_countUpdate(vector_t p_tumb)
{
	static int c_tumble;
	if (p_tumb.x <= p_bar && p_tumb.y <= p_bar && p_tumb.z <= p_bar) {
		c_tumble = c_tumble + 1;
	}
	else {
		c_tumble = 0;
	}

}

float t_conf = 3600;
vector_t m_pol = {1, 1, 1};
/*
 * Description: performs step7 of the detumbling algorithm - decision to actuate
 * input: c_tumble
 * output: branch to another subroutine
 */
void step7_actuationDecision(int c_tumble, vector_t b_cur, vector_t b_dot_norm)
{
	vector_t m_des;
	vector_t t_on;
	if (c_tumble*Tc < t_conf) {
		m_des = step8_controlCalc(b_cur, b_dot_norm);
		t_on = step9_torqueActuate(m_des, m_pol);
		// todo: actuate
	}
	else {
		step10_hold();
	}
}

float k_w = 0.000001422;
/*
 * Description: performs step8 of the detumbling algorithm - how much to actuate
 * input: k_w, b_cur, d_dot_norm
 * output: desired magnetic moment
 */
vector_t step8_controlCalc(vector_t b_cur, vector_t b_dot_norm)
{
	vector_t m_des;
	float temp_norm = sqrt(b_cur.x*b_cur.x + b_cur.y*b_cur.y + b_cur.z*b_cur.z);
	m_des.x = -k_w * b_dot_norm.x / temp_norm;
	m_des.y = -k_w * b_dot_norm.y / temp_norm;
	m_des.z = -k_w * b_dot_norm.z / temp_norm;
	return m_des;
}

float delta = 0.6;
float Ta;

vector_t m_max = {0.002, 0.002, 0.002};

/*
 * Description: performs step9 of the detumbling algorithm - actually actuate
 * input: desired magnetic moment, polarity
 * output: t_on
 */
vector_t step9_torqueActuate(vector_t m_des, vector_t m_pol)
{
	vector_t t_on;
	vector_t s_on;
	Ta = delta*Tc;
	// magnitude
	t_on.x = Ta * MIN(1, abs(m_des.x)/m_max.x);
	t_on.y = Ta * MIN(1, abs(m_des.y)/m_max.y);
	t_on.z = Ta * MIN(1, abs(m_des.z)/m_max.z);

	// direction
	s_on.x = m_pol.x * SGN(m_des.x);
	s_on.y = m_pol.y * SGN(m_des.y);
	s_on.z = m_pol.z * SGN(m_des.z);

	// TODO: add actuation signals
}

float Ts = 0.02;
/*
 * Description: Hold for desaturate/hysterisis 
 * input: control, hold and actuation time
 * output: stop processor in the meantime
 */
float step10_hold()
{
	// delay(Ta+Tc+Ts);
}
