#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <float.h>
#include "detumble_algo.h"
/*
TODO: Make provision for online tuning of params: 
maybe use python to update a settings file
check accuracy of double/ undersaturating points
*/


void main(int argc, char *argv[]) 
{
	if (argc < 2) {
		printf("ERR: supply arguments correctly \n");
		// exit(0);
	}
	else {
		for (int i = 0; i <= argc; i++) {
			printf("%d: %s \n", i, argv[i]);
		}
	}
	FILE *csv_input;
	FILE *csv_output;
	char buf[LINE_BUFFER] = { 0 };
	//csv_input = fopen(argv[1], "r");
	csv_input = fopen("data4test_02_in.csv", "r");
	if (csv_input == NULL) {
		printf("failed to open input csv\n");
	}

	csv_output = fopen("data4test_02_out.csv", "w");
	if (csv_output == NULL) {
		printf("failed to open output csv\n");
	}

	int row = 2;
	char *token = 0;
	int column = 0;
	vector_t mag1Data;
	vector_t mag2Data;

	double parse_row[15];

	// TODO: strcpy can populate array from strings mostly.

	// strip the header
	fgets(buf, sizeof(buf), csv_input); 

	// print header yourself
	//printf("\033[1;31m\n");

	//printf("mag1.x\t\tmag1.y\t\tmag1.z\t\tmag2.x\t\tmag2.y\t\tmag2.z\t\tt_on.x\t\tt_on.y\t\tt_on.z\t\ts_on.x\t\ts_on.y\t\ts_on.z\t\tp_tumb.x\t\tp_tumb.y\t\tp_tumb.z\n");
	fprintf(csv_output, "mag1.x,mag1.y,mag1.z,mag2.x,mag2.y,mag2.z,t_on.x,t_on.y,t_on.z,s_on.x,s_on.y,s_on.z,p_tumb.x,p_tumb.y,p_tumb.z\n");
	//printf("\033[0m\n");
	// read from row 2
	while (fgets(buf, sizeof(buf), csv_input)) {
		// printf("row %d: ", row ++);
		row ++;
		// initialize buffer for each line and split the string by ","
		token = strtok(buf, ",");


		// populate the readings
		while(token) {

			parse_row[column] = atof(token);
			// printf("%f\t", parse_row[column]);

			token = strtok(NULL, ",");  // reset the static pointer
			column ++;

		}
		
		mag1Data.x = parse_row[0];
		mag1Data.y = parse_row[1];
		mag1Data.z = parse_row[2];
		mag2Data.x = parse_row[3];
		mag2Data.y = parse_row[4];
		mag2Data.z = parse_row[5];
		vector_t s_on;
		vector_t t_on;
		vector_t p_tumb;
		controlLoop(mag1Data, mag2Data, &s_on, &t_on, &p_tumb);
		fprintf(csv_output, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf\n",
			mag1Data.x, mag1Data.y, mag1Data.z, mag2Data.x, mag2Data.y, mag2Data.z, t_on.x, t_on.y, t_on.z, s_on.x, s_on.y, s_on.z, p_tumb.x, p_tumb.y, p_tumb.z);
		column = 0;


		// printf("try: %f\n", mag1Data.x);

	}
	fclose(csv_input);
	fclose(csv_output);
	getch();
}


// 			if (strstr(token, "\n") != NULL) {
			//	line_cnt ++;
			