#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
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
	int row = 2;
	char *token = 0;
	int column = 0;
	vector_t mag1Data;
	vector_t mag2Data;

	vector_t b1_bias = {0,0,0};
	vector_t b2_bias = {0,0,0};
	float parse_row[15];


	// TODO: strcpy can populate array from strings mostly.

	// strip the header
	fgets(buf, sizeof(buf), csv_input); 

	// read from row 2
	while (fgets(buf, sizeof(buf), csv_input)) {
		printf("row %d: ", row ++);

		// initialize buffer for each line and split the string by ","
		token = strtok(buf, ",");


		// populate the readings
		while(token) {

			parse_row[column] = atof(token);
			printf("%f\t", parse_row[column]);

			token = strtok(NULL, ",");  // reset the static pointer
			column ++;

		}
		mag1Data.x = parse_row[0];
		mag1Data.y = parse_row[1];
		mag1Data.z = parse_row[2];
		mag2Data.x = parse_row[3];
		mag2Data.y = parse_row[4];
		mag2Data.z = parse_row[5];

		vector_t b1_raw = mag1Data;
		vector_t b2_raw = mag2Data;

		vector_t fused;
		// use the readings in the algo
		fused = step3_biasCalc(0.5, b1_raw, b1_bias, 0.5, b2_raw, b2_bias);
		column = 0;
		printf("try: %f\n", mag1Data.x);

	}
	fclose(csv_input);
}


// 			if (strstr(token, "\n") != NULL) {
			//	line_cnt ++;
			