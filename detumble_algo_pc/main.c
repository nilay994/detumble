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


int main(int argc, char *argv[])
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
	csv_input = fopen("data4test_04_in.csv", "r");
	if (csv_input == NULL) {
		printf("failed to open input csv\n");
	}

	csv_output = fopen("data4test_04_out_pc.csv", "w");
	if (csv_output == NULL) {
		printf("failed to open output csv\n");
	}
	// initialize parsing variables
	int row = 2;
	char *token = 0;
	int column = 0;
	// initialize detumble variables
	vector_t mag1Data = { 0,0,0 };
	vector_t mag2Data = { 0,0,0 };
	vector_t s_on = { 0,0,0 };
	vector_t t_on = { 0,0,0 };
	vector_t p_tumb = { 0,0,0 };

	double parse_row[17];

	// TODO: strcpy can populate array from strings mostly.

	// strip the header
	fgets(buf, sizeof(buf), csv_input);

	// print header yourself
	//printf("\033[1;31m\n");

	printf("Progress:");
	fprintf(csv_output, "mag1.x,mag1.y,mag1.z,mag2.x,mag2.y,mag2.z,t_on.x,t_on.y,t_on.z,s_on.x,s_on.y,s_on.z,p_tumb.x,p_tumb.y,p_tumb.z,c_tumb,c_detumb\n");
	//printf("\033[0m\n");
	// read from row 2
	while (fgets(buf, sizeof(buf), csv_input)) {
		// printf("row %d: ", row ++);
		row++;
		// initialize buffer for each line and split the string by ","
		token = strtok(buf, ",");


		// populate the readings
		while (token) {

			parse_row[column] = atof(token);
			// printf("%f\t", parse_row[column]);

			token = strtok(NULL, ",");  // reset the static pointer
			column++;

		}

		mag1Data.x = parse_row[0];
		mag1Data.y = parse_row[1];
		mag1Data.z = parse_row[2];
		mag2Data.x = parse_row[3];
		mag2Data.y = parse_row[4];
		mag2Data.z = parse_row[5];

		// make sure to leave counts below static, they aren't remembered by controlLoop, 
		// they are just incremented by controlLoop
		static unsigned int c_tumb = 0;
		static unsigned int c_detumb = 0;
		controlLoop(mag1Data, mag2Data, &s_on, &t_on, &p_tumb, &c_tumb, &c_detumb);
		fprintf(csv_output, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%d,%d\n",
			mag1Data.x, mag1Data.y, mag1Data.z,
			mag2Data.x, mag2Data.y, mag2Data.z,
			t_on.x, t_on.y, t_on.z,
			s_on.x, s_on.y, s_on.z,
			p_tumb.x, p_tumb.y, p_tumb.z,
			c_tumb, c_detumb);
		column = 0;

		if (row % 10000 == 0) {
			printf("|");
		}
	}
	fclose(csv_input);
	fclose(csv_output);
	//getch();
	return 0;
}
