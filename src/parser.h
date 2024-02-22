/*
 * parser.h
 *
 *  Created on: 2024.02.22
 *
 *      Author: Manuel Müller, Rishika Agawal
 *
 * parser.h
 * this module contains the parsing of the client server communication.
 */

#ifndef PARSER_H_
#define PARSER_H_

#include <bits/stdc++.h>
#include "GlobalVariables.h"
#define TOT_COORD 9
#define NUM_OBS 3



void adv_tokenizer(std::string , char );
void adv_tokenizer1(std::string , char, int, bool);
float* string_parser(std::string );

#endif  /* PARSER_H_ */