/*
 * clasification_system.h
 *
 *  Created on: 30 de abr de 2019
 *      Author: Jefferson
 */

#ifndef CLASSIFICATION_SYSTEM_CLASSIFICATION_SYSTEM_H_
#define CLASSIFICATION_SYSTEM_CLASSIFICATION_SYSTEM_H_

#include "stdint.h"
#include <math.h>

#define NUMBER_OF_INPUTS               12
#define NUMBER_OF_HIDDEN               21
#define NUMBER_OF_OUTPUTS              22



#define NUMBER_OF_INPUTS_A               40
#define NUMBER_OF_HIDDEN_A               40
#define NUMBER_OF_OUTPUTS_A              5


void classSystemScaleDataInput(float * vetor);
void classSystemPredict(float * entrada, float * saida);
char classSystemPostProcess(float * saidas);

void classSystemScaleDataInputA(float * vetor);
void classSystemPredictA(float * entrada, float * saida);
char classSystemPostProcessA(float * saidas);

#endif /* CLASSIFICATION_SYSTEM_CLASSIFICATION_SYSTEM_H_ */
