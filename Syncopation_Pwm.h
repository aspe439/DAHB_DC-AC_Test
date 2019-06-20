/*
 * Syncopation_Pwm.h
 *
 *  Created on: Jan 23, 2019
 *      Author: y3437
 */

#ifndef SYNCOPATION_PWM_H_
#define SYNCOPATION_PWM_H_

extern void Dab_EN();
extern void Boost_EN();
extern void Dab_DIS();
extern void Boost_DIS();

extern void DabFreq_INC();
extern void DabFreq_DEC();
extern void DabPhs_INC();
extern void DabPhs_DEC();

extern void DabPhs_SET(int16 arg);
extern void Boost_SET(float duty);

extern void Dab_Update();


#endif /* SYNCOPATION_PWM_H_ */
