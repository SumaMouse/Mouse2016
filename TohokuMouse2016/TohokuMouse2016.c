/***********************************************************************/
/*                                                                     */
/*  FILE        :Main.c or Main.cpp                                    */
/*  DATE        :Tue, Oct 31, 2006                                     */
/*  DESCRIPTION :Main Program                                          */
/*  CPU TYPE    :                                                      */
/*                                                                     */
/*  NOTE:THIS IS A TYPICAL EXAMPLE.                                    */
/*                                                                     */
/***********************************************************************/

#include <machine.h>
#include "iodefine.h"
#include "Common.h"

#include "Mcu.h"
#include "Sci.h"
#include "Gpio.h"
#include "Adc.h"
#include "Spi.h"
#include "Rspi.h"

#include "Test.h"

//#include "typedefine.h"
#ifdef __cplusplus
//#include <ios>                        // Remove the comment when you use ios
//_SINT ios_base::Init::init_cnt;       // Remove the comment when you use ios
#endif

void main(void);
#ifdef __cplusplus
extern "C" {
void abort(void);
}
#endif


#pragma interrupt (Excep_TMR0_CMIA0(vect=170))


static u32 timer1ms = 0;

void main(void)
{

	McuInit();
	SciInit();
	AdcInit();
	SpiInit();
	RspiInit();
	
	TestInit();
	
	setpsw_i();

	GpioWrteLed0(TRUE);
	GpioWrteLed1(TRUE);
	GpioWrteLed2(TRUE);

	while(1) {
		
		TestMotor();
	}
}

void Excep_TMR0_CMIA0(void){
	
	static u8 t = 0;
	
	IR(TMR0,CMIA0) = 0;
	
	SciSendPeriodic();
	
	timer1ms++;
	
	if (timer1ms >= 1000) {

		GpioWrteLed0(t);
		GpioWrteLed2(t);
		t ^= 1;
		GpioWrteLed1(t);

		timer1ms = 0;
	}
}


#ifdef __cplusplus
void abort(void)
{

}
#endif
