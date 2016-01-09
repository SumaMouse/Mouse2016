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
#include "iodefine.h"

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

void main(void)
{
	unsigned long timer = 0;
	
	PORT2.PDR.BIT.B7 = 1;
	PORT3.PDR.BIT.B1 = 1;

	PORT2.PODR.BIT.B7 = 0;
	PORT3.PODR.BIT.B1 = 0;
	for(timer=0;timer<0xFFFul;timer++);
	
	while(1) {
		
		PORT2.PODR.BIT.B7 = 1;
		PORT3.PODR.BIT.B1 = 0;
		for(timer=0;timer<0xFFFul;timer++);

		PORT2.PODR.BIT.B7 = 0;
		PORT3.PODR.BIT.B1 = 1;
		for(timer=0;timer<0xFFFul;timer++);

	}
}

#ifdef __cplusplus
void abort(void)
{

}
#endif
