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
#include "Common.h"
#include "iodefine.h"

#include "AplMain.h"

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


void main(void)
{
	AplMainInit();

	setpsw_i();

	AplMain();
	
}

void Excep_TMR0_CMIA0(void){
	
	IR(TMR0,CMIA0) = 0;
	
	Apl1msTask();
	
}


#ifdef __cplusplus
void abort(void)
{

}
#endif
