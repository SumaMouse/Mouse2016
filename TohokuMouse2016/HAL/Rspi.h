#ifndef	__RSPI_H__
#define	__RSPI_H__

void RspiInit(void);
u16 RspiTxRx(u16 txData, u16* rxData);
void RspiSetCS(u16 level);

#endif

