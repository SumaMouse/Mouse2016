#ifndef	__RSPI_H__
#define	__RSPI_H__

extern void RspiInit(void);
extern u16 RspiTxRx(u16 txData, u16* rxData);
extern void RspiSetSS(u8 ch, u16 level);

#endif

