#ifndef	__SPI_H__
#define	__SPI_H__

void SpiInit(void);
u8 SpiSimpleTxRx(u8 txData, u8* rxData);
void SpiSimpleSetCS(u8 level);

#endif

