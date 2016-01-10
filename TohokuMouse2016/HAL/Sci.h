#ifndef	__SCI_H__
#define	__SCI_H__

void	SciInit(void);
void	SciSendPeriodic(void);
void	SciSendString(s8 *str);
void	SciSendHex(u8 beam, u32 value);
void	SciSendDec(u8 beam, s32 value);
void	SciSendByte(u8 data);
u8		SciReceiveByte(void);
u8		SciGetRxDataSize(void);
void	SciSetRxEnable(void);
void	SciSetRxDisable(void);

#endif
