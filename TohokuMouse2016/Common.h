#ifndef __COMMON__
#define __COMMON__

#define	PI						(3.14159265f)
#define	CONTROL_CYCLE_SEC		(0.001f)		/* 1ms */

#define	ABS(a)		((a)<0?((a)*(-1)):(a))

#define	SIDE_RIGHT	(0)
#define	FRONT_RIGHT	(1)
#define	FRONT_LEFT	(2)
#define	SIDE_LEFT	(3)

typedef	unsigned char	u8;
typedef	signed char		s8;
typedef	unsigned int	u16;
typedef	signed int		s16;
typedef	unsigned long	u32;
typedef	signed long		s32;
typedef	float			f32;

enum {FALSE=0,TRUE=1};
typedef unsigned char	bool;

#endif	/* __COMMON__ */
