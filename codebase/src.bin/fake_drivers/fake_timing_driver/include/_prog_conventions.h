#ifdef __linux__
  #include <sys/io.h>
#endif
#define uint08	unsigned char
#define uint16	unsigned short
#define uint32	unsigned int
#define int08	char
#define int16	short
#define int32	int

#ifdef __linux__
  #define out8(X,Y) outb(Y,X)
  #define out16(X,Y) outw(Y,X)
  #define out32(X,Y) outl(Y,X)
#endif

// DEFINE CONSTANTS
#define CLOCK_INTERNAL 		0x10
#define CLOCK_EXTERNAL 		0x20
