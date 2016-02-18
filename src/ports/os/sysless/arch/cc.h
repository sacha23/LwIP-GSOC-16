
#include <stdint.h>
#include <stdio.h>

#include <stdint.h>
 
/* Types based on stdint.h */
typedef uint8_t            u8_t; 
typedef int8_t             s8_t; 
typedef uint16_t           u16_t; 
typedef int16_t            s16_t; 
typedef uint32_t           u32_t; 
typedef int32_t            s32_t; 
typedef uintptr_t          mem_ptr_t; 
 
/* Define (sn)printf formatters for these lwIP types */
#define U16_F "hu"
#define S16_F "hd"
#define X16_F "hx"
#define U32_F "lu"
#define S32_F "ld"
#define X32_F "lx"
#define SZT_F "uz"
 
/* ARM/LPC17xx is little endian only */
#define BYTE_ORDER LITTLE_ENDIAN
 
/* Use LWIP error codes */
#define LWIP_PROVIDE_ERRNO
 
#define PACK_STRUCT_BEGIN
#define PACK_STRUCT_STRUCT __attribute__ ((__packed__))
#define PACK_STRUCT_END
#define PACK_STRUCT_FIELD(fld) fld
 
/* Used with IP headers only */
#define LWIP_CHKSUM_ALGORITHM 1
 
/* Plaform specific diagnostic output */
#define LWIP_PLATFORM_DIAG(vars)	printf vars
#define LWIP_PLATFORM_ASSERT(flag) do { printf("Halted ASSERT: %s:%d - %s\n", __FILE__, __LINE__, flag); while(1); } while(0)

/* 32-bit random value used by igmp and others */
#define LWIP_RAND() ((uint32_t)rand())
