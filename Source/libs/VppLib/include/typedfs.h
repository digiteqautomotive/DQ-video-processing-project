#ifndef _TYPEDEFS_H_
#define _TYPEDEFS_H_
/******** Definition of items with uniquely size **********
(c) 1997-2025 Jaroslav Fojtik
   if you don't use below mentionted compiler, please correct this items
   for your compiler and send me your correction to:
		JaFojtik@seznam.cz or JaFojtik@yandex.com
* Licency: LGPL

List of supported types:

   Type         Size & Description
   --------+---------+---------------------------
   uint8_t   1 byte  =  8 bit
   int8_t    1 byte  =  8 bit signed
   uint16_t  2 bytes = 16 bit
   int16_t   2 bytes = 16 bit signed
   uint32_t  4 bytes = 32 bit
   int32_t   4 bytes = 32 bit signed
   uint64_t  8 bytes = 64 bit
   int64_t   8 bytes = 64 bit signed

***************************************************************/

#ifdef __cplusplus
extern "C" {
#endif


#ifdef __BORLANDC__
 #define NATIVE_ACCESS
 #ifndef LO_ENDIAN
  #define LO_ENDIAN
 #endif
 #if defined(__OS2__) || defined(__WIN32__)
   #define uint16_t   unsigned short int
   #define int16_t   signed short int
   #define  int64_t __int64
   #define  uint64_t unsigned __int64
 #else
   #define uint16_t   unsigned int
   #define int16_t   signed int
 #endif
 #define uint32_t   unsigned long int
 #define int32_t   signed long int

 #if __BORLANDC__>0x0540
   #include <basetsd.h>
 #endif

#else  /* __BORLANDC__ */

#if defined(__EGC__) || defined(__GNUC__)
 #if defined(__MINGW32__) || defined(__MINGW64__)
  #include <basetsd.h>
 #endif

 #if (__BYTE_ORDER__==__ORDER_LITTLE_ENDIAN__) || defined(__DJGPP__) || defined(__MINGW32__) || defined(__MINGW64__)
  #ifndef LO_ENDIAN
   #define LO_ENDIAN
  #endif
 #else
  #if (__BYTE_ORDER__==__ORDER_BIG_ENDIAN__)
   #ifndef HI_ENDIAN
    #define HI_ENDIAN
   #endif
  #endif
 #endif

 #if !defined(NATIVE_ACCESS) && !defined(HI_ENDIAN) && !defined(__ARM_FEATURE_UNALIGNED)
  #define NATIVE_ACCESS
 #endif

 #if __GNUC__>=3 || GCC_VERSION>=30100

  #define HAS_STDINT_H

 #else
  #if !defined(_ASM_GENERIC_INT_LL64_H) && !defined(_UAPI_ASM_GENERIC_INT_LL64_H)
   typedef int int8_t __attribute__((mode(QI)));
   typedef unsigned int uint8_t __attribute__((mode(QI)));
   typedef int int16_t __attribute__((mode(HI)));
   typedef unsigned int uint16_t __attribute__((mode(HI)));
   typedef int int32_t __attribute__((mode(SI)));
  #if !defined(__MINGW32__) && !defined(__MINGW64__)
   typedef unsigned int uint32_t __attribute__((mode(SI)));
  #endif
   typedef int int64_t __attribute__((mode(DI)));
   typedef unsigned int uint64_t __attribute__((mode(DI)));
  #endif
  #define uint8_t_defined
  #define uint16_t_defined
  #define uint32_t_defined
 #endif
 #define uint64_t_defined

 #if __GNUC__>4 || (__GNUC__==4 && __GNUC_MINOR__>=3)
  #define LD_UINT16_SWAP(ptr)  __builtin_bswap16((uint16_t)(*(uint16_t*)(uint8_t*)(ptr)))
  #define LD_UINT32_SWAP(ptr)  __builtin_bswap32((uint32_t)(*(uint32_t*)(uint8_t*)(ptr)))
  #define LD_UINT64_SWAP(ptr)  __builtin_bswap64((uint64_t)(*(uint64_t*)(uint8_t*)(ptr)))

  #define ST_UINT16_SWAP(ptr,val)  *(uint16_t*)(uint8_t*)(ptr)=__builtin_bswap16((uint16_t)(val))
  #define ST_UINT32_SWAP(ptr,val)  *(uint32_t*)(uint8_t*)(ptr)=__builtin_bswap32((uint32_t)(val))
  #define ST_UINT64_SWAP(ptr,val)  *(uint64_t*)(uint8_t*)(ptr)=__builtin_bswap64((uint64_t)(val))
 #endif

#else  /* __EGC__ || __GNUC__ */

#ifdef __WATCOMC__
 #define NATIVE_ACCESS
 #ifdef __386__
  #define uint16_t   unsigned short int
  #define int16_t   signed short int
  #define uint32_t   unsigned int
  #define int32_t   signed int
 #else
  #define uint16_t   unsigned int
  #define int16_t   signed int
  #define uint32_t   unsigned long int
  #define int32_t   signed long int
 #endif
#else /*__WATCOMC__*/

#ifdef __HPUXC__
 #define uint16_t	unsigned short int
 #define int16_t	signed short int
 #define uint32_t	unsigned int
 #define int32_t	signed int
#else

#if defined(__IAR_SYSTEMS_ICC__) || defined(__ARMCC_VERSION)
 #ifndef LO_ENDIAN
  #define LO_ENDIAN
 #endif

 #ifdef __IAR_SYSTEMS_ICC__	/* IAR */
  #ifdef NATIVE_ACCESS
   #undef NATIVE_ACCESS
  #endif
 #else				/* Keil */
  #define HAS_STDINT_H
  #define uint64_t_defined

  #ifndef NATIVE_ACCESS
   #define NATIVE_ACCESS
   #define PACKED_PTR __packed
  #endif
 #endif

#else

#ifdef _MSC_VER

 #ifndef NATIVE_ACCESS
  #define NATIVE_ACCESS
 #endif
 #ifndef LO_ENDIAN
  #define LO_ENDIAN
 #endif

 #if _MSC_VER >= 1600
  #define HAS_STDINT_H
  #define uint64_t_defined
 #else
  #if _MSC_VER > 1200
   #define  int64_t __int64
   #define  uint64_t unsigned __int64
  #endif
 #endif

 #define LD_UINT16_SWAP(ptr)  _byteswap_ushort((uint16_t)(*(uint16_t*)(uint8_t*)(ptr)))
 #define LD_UINT32_SWAP(ptr)  _byteswap_ulong((uint32_t)(*(uint32_t*)(uint8_t*)(ptr)))
 #define LD_UINT64_SWAP(ptr)  _byteswap_uint64((unsigned __int64)(*(unsigned __int64*)(uint8_t*)(ptr)))

 #include <stdlib.h>
 #define ST_UINT16_SWAP(ptr,val)  *(uint16_t*)(uint8_t*)(ptr)=_byteswap_ushort((uint16_t)(val))
 #define ST_UINT32_SWAP(ptr,val)  *(uint32_t*)(uint8_t*)(ptr)=_byteswap_ulong((uint32_t)(val))
 #define ST_UINT64_SWAP(ptr,val)  *(uint64_t*)(uint8_t*)(ptr)=_byteswap_uint64((uint64_t)(val))

/* Here you may include your definition for other C */
#endif
#endif
#endif
#endif
#endif
#endif


#ifndef PACKED_PTR
 #define PACKED_PTR
#endif


#if defined(NATIVE_ACCESS) && defined(NO_NATIVE_ACCESS)
   #undef NATIVE_ACCESS
#endif

#ifdef HAS_STDINT_H

 #include <stdint.h>

#else

 #if !defined(uint8_t) && !defined(uint8_t_defined)
  #define uint8_t    unsigned char
  #define int8_t    signed char
 #endif
 #if !defined(uint16_t) && !defined(uint16_t_defined)
  #define uint16_t   unsigned short
  #define int16_t   signed short
 #endif
 #if !defined(uint32_t) && !defined(uint32_t_defined)
  #define uint32_t   unsigned int
  #define int32_t   signed int
 #endif
 #if defined(uint64_t) && !defined(uint64_t_defined)
  #define uint64_t_defined
 #endif

#endif


/*
#if defined(_MSC_VER) && !defined(_INC_WINDOWS) && !defined(INT32)
   typedef int32_t  INT32;
   typedef uint32_t  UINT32;
#endif
*/

/*----------- END of per compiler section ------------*/


#define LD_UINT16_CPU(ptr)	(uint16_t)(*(PACKED_PTR uint16_t*)(uint8_t*)(ptr))
#define ST_UINT16_CPU(ptr,val)	*(PACKED_PTR uint16_t*)(uint8_t*)(ptr)=(uint16_t)(val)

#define LD_UINT32_CPU(ptr)	(uint32_t)(*(PACKED_PTR uint32_t*)(uint8_t*)(ptr))
#define ST_UINT32_CPU(ptr,val)	*(PACKED_PTR uint32_t*)(uint8_t*)(ptr)=(uint32_t)(val)

#if defined(uint64_t_defined)
 #define LD_UINT64_CPU(ptr)	(uint64_t)(*(PACKED_PTR uint64_t*)(uint8_t*)(ptr))
 #define ST_UINT64_CPU(ptr,val) *(PACKED_PTR uint64_t*)(uint8_t*)(ptr)=(uint64_t)(val)
#endif


/* Prototypes for floats and doubles. */
double LD_DOUBLE_SWAP(const void *b);
void ST_DOUBLE_SWAP(void *b, double d);
float LD_FLOAT_SWAP(const void *b);
void ST_FLOAT_SWAP(void *b, float f);



#ifdef NATIVE_ACCESS  /* Enable word access to structures. */

 #ifdef LO_ENDIAN
  #define LD_UINT16_LO(ptr)	LD_UINT16_CPU(ptr)
  #define LD_UINT32_LO(ptr)	LD_UINT32_CPU(ptr)
  #define ST_UINT16_LO(ptr,val)	ST_UINT16_CPU(ptr,val)
  #define ST_UINT32_LO(ptr,val)	ST_UINT32_CPU(ptr,val)
  #if defined(uint64_t_defined)
   #define LD_UINT64_LO(ptr)	LD_UINT64_CPU(ptr)
   #define ST_UINT64_LO(ptr,val) ST_UINT64_CPU(ptr,val)
  #endif
  #ifdef LD_UINT16_SWAP
   #define LD_UINT16_HI(ptr)	LD_UINT16_SWAP(ptr)
  #endif
  #ifdef LD_UINT32_SWAP
   #define LD_UINT32_HI(ptr)	LD_UINT32_SWAP(ptr)
  #endif
  #ifdef ST_UINT16_SWAP
   #define ST_UINT16_HI(ptr,val) ST_UINT16_SWAP(ptr,val)
  #endif
  #ifdef ST_UINT32_SWAP
   #define ST_UINT32_HI(ptr,val) ST_UINT32_SWAP(ptr,val)
  #endif
  #if defined(uint64_t_defined)
   #ifdef LD_UINT64_SWAP
    #define LD_UINT64_HI(ptr)	LD_UINT64_SWAP(ptr)
   #endif
   #ifdef ST_UINT64_SWAP
    #define ST_UINT64_HI(ptr,val) ST_UINT64_SWAP(ptr,val)
   #endif
  #endif

  #define LD_FLOAT_HI(ptr)	LD_FLOAT_SWAP(ptr)
  #define ST_FLOAT_HI(ptr,val)	ST_FLOAT_SWAP(ptr,val)
  #define LD_DOUBLE_HI(ptr)	LD_DOUBLE_SWAP(ptr)
  #define ST_DOUBLE_HI(ptr,val)	ST_DOUBLE_SWAP(ptr,val)
  #define LD_FLOAT_LO(ptr)	LD_UINT32_CPU(ptr)
 #endif

 #ifdef HI_ENDIAN
  #define LD_UINT16_HI(ptr)	LD_UINT16_CPU(ptr)
  #define LD_UINT32_HI(ptr)	LD_UINT32_CPU(ptr)
  #define ST_UINT16_HI(ptr,val)	ST_UINT16_CPU(ptr,val)
  #define ST_UINT32_HI(ptr,val)	ST_UINT32_CPU(ptr,val)
  #if defined(uint64_t_defined)
   #define LD_UINT64_HI(ptr)	LD_UINT64_CPU(ptr)
   #define ST_UINT64_HI(ptr,val) ST_UINT64_CPU(ptr,val)
  #endif

  #define LD_UINT16_LO(ptr)	LD_UINT16_SWAP(ptr)
  #define LD_UINT32_LO(ptr)	LD_UINT32_SWAP(ptr)
  #define ST_UINT16_LO(ptr,val)	ST_UINT16_SWAP(ptr,val)
  #define ST_UINT32_LO(ptr,val)	ST_UINT32_SWAP(ptr,val)
  #if defined(uint64_t_defined)
   #define LD_UINT64_LO(ptr)	LD_UINT64_SWAP(ptr)
   #define ST_UINT64_LO(ptr,val) ST_UINT64_SWAP(ptr,val)
  #endif

  #define LD_FLOAT_HI(ptr)	LD_UINT32_CPU(ptr)
  #define LD_FLOAT_LO(ptr)	LD_FLOAT_SWAP(ptr)
 #endif
#endif


	/* Safe replacements for non native access. */
#ifdef __cplusplus

#ifndef LD_UINT16_LO
 inline uint16_t LD_UINT16_LO(const void *ptr) {return (uint16_t)(((uint16_t)*(((uint8_t*)ptr)+1)<<8)|(uint16_t)*(uint8_t*)(ptr));}
#endif
#ifndef LD_UINT32_LO
 inline uint32_t LD_UINT32_LO(const void *ptr) {return (uint32_t)(((uint32_t)*((uint8_t*)(ptr)+3)<<24)|((uint32_t)*((uint8_t*)(ptr)+2)<<16)|((uint16_t)*((uint8_t*)(ptr)+1)<<8)|*(uint8_t*)(ptr));}
#endif
#ifndef ST_UINT16_LO
 inline void ST_UINT16_LO(void *ptr, const uint16_t val) {*(uint8_t*)(ptr)=(uint8_t)(val); *((uint8_t*)(ptr)+1)=(uint8_t)((uint16_t)(val)>>8);}
#endif
#ifndef ST_UINT32_LO
 inline void ST_UINT32_LO(void *ptr, const uint32_t val) {*(uint8_t*)(ptr)=(uint8_t)(val); *((uint8_t*)(ptr)+1)=(uint8_t)((uint16_t)(val)>>8); *((uint8_t*)(ptr)+2)=(uint8_t)((uint32_t)(val)>>16); *((uint8_t*)(ptr)+3)=(uint8_t)((uint32_t)(val)>>24);};
#endif

#if defined(uint64_t_defined)
 #ifndef LD_UINT64_LO
   inline uint64_t LD_UINT64_LO(const void *ptr)
   {return (uint64_t)((((uint64_t)*((uint8_t*)(ptr)+7)<<56) | ((uint64_t)*((uint8_t*)(ptr)+6)<<48) | ((uint64_t)*((uint8_t*)(ptr)+5)<<40) | ((uint64_t)*((uint8_t*)(ptr)+4)<<32) | ((uint32_t)*((uint8_t*)(ptr)+3)<<24) | ((uint32_t)*((uint8_t*)(ptr)+2)<<16) | ((uint16_t)*((uint8_t*)(ptr)+1)<<8) | *(uint8_t*)(ptr)));}
 #endif
 #ifndef ST_UINT64_LO
  inline void ST_UINT64_LO(void *ptr, const uint64_t val)
  {*(uint8_t*)(ptr)=(uint8_t)(val); *((uint8_t*)(ptr)+1)=(uint8_t)((uint16_t)(val)>>8); *((uint8_t*)(ptr)+2)=(uint8_t)((uint32_t)(val)>>16); *((uint8_t*)(ptr)+3)=(uint8_t)((uint32_t)(val)>>24); *((uint8_t*)(ptr)+4)=(uint8_t)((uint64_t)(val)>>32); *((uint8_t*)(ptr)+5)=(uint8_t)((uint64_t)(val)>>40); *((uint8_t*)(ptr)+6)=(uint8_t)((uint64_t)(val)>>48); *((uint8_t*)(ptr)+7)=(uint8_t)((uint64_t)(val)>>56);}
 #endif
#endif


#ifndef LD_UINT16_HI
  inline uint16_t LD_UINT16_HI(const void *ptr) {return (uint16_t)(((uint16_t)*(uint8_t*)((ptr))<<8)|(uint16_t)*((uint8_t*)(ptr)+1));}
#endif
#ifndef LD_UINT32_HI
 inline uint32_t LD_UINT32_HI(const void *ptr) {return (uint32_t)(((uint32_t)*(uint8_t*)((ptr))<<24)|((uint32_t)*((uint8_t*)(ptr)+1)<<16)|((uint16_t)*((uint8_t*)(ptr)+2)<<8)|*((uint8_t*)(ptr)+3));}
#endif
#ifndef ST_UINT16_HI
 inline void ST_UINT16_HI(void *ptr, const uint16_t val) {*((uint8_t*)(ptr)+1)=(uint8_t)(val); *(uint8_t*)((ptr))=(uint8_t)((uint16_t)(val)>>8);}
#endif
#ifndef ST_UINT32_HI
 inline void ST_UINT32_HI(void *ptr, const uint32_t val) {*((uint8_t*)(ptr)+3)=(uint8_t)(val); *((uint8_t*)(ptr)+2)=(uint8_t)((uint16_t)(val)>>8); *((uint8_t*)(ptr)+1)=(uint8_t)((uint32_t)(val)>>16); *(uint8_t*)((ptr))=(uint8_t)((uint32_t)(val)>>24);}
#endif

#if defined(uint64_t_defined)
 #ifndef LD_UINT64_HI
  inline uint64_t LD_UINT64_HI(const void *ptr)
  {return (uint64_t)( ((uint64_t)*(uint8_t*)(ptr)<<56) | ((uint64_t)*((uint8_t*)(ptr)+1)<<48) | ((uint64_t)*((uint8_t*)(ptr)+2)<<40) | ((uint64_t)*((uint8_t*)(ptr)+3)<<32) | ((uint32_t)*((uint8_t*)(ptr)+4)<<24)|((uint32_t)*((uint8_t*)(ptr)+5)<<16) | ((uint16_t)*((uint8_t*)(ptr)+6)<<8) | *((uint8_t*)(ptr)+7) );}
 #endif
 #ifndef ST_UINT64_HI
  inline void ST_UINT64_HI(void *ptr, const uint64_t val)
  {*((uint8_t*)(ptr)+7)=(uint8_t)(val); *((uint8_t*)(ptr)+6)=(uint8_t)((uint16_t)(val)>>8); *((uint8_t*)(ptr)+5)=(uint8_t)((uint32_t)(val)>>16); *((uint8_t*)(ptr)+4)=(uint8_t)((uint32_t)(val)>>24); *((uint8_t*)(ptr)+3)=(uint8_t)((uint64_t)(val)>>32); *((uint8_t*)(ptr)+2)=(uint8_t)((uint64_t)(val)>>40); *((uint8_t*)(ptr)+1)=(uint8_t)((uint64_t)(val)>>48); *(uint8_t*)((ptr))=(uint8_t)((uint64_t)(val)>>56);}
 #endif
#endif


#else

#ifndef LD_UINT16_LO
 #define LD_UINT16_LO(ptr)	(uint16_t)(((uint16_t)*(uint8_t*)((ptr)+1)<<8)|(uint16_t)*(uint8_t*)(ptr))
#endif
#ifndef LD_UINT32_LO
 #define LD_UINT32_LO(ptr)	(uint32_t)(((uint32_t)*(uint8_t*)((ptr)+3)<<24)|((uint32_t)*(uint8_t*)((ptr)+2)<<16)|((uint16_t)*(uint8_t*)((ptr)+1)<<8)|*(uint8_t*)(ptr))
#endif
#ifndef ST_UINT16_LO
 #define ST_UINT16_LO(ptr,val)	*(uint8_t*)(ptr)=(uint8_t)(val); *(uint8_t*)((ptr)+1)=(uint8_t)((uint16_t)(val)>>8)
#endif
#ifndef ST_UINT32_LO
 #define ST_UINT32_LO(ptr,val)	*(uint8_t*)(ptr)=(uint8_t)(val); *(uint8_t*)((ptr)+1)=(uint8_t)((uint16_t)(val)>>8); *(uint8_t*)((ptr)+2)=(uint8_t)((uint32_t)(val)>>16); *(uint8_t*)((ptr)+3)=(uint8_t)((uint32_t)(val)>>24)
#endif

#if defined(uint64_t_defined)
 #ifndef LD_UINT64_LO
  #define LD_UINT64_LO(ptr)	(uint64_t)((((uint64_t)*(uint8_t*)((ptr)+7)<<56) | ((uint64_t)*(uint8_t*)((ptr)+6)<<48) | ((uint64_t)*(uint8_t*)((ptr)+5)<<40) | ((uint64_t)*(uint8_t*)((ptr)+4)<<32) | ((uint32_t)*(uint8_t*)((ptr)+3)<<24) | ((uint32_t)*(uint8_t*)((ptr)+2)<<16) | ((uint16_t)*(uint8_t*)((ptr)+1)<<8) | *(uint8_t*)(ptr)))
 #endif
 #ifndef ST_UINT64_LO
  #define ST_UINT64_LO(ptr,val)	*(uint8_t*)(ptr)=(uint8_t)(val); *(uint8_t*)((ptr)+1)=(uint8_t)((uint16_t)(val)>>8); *(uint8_t*)((ptr)+2)=(uint8_t)((uint32_t)(val)>>16); *(uint8_t*)((ptr)+3)=(uint8_t)((uint32_t)(val)>>24); *(uint8_t*)((ptr)+4)=(uint8_t)((uint64_t)(val)>>32); *(uint8_t*)((ptr)+5)=(uint8_t)((uint64_t)(val)>>40); *(uint8_t*)((ptr)+6)=(uint8_t)((uint64_t)(val)>>48); *(uint8_t*)((ptr)+7)=(uint8_t)((uint64_t)(val)>>56)
 #endif
#endif


#ifndef LD_UINT16_HI
 #define LD_UINT16_HI(ptr)	(uint16_t)(((uint16_t)*(uint8_t*)((ptr))<<8)|(uint16_t)*((uint8_t*)(ptr)+1))
#endif
#ifndef LD_UINT32_HI
 #define LD_UINT32_HI(ptr)	(uint32_t)(((uint32_t)*(uint8_t*)((ptr))<<24)|((uint32_t)*(uint8_t*)((ptr)+1)<<16)|((uint16_t)*(uint8_t*)((ptr)+2)<<8)|*((uint8_t*)(ptr)+3))
#endif
#ifndef ST_UINT16_HI
 #define ST_UINT16_HI(ptr,val)	*((uint8_t*)(ptr)+1)=(uint8_t)(val); *(uint8_t*)((ptr))=(uint8_t)((uint16_t)(val)>>8)
#endif
#ifndef ST_UINT32_HI
 #define ST_UINT32_HI(ptr,val)	*((uint8_t*)(ptr)+3)=(uint8_t)(val); *(uint8_t*)((ptr)+2)=(uint8_t)((uint16_t)(val)>>8); *(uint8_t*)((ptr)+1)=(uint8_t)((uint32_t)(val)>>16); *(uint8_t*)((ptr))=(uint8_t)((uint32_t)(val)>>24)
#endif

#if defined(uint64_t_defined)
 #ifndef LD_UINT64_HI
  #define LD_UINT64_HI(ptr)	(uint64_t)( ((uint64_t)*(uint8_t*)(ptr)<<56) | ((uint64_t)*(uint8_t*)((ptr)+1)<<48) | ((uint64_t)*(uint8_t*)((ptr)+2)<<40) | ((uint64_t)*(uint8_t*)((ptr)+3)<<32) | ((uint32_t)*(uint8_t*)((ptr)+4)<<24)|((uint32_t)*(uint8_t*)((ptr)+5)<<16) | ((uint16_t)*(uint8_t*)((ptr)+6)<<8) | *(uint8_t*)((ptr)+7) )
 #endif
 #ifndef ST_UINT64_HI
  #define ST_UINT64_HI(ptr,val)	*((uint8_t*)(ptr)+7)=(uint8_t)(val); *((uint8_t*)(ptr)+6)=(uint8_t)((uint16_t)(val)>>8); *(uint8_t*)((ptr)+5)=(uint8_t)((uint32_t)(val)>>16); *(uint8_t*)((ptr)+4)=(uint8_t)((uint32_t)(val)>>24); *(uint8_t*)((ptr)+3)=(uint8_t)((uint64_t)(val)>>32); *(uint8_t*)((ptr)+2)=(uint8_t)((uint64_t)(val)>>40); *(uint8_t*)((ptr)+1)=(uint8_t)((uint64_t)(val)>>48); *(uint8_t*)((ptr))=(uint8_t)((uint64_t)(val)>>56)
 #endif
#endif


#endif


	/* Use byte-by-byte access to structures */
#define LD_UINT16(ptr)		LD_UINT16_LO(ptr)
#define LD_UINT32(ptr)		LD_UINT32_LO(ptr)
#define ST_UINT16(ptr,val)	ST_UINT16_LO(ptr,val)
#define ST_UINT32(ptr,val)	ST_UINT32_LO(ptr,val)
#if defined(uint64_t_defined)
 #define LD_UINT64(ptr)		LD_UINT64_LO(ptr)
 #define ST_UINT64(ptr,val)	ST_UINT64_LO(ptr,val)
#endif


#ifdef __cplusplus
 }
#endif

#endif  		/* End of Header typedfs.h */
