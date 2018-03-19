//==============================================================================
//
//  File        : mmpf_typedef.h
//  Description : Type define file for A-I-T MMPF source code
//  Author      : Philip Lin
//  Revision    : 1.0
//
//==============================================================================

#ifndef _MMPF_TYPEDEF_H_
#define _MMPF_TYPEDEF_H_

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

//==============================================================================
//
//                              STRUCTURES
//
//==============================================================================

typedef unsigned char   MMP_BOOL;
typedef char            MMP_BYTE;
typedef short           MMP_SHORT;
typedef int             MMP_LONG;
typedef long long       MMP_LONG64;
typedef unsigned char   MMP_UBYTE;
typedef unsigned short  MMP_USHORT;
typedef unsigned int    MMP_ULONG;
typedef unsigned long   MMP_U_LONG;
typedef unsigned long long  MMP_ULONG64;


typedef void*           MMP_HANDLE;

#define MMP_TRUE        (1)
#define MMP_FALSE       (0)

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* _MMPF_TYPEDEF_H_ */
