// Copyright 2013 Dolphin Emulator Project
// Licensed under GPLv2
// Refer to the license.txt file included.

#pragma once

#include "Common/x64Emitter.h"

// x64 ABI:s, and helpers to help follow them when JIT-ing code.
// All convensions return values in EAX (+ possibly EDX).

// Windows 64-bit
// * 4-reg "fastcall" variant, very new-skool stack handling
// * Callee moves stack pointer, to make room for shadow regs for the biggest function _it itself calls_
// * Parameters passed in RCX, RDX, ... further parameters are MOVed into the allocated stack space.
// Scratch:      RAX RCX RDX R8 R9 R10 R11
// Callee-save:  RBX RSI RDI RBP R12 R13 R14 R15
// Parameters:   RCX RDX R8 R9, further MOV-ed

// Linux 64-bit
// * 6-reg "fastcall" variant, old skool stack handling (parameters are pushed)
// Scratch:      RAX RCX RDX RSI RDI R8 R9 R10 R11
// Callee-save:  RBX RBP R12 R13 R14 R15
// Parameters:   RDI RSI RDX RCX R8 R9

#ifdef _WIN32 // 64-bit Windows - the really exotic calling convention

#define ABI_PARAM1 RCX
#define ABI_PARAM2 RDX
#define ABI_PARAM3 R8
#define ABI_PARAM4 R9

// xmm0-xmm15 use the upper 16 bits in the functions that push/pop registers.
#define ABI_ALL_CALLER_SAVED ((1 << RAX) | (1 << RCX) | (1 << RDX) | (1 << R8) | \
                              (1 << R9) | (1 << R10) | (1 << R11) | \
                              (1 << (XMM0+16)) | (1 << (XMM1+16)) | (1 << (XMM2+16)) | (1 << (XMM3+16)) | \
                              (1 << (XMM4+16)) | (1 << (XMM5+16)))

#else  //64-bit Unix / OS X

#define ABI_PARAM1 RDI
#define ABI_PARAM2 RSI
#define ABI_PARAM3 RDX
#define ABI_PARAM4 RCX
#define ABI_PARAM5 R8
#define ABI_PARAM6 R9

// FIXME: avoid pushing all 16 XMM registers when possible? most functions we call probably
// don't actually clobber them.
#define ABI_ALL_CALLER_SAVED ((1 << RAX) | (1 << RCX) | (1 << RDX) | (1 << RDI) | \
                              (1 << RSI) | (1 << R8) | (1 << R9) | (1 << R10) | (1 << R11) | \
                              0xffff0000 /* xmm0..15 */)

#endif // WIN32

#define ABI_RETURN RAX

