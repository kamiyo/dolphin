// Copyright 2013 Dolphin Emulator Project
// Licensed under GPLv2
// Refer to the license.txt file included./

#pragma once

#include <unordered_map>

#include "Common/CPUDetect.h"
#include "Common/x64Emitter.h"

namespace MMIO { class Mapping; }

#define MEMCHECK_START \
	Gen::FixupBranch memException; \
	if (jit->js.memcheck) \
	{ TEST(32, PPCSTATE(Exceptions), Gen::Imm32(EXCEPTION_DSI)); \
	memException = J_CC(Gen::CC_NZ, true); }

#define MEMCHECK_END \
	if (jit->js.memcheck) \
	SetJumpTarget(memException);

// We offset by 0x80 because the range of one byte memory offsets is
// -0x80..0x7f.
#define PPCSTATE(x) MDisp(RPPCSTATE, \
	(int) ((char *) &PowerPC::ppcState.x - (char *) &PowerPC::ppcState) - 0x80)
// In case you want to disable the ppcstate register:
// #define PPCSTATE(x) M((void*) &PowerPC::ppcState.x)
#define PPCSTATE_LR PPCSTATE(spr[SPR_LR])
#define PPCSTATE_CTR PPCSTATE(spr[SPR_CTR])
#define PPCSTATE_SRR0 PPCSTATE(spr[SPR_SRR0])
#define PPCSTATE_SRR1 PPCSTATE(spr[SPR_SRR1])

// Like XCodeBlock but has some utilities for memory access.
class EmuCodeBlock : public Gen::X64CodeBlock
{
public:
	void LoadAndSwap(int size, Gen::X64Reg dst, const Gen::OpArg& src);
	void SwapAndStore(int size, const Gen::OpArg& dst, Gen::X64Reg src);

	void UnsafeLoadRegToReg(Gen::X64Reg reg_addr, Gen::X64Reg reg_value, int accessSize, s32 offset = 0, bool signExtend = false);
	void UnsafeLoadRegToRegNoSwap(Gen::X64Reg reg_addr, Gen::X64Reg reg_value, int accessSize, s32 offset);
	// these return the address of the MOV, for backpatching
	u8 *UnsafeWriteRegToReg(Gen::X64Reg reg_value, Gen::X64Reg reg_addr, int accessSize, s32 offset = 0, bool swap = true);
	u8 *UnsafeLoadToReg(Gen::X64Reg reg_value, Gen::OpArg opAddress, int accessSize, s32 offset, bool signExtend);

	// Generate a load/write from the MMIO handler for a given address. Only
	// call for known addresses in MMIO range (MMIO::IsMMIOAddress).
	void MMIOLoadToReg(MMIO::Mapping* mmio, Gen::X64Reg reg_value, u32 registers_in_use, u32 address, int access_size, bool sign_extend);

	enum SafeLoadStoreFlags
	{
		SAFE_LOADSTORE_NO_SWAP = 1,
		SAFE_LOADSTORE_NO_PROLOG = 2,
		SAFE_LOADSTORE_NO_FASTMEM = 4,
		SAFE_LOADSTORE_CLOBBER_RSCRATCH_INSTEAD_OF_ADDR = 8
	};

	void SafeLoadToReg(Gen::X64Reg reg_value, const Gen::OpArg & opAddress, int accessSize, s32 offset, u32 registersInUse, bool signExtend, int flags = 0);
	// Clobbers RSCRATCH or reg_addr depending on the relevant flag.  Preserves
	// reg_value if the load fails and js.memcheck is enabled.
	void SafeWriteRegToReg(Gen::X64Reg reg_value, Gen::X64Reg reg_addr, int accessSize, s32 offset, u32 registersInUse, int flags = 0);

	// applies to safe and unsafe WriteRegToReg
	bool WriteClobbersRegValue(int accessSize, bool swap)
	{
		return swap && !cpu_info.bMOVBE && accessSize > 8;
	}

	void SafeWriteF32ToReg(Gen::X64Reg xmm_value, Gen::X64Reg reg_addr, s32 offset, u32 registersInUse, int flags = 0);

	void WriteToConstRamAddress(int accessSize, Gen::X64Reg arg, u32 address, bool swap = false);
	void JitClearCA();
	void JitSetCA();
	void JitClearCAOV(bool oe);

	void ForceSinglePrecisionS(Gen::X64Reg xmm);
	void ForceSinglePrecisionP(Gen::X64Reg xmm);
	void Force25BitPrecision(Gen::X64Reg xmm, Gen::X64Reg tmp);

	// RSCRATCH might get trashed
	void ConvertSingleToDouble(Gen::X64Reg dst, Gen::X64Reg src, bool src_is_gpr = false);
	void ConvertDoubleToSingle(Gen::X64Reg dst, Gen::X64Reg src);
	void SetFPRF(Gen::X64Reg xmm);
protected:
	std::unordered_map<u8 *, u32> registersInUseAtLoc;
	std::unordered_map<u8 *, u32> pcAtLoc;
};
