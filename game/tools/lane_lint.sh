#!/bin/sh
# lane_lint.sh <elf> — the per-lane kernel functions must have no data-dependent branches,
# or the divergent-branch detector halts the core. vtrace_ref may have exactly two branches,
# both uniform by construction: the BALLOT loop test (D20: BALLOT before LMASK; a cap of 0
# never enters) and its rotated copy at the loop bottom. kc_render_frame holds the whole
# per-pixel kernel inlined (render.c) and may have at most six branches, all on uniform loop
# values: the frame-height guard, the block-row loop test, the block loop guard and test, and
# the two jumps between them. A per-lane select that clang lowered to a branch adds one more.
ELF=$1
bad=0
for f in qrcp ray_setup kc_shade_pixel vtrace_ref kc_render_frame; do
    llvm-nm "$ELF" | grep -q " $f\$" || continue
    n=$(llvm-objdump -d --no-show-raw-insn --disassemble-symbols=$f "$ELF" \
        | grep -cE '\s(beq|bne|blt|bge|bltu|bgeu|beqz|bnez|bltz|bgez|blez|bgtz|j)\s')
    allowed=0; [ "$f" = vtrace_ref ] && allowed=2
    if [ "$f" = kc_render_frame ]; then
        if [ "$n" -gt 6 ]; then echo "FAIL: $f has $n branches (allowed at most 6)"; bad=1; fi
    elif [ "$n" -ne "$allowed" ]; then echo "FAIL: $f has $n branches (allowed $allowed)"; bad=1; fi
done
[ $bad -eq 0 ] && echo "lane lint OK: kernel is branch-free per lane"
exit $bad
