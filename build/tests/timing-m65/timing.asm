; timing.asm - the harness of timing.py: runs test blocks and times each
; with CIA 2's two timers as one 32-bit count of the CIA's clock. It runs the
; same on a MEGA65 and in Xemu; timing.py compares the two.
;
; timing.py writes the tests (out/tests.bin, loaded to TESTS, chip RAM bank 4
; on) and the addresses this file and the tests share (out/timing.inc).
; A test's record:
;   byte   the length of its start code (PRE)      $FF: the end of the tests
;   word   the length of its block (CODE)
;   word   how many times the block runs (CNT)
;   bytes  the start code: sets registers and flags, ends with JMP CODE
;   bytes  the block: the instruction under test a number of times, then
;          DEW CNT, a jump to its top while CNT is not 0, and JMP BACK
; Every record is timed REPS times, the timers started before its start code
; and stopped at BACK, and the longest kept (the CIA's clock on a MEGA65 now
; and then reads about 30 us short): 4 bytes at RESULTS for each, what the
; timers showed (A low, A high, B low, B high; they count down from $FFFF).
; DONE is $A5 when all are through.
;
; A test is two records that differ only in how many times the instruction
; is in the block: the difference of their times is the instruction's alone.
;
;   acme -f cbm -I out -o out/timing.prg timing.asm

!cpu m65
!source "timing.inc"        ; CODE, PRE, RESULTS, DONE, TESTS, REPS, CNT, the tests' own addresses

tp      = $f0               ; (4) the record being read
rp      = $f4               ; (2) where its result goes
best    = $f6               ; (4) the least the timers showed in its runs so far
rep     = $fa               ; runs to go
; CNT   = $fb               ; (2) the block's runs to go (timing.inc)
len     = $fd               ; (2) bytes to copy
dst     = $ee               ; (2) where to

* = $2001
        !word +, 10
        !byte $9e
        !text "8208"        ; SYS $2010
        !byte 0
+       !word 0

* = $2010
start:
        sei
        cld
        see
        lda #0
        tab
        lda #$47
        sta $d02f
        lda #$53
        sta $d02f           ; the VIC-IV's registers
        lda $d031
        ora #$40
        sta $d031
        lda $d054
        ora #$40
        sta $d054           ; 40.5 MHz
        lda #$7f
        sta $dc0d           ; no interrupt from anything
        sta $dd0d
        lda #0
        sta $d01a
        lda $dc0d
        lda $dd0d
        sta DONE

        ; what the tests expect to find: pointers and constants in the base
        ; page, a pointer in the stack page for (d,SP),Y, three subroutines
        ldx #0
-       lda bp_init,x
        sta BP_FIRST,x
        inx
        cpx #bp_init_end - bp_init
        bne -
        lda #<(DATA + $10)
        sta $01f1
        lda #>(DATA + $10)
        sta $01f2
        ldx #0
-       lda sub_init,x
        sta SUBS,x
        inx
        cpx #sub_init_end - sub_init
        bne -
        lda #$4c            ; BACK: where every block ends up
        sta BACK
        lda #<back
        sta BACK+1
        lda #>back
        sta BACK+2
        ; the memory the instructions under test work on: bytes of 1
        lda #<DATA
        sta dst
        lda #>DATA
        sta dst+1
        ldy #0
        lda #1
-       sta (dst),y
        iny
        bne -
        inc dst+1
        ldx dst+1
        cpx #>(DATA + $0f00)
        bne -

        lda #<TESTS
        sta tp
        lda #>TESTS
        sta tp+1
        lda #^TESTS
        sta tp+2
        lda #0
        sta tp+3
        lda #<RESULTS
        sta rp
        lda #>RESULTS
        sta rp+1

next:   ldz #0
        lda [tp],z          ; the start code's length
        cmp #$ff
        bne +
        jmp finished
+       sta len
        lda #0
        sta len+1
        jsr skip1
        lda [tp],z          ; the block's
        pha
        jsr skip1
        lda [tp],z
        pha
        jsr skip1
        lda [tp],z          ; its runs
        sta runs
        jsr skip1
        lda [tp],z
        sta runs+1
        jsr skip1
        lda #<PRE
        sta dst
        lda #>PRE
        sta dst+1
        jsr copy
        pla
        sta len+1
        pla
        sta len
        lda #<CODE
        sta dst
        lda #>CODE
        sta dst+1
        jsr copy

        lda #$ff
        sta best
        sta best+1
        sta best+2
        sta best+3
        lda #REPS
        sta rep
again:  ldx #$ff
        txs
        sei
        cld
        see
        lda runs
        sta CNT
        lda runs+1
        sta CNT+1
        lda #$7f            ; the clock: timer B counts timer A's underflows
        sta $dd0d
        lda #0
        sta $dd0e
        sta $dd0f
        lda #$ff
        sta $dd04
        sta $dd05
        sta $dd06
        sta $dd07
        lda #$51
        sta $dd0f
        lda #$11
        sta $dd0e
        jmp PRE             ; (which goes on to CODE, which comes back here)

back:   lda #0
        sta $dd0e
        ldx #$ff
        txs
        sei
        cld
        see
        ; the least of the runs: the timers count down
        lda $dd07
        cmp best+3
        bcc take
        bne keep
        lda $dd06
        cmp best+2
        bcc take
        bne keep
        lda $dd05
        cmp best+1
        bcc take
        bne keep
        lda $dd04
        cmp best
        bcs keep
take:   lda $dd04
        sta best
        lda $dd05
        sta best+1
        lda $dd06
        sta best+2
        lda $dd07
        sta best+3
keep:   dec rep
        beq +
        jmp again
+       ldy #3
-       lda best,y
        sta (rp),y
        dey
        bpl -
        clc
        lda rp
        adc #4
        sta rp
        bcc +
        inc rp+1
+       inc $d020
        jmp next

finished:
        lda #$a5
        sta DONE
        lda #0
        sta $d020
-       jmp -

; the record pointer one on
skip1:  inc tp
        bne +
        inc tp+1
        bne +
        inc tp+2
+       rts

; len bytes from the record to dst
copy:   lda len
        ora len+1
        beq ++
        ldy #0
-       ldz #0
        lda [tp],z
        sta (dst),y
        jsr skip1
        inc dst
        bne +
        inc dst+1
+       lda len
        bne +
        dec len+1
+       dec len
        lda len
        ora len+1
        bne -
++      rts

runs:   !word 0

; the base page from BP_FIRST on, as timing.py lays it out
bp_init:
        !source "bp.inc"
bp_init_end:

; the subroutines at SUBS: RTS; RTN #0; a vector to the first
sub_init:
        rts
        !byte $62, $00
        !word SUBS
sub_init_end:
