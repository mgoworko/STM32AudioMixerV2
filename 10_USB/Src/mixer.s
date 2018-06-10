	.globl mixer
	.type mixer,%function

mixer:
	stm r4, {r5, r6}
	mov r4, #0x0
	cmp r4, r3
	bge end
loop:
	ldr r5, [r0, r4]
	ldr r6, [r1, r4]
	sadd16 r5, r5, r6
	str r5, [r2, r4]
	add r4, r4, #0x4
	cmp r4, r3
	bne loop
end:
	ldm r4, {r5, r6}
	bx lr
