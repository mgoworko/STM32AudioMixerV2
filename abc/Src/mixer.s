	.globl mixer
	.type mixer,%function

#r0 - wskaznik na plik w pamieci
#r1 - wskaznik na plik do zmixowania
#r2 - wskaznik na bufor wynikowy
#r3 - stopien sciszenia danych
#r4 - licznik do petli i offsetu danych
#r5 - dane wczytane z pliku z pamieci
#r6 - dane wczytane z pliku do zmixowania

mixer:
	push {r4-r7}
	mov r4, #0
	cmp r3, #0
	ble mix
quiet:
	ldr r5, [r0, r4]
	
	mov r6, r5
	asr r6, r6, #16
	asr r6, r3
	lsl r6, r6, #16
	lsl r5, r5, #16
	asr r5, r3
	lsr r5, r5, #16
	orr r5, r5, r6

	ldr r6, [r1, r4]
	shadd16 r5, r5, r6
	str r5, [r2, r4]
	add r4, r4, #4
	cmp r4, #60
	bne quiet
	b end
mix:
	ldr r5, [r0, r4]
	ldr r6, [r1, r4]
	shadd16 r5, r5, r6
	str r5, [r2, r4]
	add r4, r4, #4
	cmp r4, #60
	bne mix
end:
	pop {r4-r7}
	bx lr
