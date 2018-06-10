	.globl mixer
	.type mixer,%function

#r0 - wska�nik na plik w pami�ci
#r1 - wska�nik na plik do zmixowania
#r2 - wska�nik na bufor wynikowy
#r3 - wielko�� danych
#r4 - licznik do p�tli i offsetu danych
#r5 - dane wczytane z pliku z pami�ci
#r6 - dane wczytane z pliku do zmixowania

mixer:
	stm r4, {r5, r6}
	mov r4, #0x0
	cmp r4, r3
	bge end
loop:
	ldr r5, [r0, r4]
	ldr r6, [r1, r4]
	shadd16 r5, r5, r6
	str r5, [r2, r4]
	add r4, r4, #0x4
	cmp r4, r3
	bne loop
end:
	ldm r4, {r5, r6}
	bx lr
