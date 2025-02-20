
uppercase.bin:     file format elf32-littleriscv


Disassembly of section .text:

00010074 <_start>:
   10074:	ffff2517          	auipc	a0,0xffff2
   10078:	f8c50513          	addi	a0,a0,-116 # 2000 <__DATA_BEGIN__>

0001007c <loop>:
   1007c:	00050583          	lb	a1,0(a0)
   10080:	02058463          	beqz	a1,100a8 <end_program>
   10084:	06100613          	li	a2,97
   10088:	07a00693          	li	a3,122
   1008c:	00c5ca63          	blt	a1,a2,100a0 <next_char>
   10090:	00b6c863          	blt	a3,a1,100a0 <next_char>
   10094:	02000713          	li	a4,32
   10098:	40e585b3          	sub	a1,a1,a4
   1009c:	00b50023          	sb	a1,0(a0)

000100a0 <next_char>:
   100a0:	00150513          	addi	a0,a0,1
   100a4:	fd9ff06f          	j	1007c <loop>

000100a8 <end_program>:
   100a8:	0000006f          	j	100a8 <end_program>
