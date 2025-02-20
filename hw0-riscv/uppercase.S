        .section .data
input_string:   .asciz "The @quick #Brown-Fox, jumped over the91 lazy dog!"
        # tohost/fromhost are data locations used by Spike
        .align 8
tohost: .byte 0
        .align 8
fromhost: .byte 0

        
        .section .text
        .globl _start
_start:
        # Load the address of the input string into a0
        la a0, input_string

        # Your code here!
loop:
        lb a1, 0(a0)          
        beqz a1, end_program  
        li a2, 'a'            
        li a3, 'z'            
        blt a1, a2, next_char 
        bgt a1, a3, next_char 
        li a4, 32             
        sub a1, a1, a4        
        sb a1, 0(a0)          
next_char:
        addi a0, a0, 1        
        j loop                

        
end_program:
        # jump here when you're done
        j end_program

