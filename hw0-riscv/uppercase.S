        .section .data
input_string:   .asciz "Hello world!"

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

loop:
        lb t0, 0(a0)         # Load byte from memory (current character)
        beqz t0, end_program # Stop if NUL terminator is reached
        
        li t1, 'a'           # Load ASCII value of 'a'
        li t2, 'z'           # Load ASCII value of 'z'
        
        blt t0, t1, next     # If char < 'a', skip conversion
        bgt t0, t2, next     # If char > 'z', skip conversion
        
        addi t0, t0, -32     # Convert to uppercase (subtract 32)
        sb t0, 0(a0)         # Store the updated character in memory

next:
        addi a0, a0, 1       # Move to the next character
        j loop               # Repeat

        
end_program:
        # jump here when you're done
        j end_program

