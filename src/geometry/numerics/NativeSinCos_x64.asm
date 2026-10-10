; Windows x64 调用约定：xmm0 传入角度，rdx 和 r8 分别指向正弦及余弦输出。
; 保持 x87 栈平衡，不改变调用方的控制字。
.code
; 计算同一角度的正弦和余弦，并将结果写入输出地址。
meshstudioAlgorithm6SinCos PROC FRAME
    sub rsp, 24
    .allocstack 24
    .endprolog
    movsd QWORD PTR [rsp+8], xmm0
    fld QWORD PTR [rsp+8]
    fsincos
    fnstsw ax
    test ax, 0400h
    jz store_pair
    ; 角度超出硬件范围时，用同一 x87 的 2π 值约化角度。
    fldpi
    fadd st(0), st(0)
    fxch st(1)
reduce_angle:
    fprem1
    fnstsw ax
    test ax, 0400h
    jnz reduce_angle
    fstp st(1)
    fsincos
store_pair:
    fxch st(1)
    fstp QWORD PTR [rdx]
    fstp QWORD PTR [r8]
    add rsp, 24
    ret
meshstudioAlgorithm6SinCos ENDP
END
