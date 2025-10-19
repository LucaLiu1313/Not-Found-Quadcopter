 AREA	|.text|,CODE,READONLY
myScatterLoad	PROC
	EXPORT myScatterLoad
	IMPORT |Image$$RW_IRAM1$$Base|		;从别处导入data段的链接地址
	IMPORT |Image$$RW_IRAM1$$Length|	;从别处导入data段的长度
	IMPORT |Load$$RW_IRAM1$$Base|		;从别处导入data段的加载地址
	IMPORT |Image$$RW_IRAM1$$ZI$$Base|	;从别处导入ZI段的链接地址
	IMPORT |Image$$RW_IRAM1$$ZI$$Length|;从别处导入ZI段的长度
		
	; 复制数据段
	LDR R0, = |Load$$RW_IRAM1$$Base|   	;将data段的加载地址存入R0寄存器
	LDR R1, = |Image$$RW_IRAM1$$Base|   ;将data段的链接地址存入R1寄存器
	LDR R2, = |Image$$RW_IRAM1$$Length| ;将data段的长度存入R2寄存器
CopyData		
	SUB R2, R2, #4						;每次复制4个字节的data段数据
	LDR R3, [R0, R2]					;把加载地址处的值取出到R3寄存器
	STR R3, [R1, R2]					;把取出的值从R3寄存器存入到链接地址					
	CMP R2, #0							;将计数和0相比较
	BNE CopyData						;如果不相等，跳转到CopyData标签处，相等则往下执行

; 清除BSS段
	LDR R0, = |Image$$RW_IRAM1$$ZI$$Base|   ;将bss段的链接地址存入R1寄存器
	LDR R1, = |Image$$RW_IRAM1$$ZI$$Length| ;将bss段的长度存入R2寄存器
CleanBss	
	SUB R1, R1, #4						;每次清除4个字节的bss段数据
	MOV R3, #0							;将0存入r3寄存器
	STR R3, [R0, R1]					;把R3寄存器存入到链接地址					
	CMP R1, #0							;将计数和0相比较
	BNE CleanBss						;如果不相等，跳转到CleanBss标签处，相等则往下执行

	ALIGN 								;填充字节使地址对齐
	ENDP
		
	END	