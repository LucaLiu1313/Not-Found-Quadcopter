 AREA	|.text|,CODE,READONLY
myScatterLoad	PROC
	EXPORT myScatterLoad
	IMPORT |Image$$RW_IRAM1$$Base|		;�ӱ𴦵���data�ε����ӵ�ַ
	IMPORT |Image$$RW_IRAM1$$Length|	;�ӱ𴦵���data�εĳ���
	IMPORT |Load$$RW_IRAM1$$Base|		;�ӱ𴦵���data�εļ��ص�ַ
	IMPORT |Image$$RW_IRAM1$$ZI$$Base|	;�ӱ𴦵���ZI�ε����ӵ�ַ
	IMPORT |Image$$RW_IRAM1$$ZI$$Length|;�ӱ𴦵���ZI�εĳ���
		
	; �������ݶ�
	LDR R0, = |Load$$RW_IRAM1$$Base|   	;��data�εļ��ص�ַ����R0�Ĵ���
	LDR R1, = |Image$$RW_IRAM1$$Base|   ;��data�ε����ӵ�ַ����R1�Ĵ���
	LDR R2, = |Image$$RW_IRAM1$$Length| ;��data�εĳ��ȴ���R2�Ĵ���
CopyData		
	SUB R2, R2, #4						;ÿ�θ���4���ֽڵ�data������
	LDR R3, [R0, R2]					;�Ѽ��ص�ַ����ֵȡ����R3�Ĵ���
	STR R3, [R1, R2]					;��ȡ����ֵ��R3�Ĵ������뵽���ӵ�ַ					
	CMP R2, #0							;��������0��Ƚ�
	BNE CopyData						;�������ȣ���ת��CopyData��ǩ�������������ִ��

; ���BSS��
	LDR R0, = |Image$$RW_IRAM1$$ZI$$Base|   ;��bss�ε����ӵ�ַ����R1�Ĵ���
	LDR R1, = |Image$$RW_IRAM1$$ZI$$Length| ;��bss�εĳ��ȴ���R2�Ĵ���
CleanBss	
	SUB R1, R1, #4						;ÿ�����4���ֽڵ�bss������
	MOV R3, #0							;��0����r3�Ĵ���
	STR R3, [R0, R1]					;��R3�Ĵ������뵽���ӵ�ַ					
	CMP R1, #0							;��������0��Ƚ�
	BNE CleanBss						;�������ȣ���ת��CleanBss��ǩ�������������ִ��

	ALIGN 								;����ֽ�ʹ��ַ����
	ENDP
		
	END	