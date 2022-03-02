
    ; Archivo:	    prelab.s
    ; Proyecto:	    Laboratorio_06 (Timers)
    ; Dispositivo:  PIC16F887
    ; Autor:	    Pablo Caal
    ; Compilador:   pic-as (v2.30), MPLABX V5.40
    ;
    ; Programa:	Variable que incrementa empleando timer 1 y bit intermitente empleando timer 2
    ; Hardware:	PORTA: Salida de 8 bits con LEDs conectados
    ;		PORTC: Salida de 8 bits conecatado a dos display de 7 segmentos 
    ;		PORTD: Salida de 2 bits para seleccionar al display activo
    ;		PORTE: Salida de 1 bit para LED intermitente
    ; Creado: 01 mar, 2022
    ; Última modificación: 02 mar, 2022
    
    PROCESSOR 16F887
    #include <xc.inc>

    ; CONFIG1
	CONFIG  FOSC = INTRC_NOCLKOUT	; Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
	CONFIG  WDTE = OFF		; Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
	CONFIG  PWRTE = OFF		; Power-up Timer Enable bit (PWRT enabled)
	CONFIG  MCLRE = OFF		; RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
	CONFIG  CP = OFF		; Code Protection bit (Program memory code protection is disabled)
	CONFIG  CPD = OFF		; Data Code Protection bit (Data memory code protection is disabled)

	CONFIG  BOREN = OFF		; Brown Out Reset Selection bits (BOR disabled)
	CONFIG  IESO = OFF		; Internal External Switchover bit (Internal/External Switchover mode is disabled)
	CONFIG  FCMEN = OFF		; Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
	CONFIG  LVP = OFF		; Low Voltage Programming Enable bit (RB3/PGM pin has PGM function, low voltage programming enabled)

    ; CONFIG2
	CONFIG  BOR4V = BOR40V		; Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
	CONFIG  WRT = OFF		; Flash Program Memory Self Write Enable bits (Write protection off)

    ;------------------------------ MACROS -------------------------------------
    ; MACRO PARA TIMER0
    RESET_TIMER0 MACRO VALOR_TIMER
	BANKSEL TMR0		    ; Direccionamos a banco correcto
	MOVLW   VALOR_TIMER	    ; Cargamos a W el valor a configurar
	MOVWF   TMR0		    ; Enviamos a TMR0 el valor para configurar el tiempo de retardo
	BCF	T0IF		    ; Limpiamos la bandera de interrupción
	endm
	
    ; MACRO PARA TIMER1
    RESET_TIMER1 MACRO TMR1_H, TMR1_L	 
	BANKSEL TMR1H		    ; Direccionamos a banco correcto
	MOVLW   TMR1_H		    ; Cargamos a W el literal 
	MOVWF   TMR1H		    ; Colocamos al literal en TMR1H
	MOVLW   TMR1_L		    ; Cargamos a W otro literal
	MOVWF   TMR1L		    ; Colocamos al otro literal en TMR1L
	BCF	TMR1IF		    ; Limpiamos bandera de interrupción
	endm
	
    ;------------------------- VALORES EN MEMORIA ------------------------------
    ; Status de las interrupciones
    PSECT udata_shr		    ; Memoria compartida
	W_TEMP:		DS 1	    ; 1 byte
	STATUS_TEMP:	DS 1	    ; 1 byte
    
    ; Variables globales
	PSECT udata_bank0	    ; Memoria común
	VARIABLE_TMR1:	DS 1	    ; Variable que almacena el valor a incrementar (AUMENTADOR)
	NIBBLES:	DS 2	    ; Variable para separar los nibbles del aumentador
	BANDERA:	DS 1	    ; Variable para verificar el display activo
	VALOR_HEX:	DS 2	    ; Variable con el valor del aumentador en hexadecimal
      
    ;-------------------------- VECTOR RESET -----------------------------------
    PSECT resVect, class=CODE, abs, delta=2
    ORG 00h			    ; Posición 0000h para el reset
    resetVec:
        PAGESEL main		    ; Cambio de pagina
        GOTO    main

    ;-------------------- SUBRUTINAS DE INTERRUPCION ---------------------------
    ORG 04h			    ; Posición 0004h para las interrupciones
    PUSH:			
	MOVWF   W_TEMP		    ; Guardamos en W el valor de la variable W_TEMP
	SWAPF   STATUS, W	    ; Hacemos swap de nibbles y guardamos en W
	MOVWF   STATUS_TEMP	    ; Almacenamos W en variable STATUS_TEMP
	
    ISR:			    ; Verificación de banderas de las interrupciones
    
	BTFSC	T0IF		    ; Verificamos si hay interrupción del TIMER0
	CALL	INT_TMR0	    ; Subrutina INT_TMR0
	
	BTFSC   TMR1IF		    ; Verificamos si hay interrupción del TIMER1
	CALL    INT_TMR1	    ; Subrutina INT_TMR1
	
	BTFSC   TMR2IF		    ; Verificamos si hay interrupción del TIMER2
	CALL    INT_TMR2	    ; Subrutina INT_TMR2
	
    POP:				
	SWAPF   STATUS_TEMP, W	    ; Hacemos swap de nibbles y guardamos en W
	MOVWF   STATUS		    ; Trasladamos W al registro STATUS
	SWAPF   W_TEMP, F	    ; Hacemos swap de nibbles y guardamos en W_TEMP 
	SWAPF   W_TEMP, W	    ; Hacemos swap de nibbles y guardamos en W
	RETFIE
    
    ;------------------------ RUTINAS PRINCIPALES ------------------------------
    PSECT code, delta=2, abs
    ORG 100h	; posición 100h para el codigo
    
    main:
	CALL	CONFIG_IO	    ; Comfiguración de los puertos	
	CALL	CONFIG_CLK	    ; Configuración del oscilador
	CALL	CONIFG_INTERRUPT    ; Configuracion de interrupciones
	CALL	CONFIG_TIMER0	    ; Configuración de TMR0
	CALL	CONFIG_TIMER1	    ; Configuración de TMR1
	CALL	CONFIG_TIMER2	    ; Configuración de TMR2
	BANKSEL PORTD		    ; Direccionamiento a banco 00
	CLRF	VARIABLE_TMR1
	
    loop:			    ; Rutina que se estará ejecutando indefinidamente
	MOVF	VARIABLE_TMR1, W    ; Colocamos el valor de la variable en W
	MOVWF	PORTA		    ; Mostramos el aumento de la variable en el PORTA
	
	CALL	OBTENER		    ; Rutina para obtener el valor de la variable en aumento
	CALL	PREPARAR	    ; Rutina de preparación/conversión a hexadecimal de la variable
	
	GOTO	loop		    ; Volvemos a comenzar con el loop
	
    ;--------------------------- SUBRUTINAS VARIAS -----------------------------
    INT_TMR0:
	RESET_TIMER0 254	    ; Ingresamos a Macro con valor 254 para configurar retardo de 2 ms
	CALL	MOSTRAR		    ; Llamamos a subrutina para colocar valores en displays
	return
	
    INT_TMR1:
	RESET_TIMER1 0xC2, 0xF7	    ; Macro para configurar el TIMER1
	INCF	VARIABLE_TMR1
	return

    INT_TMR2:
	BCF	TMR2IF		    ; Limpieza de la bandera de TMR2
	MOVLW   0x01		    ; Colocamos el valor 0x01 en W
	XORWF   PORTE		    ; Hacemos XOR del PORTE con W para solo mantener 1 bit
	return
	
    OBTENER:
	MOVLW   0x0F		    ; Colocar el valor 0x0F en registro W
	ANDWF   VARIABLE_TMR1, W    ; Hacer un AND de 0xF con la variable CONTADOR
	MOVWF   NIBBLES		    ; Almacenar el valor de W en variable NIBBLES posición 0
	
	MOVLW   0xF0		    ; Colocar el valor 0xF0 en registro W
	ANDWF   VARIABLE_TMR1, W    ; Hacer un AND de 0xF con la variable CONTADOR
	MOVWF   NIBBLES+1	    ; Almacenar el valor de W en variable NIBBLES posición 1
	SWAPF   NIBBLES+1, F	    ; Hacer un SWAP de nibbles de la variable NIBBLES posición 1
	return
	
    PREPARAR:
	MOVF    NIBBLES, W	    ; Colocamos el valor de NIBBLES (posición 0) en W
	CALL    TABLA		    ; Transformamos el valor a enviar a display
	MOVWF   VALOR_HEX	    ; Guardamos en variable DISPLAY

	MOVF    NIBBLES+1, W	    ; Colocamos el valor de NIBBLES (posición 1) en W
	CALL    TABLA		    ; Transformamos el valor a enviar a display
	MOVWF   VALOR_HEX+1	    ; Guardamos en variable DISPLAY
	return
	
    MOSTRAR:
	BCF	PORTD, 0	    ; Apagamos display de nibble alto
	BCF	PORTD, 1	    ; Apagamos display de nibble bajo
	BTFSC   BANDERA, 0	    ; Verificamos bandera
	GOTO    DISPLAY_1	 

	DISPLAY_0:			
	    MOVF    VALOR_HEX, W    ; Colocamos el valor de variable DISPLAY en W
	    MOVWF   PORTC	    ; Colocamos el valor de W en Puerto C
	    BSF	    PORTD, 1	    ; Activamos el primer display
	    BSF	    BANDERA, 0	    ; Alternamos el valor de la BANDERA para el siguiente ciclo
	return

	DISPLAY_1:
	    MOVF    VALOR_HEX+1, W  ; Colocamos el valor de variable DISPLAY en W
	    MOVWF   PORTC	    ; Colocamos el valor de W en Puerto C
	    BSF	    PORTD, 0	    ; Activamos el segundo display
	    BCF	    BANDERA, 0	    ; Alternamos el valor de la BANDERA para el siguiente ciclo
	return
	
    ;--------------------- SUBRUTINAS DE CONFIGURACIÓN -------------------------
    CONFIG_TIMER0:
	BANKSEL OPTION_REG	    ; Redireccionamos de banco
	BCF	T0CS		    ; Configuramos al timer0 como temporizador
	BCF	PSA		    ; Configurar el Prescaler para el timer0 (No para el Wathcdog timer)
	BSF	PS2
	BSF	PS1
	BCF	PS0		    ; PS<2:0> -> 110 (Prescaler 1:128)
	
	; Cálculo del valor a ingresar al TIMER1 para que tenga retardo de 1.5 ms
	; N = 256 - (Temp/(4 x Tosc x Presc))
	; N = 256 - (2 ms/(4 x (1/500 kHz) x 128))
	; N = 254
	
	RESET_TIMER0 254	    ; Reiniciamos la bandera interrupción
	return

    CONFIG_TIMER1:
	BANKSEL T1CON		    ; Direccionamos al banco correcto
	BCF	TMR1CS		    ; Activamos el uso de reloj interno
	BCF	T1OSCEN		    ; Apagamos LP
	BSF	T1CKPS1		    ; Prescaler 1:8
	BSF	T1CKPS0
	BCF	TMR1GE		    ; Mantenemos al TMR1 siempre contando
	BSF	TMR1ON		    ; Activamos al TMR1
	
	; Cálculo del valor a ingresar al TIMER1 para que tenga retardo de 1 s
	; N = 65536 - (Td/(Pre x Ti))
	; N = 65536 - ((1)/(8 x (1/(500 kHz)/4)))
	; N = 49911
	; TMR1H: 11000010 = 0xC2
	; TMR1L: 11110111 = 0xF7
	 
	RESET_TIMER1 0xC2, 0xF7	    ; Macro para configurar el TIMER1
	return
	
    CONFIG_TIMER2:
	BANKSEL T2CON		    ; Direccionamos al banco correcto
	BSF	T2CKPS1		    ; Prescaler 1:16
	BSF	T2CKPS0
	BSF	TOUTPS3		    ; Postscaler 1:16
	BSF	TOUTPS2
	BSF	TOUTPS1
	BSF	TOUTPS0
	BSF	TMR2ON		    ; Activamos al TMR1
	
	; Cálculo del valor a ingresar al TIMER2 para que tenga retardo de 500 ms
	; PR2 = (Ttmr2)/(Pres*Postc*(4/Fosc))
	; PR2 = (0.5)/(16*16*(4/500*10^3))
	; PR2 = 244
	
	BANKSEL PR2		    ; Direccionamos al banco correcto
	MOVLW   244		    ; Valor necesario para retardo de 500 ms
	MOVWF   PR2		    ; Cargamos litaral al bit PR2
	return
    
    CONFIG_CLK:			    ; Rutina de configuración de oscilador
	BANKSEL OSCCON	    
	BSF	OSCCON, 0
	BSF	OSCCON, 4
	BSF	OSCCON, 5
	BCF	OSCCON, 6	    ; Oscilador con reloj de 500 kHz
	return
	
    CONIFG_INTERRUPT:
	BANKSEL PIE1		    ; Redireccionamos de banco
	BSF	T0IE		    ; Habilitamos la interrupción del TMR0
	BSF	TMR1IE		    ; Habilitamos la interrupción del TMR1
	BSF     TMR2IE		    ; Habilitamos la interrupción del TMR2
	
	BANKSEL	INTCON		    ; Redireccionamos de banco
	BSF	PEIE		    ; Habilitamos interrupciones en periféricos
	BSF	GIE		    ; Habilitamos a todas las interrupciones
	BCF	T0IF		    ; Limpieza de la bandera de TMR0
	BCF	TMR1IF		    ; Limpieza de la bandera de TMR1
	BCF	TMR2IF		    ; Limpieza de la bandera de TMR2
	RETURN
	
    CONFIG_IO:
	BANKSEL ANSEL		    ; Direccionamos de banco
	CLRF    ANSEL		    ; Configurar como digitales
	CLRF    ANSELH		    ; Configurar como digitales
	
	BANKSEL TRISA		    ; Direccionamos de banco
	CLRF	TRISA		    ; Habilitamos al PORTA como salida
	CLRF	TRISC		    ; Habilitamos al PORTC como salida
	BCF	TRISD, 0	    ; Habilitamos al PORTD,0 como salida
	BCF	TRISD, 1	    ; Habilitamos al PORTD,1 como salida
	BCF	TRISE, 0	    ; Habilitamos al PORTE,0 como salida
	
	BANKSEL PORTA		    ; Direccionar de banco
	CLRF    PORTA		    ; Limpieza de PORTA
	CLRF	PORTC		    ; Limpieza de PORTC
	CLRF	PORTD		    ; Limpieza de PORTD
	CLRF	PORTE		    ; Limpieza de PORTE
	return
	
    ;------------------------ TABLA  HEXADECIMAL -------------------------------
    ORG 200h
    TABLA:
	CLRF    PCLATH		; Limpiamos registro PCLATH
	BSF	PCLATH, 1	; Posicionamos el PC en dirección 02xxh
	ANDLW   0x0F		; no saltar más del tamaño de la tabla
	ADDWF   PCL		; Apuntamos el PC a caracter en ASCII de CONT
	RETLW   00111111B	; 0
	RETLW   00000110B	; 1
	RETLW   01011011B	; 2
	RETLW   01001111B	; 3
	RETLW   01100110B	; 4
	RETLW   01101101B	; 5
	RETLW   01111101B	; 6
	RETLW   00000111B	; 7
	RETLW   01111111B	; 8
	RETLW   01101111B	; 9
	RETLW   01110111B	; A
	RETLW   01111100B	; b
	RETLW   00111001B	; C
	RETLW   01011110B	; d
	RETLW   01111001B	; E
	RETLW   01110001B	; F
    END


