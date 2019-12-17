# Firmware drajvera za kretanje

Ovo je source code drajvera za kretanje robota. Drajver se satoji od dva H mosta kojima se upravlja DC (Maxon) motorima. Postoji verzija i za malog i za velikog.

IDE:      MPLAB-X                                                                                                       
Compiler: MPLAB XC16 Compiler

For Linux people:
https://vtluug.org/wiki/PICKit_2

## Update za 1.0

- Poslednja verzija koda u kojoj su otklonjeni bagovi koji su nastajali zbog veoma dugih izraza koji su bili predstavljeni kao makroi pa je kontroleru trebalo veoma dugo vremena da izracunava svaki put sto je prouzrokovalo greske.Postoji verzija i za malog i za velikog.

## Update za 1.1

- Stavljena funkcija za gasenje PWM-a preko CAN

## Update 2.0
- converted to english
- commented code 99%
- same functionality as v1.x, just with clearer names/defines
- removed files/function that were not in use

### Version
2.0
