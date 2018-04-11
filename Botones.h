/*
 * Teclado.h

 *
 *  Created on: 11/08/2017
 *      Author: Sergio Chung, Francisco Avelar
 */
#ifndef BOTONES_H_
#define BOTONES_H_

#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "MK64F12.h"
#include "fsl_port.h"
#include "fsl_gpio.h"

/*Masks for the values from the 6 buttons*/
#define BOTON_B0_MASK (1)
#define BOTON_B1_MASK (2)
#define BOTON_B2_MASK (4)
#define BOTON_B3_MASK (8)

typedef enum
{
    BUTTON_0, BUTTON_1, BUTTON_2, BUTTON_3, NO_BUTTON
} Butons;

/**
 \brief
 Esta funcion inicializa los puerto GPIO para los botones y sus interrupciones
 \return void
 */
void inicializacionBotones ();

#endif /*DATATYPEDIFINITIONS_H_*/

