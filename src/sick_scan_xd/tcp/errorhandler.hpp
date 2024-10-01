/**
 * Fehler-Handler.
 *
 */
#pragma once

#include <stdexcept>	// for throw

#include "BasicDatatypes.hpp"


#define printInfoMessage(a, b)  (b ? infoMessage(a, b):doNothing())

// Fehler-"behandlung": Schreibe die Fehlermeldung und beende das Programm.
void dieWithError(std::string errorMessage);

void infoMessage(std::string message, bool print = true);

void printWarning(std::string message);

void printError(std::string message);

void doNothing();
