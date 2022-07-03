#include "main.h"
#include "subsystems/pneumatics.hpp"

int kPneumaticIndexerPort = 0;
int kPneumaticExpansionPort = 0;

void index() {
  pros::c::adi_digital_write(kPneumaticIndexerPort, HIGH);
  pros::delay(500);
  pros::c::adi_digital_write(kPneumaticIndexerPort, LOW);
}

void expand() {
  pros::c::adi_digital_write(kPneumaticExpansionPort, HIGH);
}
