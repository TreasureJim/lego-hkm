#include <cmath>

void cyl_pol_to_cart(double cyl[3], double cart[3]) {
  cart[0] = cyl[0] * sin(cyl[1]);
  cart[1] = cyl[0] * cos(cyl[1]);
  cart[2] = cyl[2];
}

void cart_to_cyl_pol(double cart[3], double cyl[3]) {
  cyl[0] = sqrt(pow(cart[0], 2) + pow(cart[1], 2));

  if (cart[0] == 0.0 || cart[1] == 0.0) {
    cyl[1] = 0.0;
  } else {
    cyl[1] = atan(cart[0] / cart[1]);
  }

  cyl[2] = cart[2];
}
