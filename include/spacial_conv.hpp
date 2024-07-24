/*! \file spacial_conv.h
    \brief Functions for converting between different co-ordinate systems.

    The OMPL library searches for valid paths using the Cylidrical Polar Co-ordinate system, see [Wikipedia](https://en.wikipedia.org/wiki/Cylindrical_coordinate_system) for more details. 
    The phi angle should **always** be represented using *radians*. 

    The rest of the system uses Cartesian co-ordinates. 
*/

#pragma once

#define CYL_POL(p, phi, z) {p, phi, z}

/// Transforms cylidrical polar co-ordinates to cartesian co-ordinates
///
/// @param cyl cylindrical co-ordinates in form `[p, phi, z]`
/// @param cart cartesian co-ordinates in form `[x, y, z]`
void cyl_pol_to_cart(double cyl[3], double cart[3]);

/// Transforms cartesian co-ordinates to cylidrical polar co-ordinates 
///
/// @param cart cartesian co-ordinates in form `[x, y, z]`
/// @param cyl cylindrical co-ordinates in form `[p, phi, z]`
void cart_to_cyl_pol(double cart[3], double cyl[3]);
