#pragma once

#include <cmath>

namespace rig
{
namespace barometry
{
// with physical constants from https://en.wikipedia.org/wiki/Barometric_formula
const double M = 0.0289644;  // molar mass of Earth's air [kg/mol]
const double R = 8.3144598;  // gas constant [ N m/(mol K)]
const double g_0 = 9.80665;  // acceleration due to gravity [m/s^2]
const double T_0 = 288.15;   // standard temperature [K]
const double P_0 = 101325;   // standard pressure [Pa]
const double L_0 = 0.0065;   // Temperature lapse rate [K/m]

inline double height_from_pressure(const double & pressure)
{
  // according to
  // https://www.grc.nasa.gov/www/k-12/airplane/atmosmet.html
  return T_0 / L_0 * (1.0 - std::pow(pressure / P_0, (R * L_0) / (g_0 * M)));
}

inline double pressure_from_height(const double & height)
{
  // according to
  // https://www.grc.nasa.gov/www/k-12/airplane/atmosmet.html
  return P_0 * std::pow((T_0 - height * L_0) / T_0, (g_0 * M) / (R * L_0));
}

}  // namespace barometry
}  // namespace rig
