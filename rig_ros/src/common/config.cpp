#include "rig/common/config.hpp"

namespace rig
{
void declare_config(BarometerConfig & cfg)
{
  using namespace config;
  name("BarometerConfig");
  field(cfg.enabled, "enabled");
  field(cfg.sigma, "sigma");
  field(cfg.bias_sigma, "bias_sigma");
  field(cfg.robust_cost, "robust_cost", "none|fair|huber|etc");
}
}  // namespace rig
