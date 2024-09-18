#include <glim/mapping/localization.hpp>

extern "C" glim::GlobalMappingBase* create_global_mapping_module() {
  glim::LocalizationParams params;
  return new glim::Localization(params);
}
