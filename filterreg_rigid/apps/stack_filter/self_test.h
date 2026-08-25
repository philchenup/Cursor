#pragma once

#include "stack_filter/stacked_object_filter.h"

#include <string>

namespace poser {

// Returns 0 on success. Prints failing cases to stderr.
int RunStackFilterSelfTest();

}  // namespace poser
