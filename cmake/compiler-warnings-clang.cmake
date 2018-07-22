include(CheckCXXCompilerFlagAndEnableIt)

if(NOT CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
  return()
endif()

set (CLANG_WARNING_FLAGS
  "all"
  "extra"
  "everything"
)

set (CLANG_DISABLED_WARNING_FLAGS
  "c++98-compat"
  "c++98-compat-pedantic"
  "conversion" # FIXME: really need to enable this one
  "covered-switch-default"
  "deprecated"
  "double-promotion"
  "exit-time-destructors"
  "global-constructors"
  "gnu-zero-variadic-macro-arguments"
  "old-style-cast"
  "padded"
  "switch-enum"
  "unused-macros"
  "unused-parameter"
  "weak-vtables"
  "zero-as-null-pointer-constant" # temporary
)

set (CLANG_NO_ERROR_WARNING_FLAGS
)

# Yes, these have to be *re-enabled* after CLANG_DISABLED_WARNING_FLAGS.
set (CLANG_REENABLED_WARNING_FLAGS
  "extra-semi"
)

set(CMAKE_REQUIRED_FLAGS_ORIG "${CMAKE_REQUIRED_FLAGS}")
set(CMAKE_REQUIRED_FLAGS "-c -Wunreachable-code -Werror=unreachable-code")
# see https://reviews.llvm.org/D25321
# see https://github.com/darktable-org/rawspeed/issues/104
CHECK_CXX_SOURCE_COMPILES(
  "void foo() {
  return;
  __builtin_unreachable();
}"
  CLANG_CXX_FLAG_UNREACHABLE_CODE_WORKS
)
set(CMAKE_REQUIRED_FLAGS "${CMAKE_REQUIRED_FLAGS_ORIG}")

if(NOT CLANG_CXX_FLAG_UNREACHABLE_CODE_WORKS)
  list(APPEND CLANG_DISABLED_WARNING_FLAGS "unreachable-code")
endif()

if(NOT (UNIX OR APPLE))
  # bogus warnings about std functions...
  list(APPEND CLANG_DISABLED_WARNING_FLAGS "used-but-marked-unused")
  # just don't care.
  list(APPEND CLANG_DISABLED_WARNING_FLAGS "nonportable-system-include-path")
endif()

if(CMAKE_CXX_COMPILER_ID STREQUAL "Clang" AND
   RAWSPEED_ENABLE_INSTR_PROFILE STREQUAL "PGOUSE")
 # Some source files are just not covered at all
 list(APPEND CLANG_NO_ERROR_WARNING_FLAGS "profile-instr-unprofiled")
endif()

foreach(warning ${CLANG_WARNING_FLAGS})
  CHECK_CXX_COMPILER_FLAG_AND_ENABLE_IT(-W${warning})
endforeach()

foreach(warning ${CLANG_DISABLED_WARNING_FLAGS})
  CHECK_CXX_COMPILER_FLAG_AND_ENABLE_IT(-Wno-${warning})
endforeach()

foreach(warning ${CLANG_NO_ERROR_WARNING_FLAGS})
  CHECK_CXX_COMPILER_FLAG_AND_ENABLE_IT(-Wno-error=${warning})
endforeach()

foreach(warning ${CLANG_REENABLED_WARNING_FLAGS})
  CHECK_CXX_COMPILER_FLAG_AND_ENABLE_IT(-W${warning})
endforeach()
