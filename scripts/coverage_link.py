# build_flags reach the compiler but not the native test link line, so the
# gcov runtime goes missing. Add --coverage to the linker explicitly.
Import("env")  # noqa: F821

env.Append(LINKFLAGS=["--coverage"])  # noqa: F821
