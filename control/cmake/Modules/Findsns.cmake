# *********************************************************
# Find sns library
# Once done this will define:
# SNS_FOUND - System has reflex installed
# SNS_INCLUDE_DIRS - The reflex include directories
# SNS_LIBRARIES - The libraries needed to use reflex
# *********************************************************

find_path( sns_INCLUDE_DIR sns.h HINTS /usr/local/include ~/local/include )
find_library( sns_LIBRARY sns HINTS /usr/local/include ~/local/lib )

set( SNS_LIBRARIES ${sns_LIBRARY} )
set( SNS_INCLUDE_DIRS ${sns_INCLUDE_DIR} )

include( FindPackageHandleStandardArgs )

# Handle the QUIETLY and REQUIRED arguments and set AMINO_FOUND to TRUE
#if all listed variables are TRUE
find_package_handle_standard_args( sns DEFAULT_MSG
  sns_LIBRARY sns_INCLUDE_DIR )
mark_as_advanced( sns_INCLUDE_DIR sns_LIBRARY )
