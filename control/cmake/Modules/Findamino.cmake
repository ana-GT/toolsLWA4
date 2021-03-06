# *********************************************************
# Find amino library
# Once done this will define:
# AMINO_FOUND - System has amino installed
# AMINO_INCLUDE_DIRS - The amino include directories
# AMINO_LIBRARIES - The libraries needed to use amino
# *********************************************************

find_path( amino_INCLUDE_DIR amino.h HINTS /usr/local/include ~/local/include )
find_library( amino_LIBRARY amino HINTS /usr/local/lib ~/local/lib )

set( AMINO_LIBRARIES ${amino_LIBRARY} )
set( AMINO_INCLUDE_DIRS ${amino_INCLUDE_DIR} )

include( FindPackageHandleStandardArgs )

# Handle the QUIETLY and REQUIRED arguments and set AMINO_FOUND to TRUE
#if all listed variables are TRUE
find_package_handle_standard_args( amino DEFAULT_MSG
  amino_LIBRARY amino_INCLUDE_DIR )
mark_as_advanced( amino_INCLUDE_DIR amino_LIBRARY )
